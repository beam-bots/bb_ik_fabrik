# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK do
  @moduledoc """
  FABRIK-based inverse kinematics solver for serial chains.

  FABRIK (Forward And Backward Reaching Inverse Kinematics) is an iterative
  solver that works by alternately reaching from the end-effector toward the
  target, then from the base back to maintain segment lengths. This implementation
  extends classic FABRIK with orientation tracking at each joint frame.

  ## Features

  - Works with `BB.Robot.State` or plain position maps
  - Position and orientation solving (quaternion or axis constraints)
  - Handles co-located joints by fitting each about its own axis
  - Respects joint limits by clamping solved values
  - Uses Nx tensors for efficient computation
  - Returns best-effort positions even on failure

  ## Usage

      robot = MyRobot.robot()
      {:ok, state} = BB.Robot.State.new(robot)

      # Solve for end-effector to reach target position
      target = Vec3.new(0.4, 0.2, 0.1)

      case BB.IK.FABRIK.solve(robot, state, :end_effector, target) do
        {:ok, positions, meta} ->
          BB.Robot.State.set_configurations(state, configurations)
          IO.puts("Solved in \#{meta.iterations} iterations")

        {:error, %BB.Error.Kinematics.Unreachable{residual: residual}} ->
          IO.puts("Target unreachable, residual: \#{residual}m")
      end

  ## Target Formats

  - `Vec3.t()` - Position-only target
  - `Transform.t()` - Position + full orientation from transform
  - `{Vec3.t(), {:quaternion, Quaternion.t()}}` - Position + explicit quaternion
  - `{Vec3.t(), {:axis, Vec3.t()}}` - Position + tool axis direction constraint

  ## Options

  - `:max_iterations` - Maximum solver iterations (default: 50)
  - `:tolerance` - Position convergence tolerance in metres (default: 1.0e-4)
  - `:orientation_tolerance` - Orientation convergence tolerance in radians (default: 0.01)
  - `:respect_limits` - Whether to clamp to joint limits (default: true)

  ## Limitations

  - Serial chains only (no branching topologies)
  - Revolute and prismatic joints (fixed joints are skipped)
  - Only position targets respect joint axes. An orientation target still solves
    in point space and fits joint values afterwards, so its result may not
    correspond to a pose the robot can hold — `meta.reached` says which
  - Constraining costs iterations, so a solve takes tens rather than a handful.
    `BB.IK.FABRIK.Math.solve_constrained/4` vectorises over a batch axis when
    many chains need solving at once
  """

  @behaviour BB.IK.Solver

  alias BB.Error.Kinematics.Unreachable
  alias BB.IK.FABRIK.{Chain, Math}
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot
  alias BB.Robot.{Kinematics, State}

  @default_max_iterations 50
  @default_tolerance 1.0e-4
  @default_orientation_tolerance 0.01

  @impl true
  @spec solve(Robot.t(), State.t() | map(), atom(), atom(), BB.IK.Solver.target(), keyword()) ::
          BB.IK.Solver.solve_result()
  def solve(robot, state_or_configurations, source_link, target_link, target, opts \\ [])

  def solve(%Robot{} = robot, %State{} = state, source_link, target_link, target, opts) do
    configurations = State.get_all_configurations(state)
    solve(robot, configurations, source_link, target_link, target, opts)
  end

  def solve(%Robot{} = robot, positions, source_link, target_link, target, opts)
      when is_map(positions) do
    {target_point, orientation_target} = normalize_target(target)

    case Chain.build(robot, positions, source_link, target_link) do
      {:error, error} ->
        {:error, error}

      {:ok, chain} ->
        shortfall = reach_shortfall(chain, target_point)

        if shortfall > 0.0 do
          {:error,
           %Unreachable{
             target_link: target_link,
             target_pose: target,
             reason: "Target beyond workspace",
             iterations: 0,
             residual: shortfall,
             positions: positions
           }}
        else
          run(robot, chain, positions, target_point, orientation_target, target_link, opts)
        end
    end
  end

  defp run(robot, chain, positions, target_point, orientation_target, target_link, opts) do
    max_iterations = Keyword.get(opts, :max_iterations, @default_max_iterations)
    tolerance = Keyword.get(opts, :tolerance, @default_tolerance)

    orientation_tolerance =
      Keyword.get(opts, :orientation_tolerance, @default_orientation_tolerance)

    respect_limits? = Keyword.get(opts, :respect_limits, true)

    kinematics = Chain.kinematics(chain)
    kinematics = if respect_limits?, do: kinematics, else: Chain.without_limits(kinematics)

    {solved, iterations, residual, orientation_residual} =
      Math.solve_constrained(
        kinematics,
        chain_frame(chain, target_point, orientation_target, robot, positions, target_link),
        %{
          max_iterations: max_iterations,
          tolerance: tolerance,
          orientation_tolerance: orientation_tolerance,
          # Trading a radian of orientation against a metre of position needs a
          # length, and the chain's own reach is the scale that matters. Much
          # shorter and the frame points sit almost on the end effector, so
          # orientation is outvoted by position; much longer and it drowns
          # position out. Half the reach sits in the middle of what works.
          lever: chain.reach / 2.0
        }
      )

    residual = Nx.to_number(residual)

    orientation_residual =
      if orientation_target == :none, do: nil, else: Nx.to_number(orientation_residual)

    meta = %{
      iterations: Nx.to_number(iterations),
      residual: residual,
      orientation_residual: orientation_residual,
      reached:
        residual <= tolerance and
          orientation_reached?(orientation_residual, orientation_tolerance)
    }

    {:ok, Map.merge(positions, Chain.configurations(chain, solved)), meta}
  end

  # The solver works wholly in the chain root's frame, so the target moves into
  # it rather than every link moving out of it.
  defp chain_frame(chain, target_point, orientation_target, robot, positions, target_link) do
    inverse = chain.root_transform |> Transform.inverse() |> Transform.tensor()
    homogeneous = Nx.concatenate([target_point, Nx.tensor([1.0], type: :f64)])
    rotation = target_rotation(orientation_target, robot, positions, target_link)

    %{
      position: inverse |> Nx.dot(homogeneous) |> Nx.slice([0], [3]),
      rotation: Nx.dot(inverse[[0..2, 0..2]], rotation),
      enforce: enforce_orientation(orientation_target)
    }
  end

  defp enforce_orientation(:none), do: Nx.tensor(0.0, type: :f64)
  defp enforce_orientation(_orientation), do: Nx.tensor(1.0, type: :f64)

  # An unenforced orientation still needs a well-formed rotation to travel beside
  # the position; the identity costs nothing and keeps one code path.
  defp target_rotation(:none, _robot, _positions, _target_link), do: Nx.eye(3, type: :f64)

  defp target_rotation({:quaternion, quaternion}, _robot, _positions, _target_link) do
    Quaternion.to_rotation_matrix(quaternion)
  end

  # An axis target names where the tool should point rather than a whole
  # orientation, so it resolves against the pose the robot is in now: the least
  # rotation swinging the tool's Z onto that axis.
  defp target_rotation({:axis, axis_vec}, robot, positions, target_link) do
    current =
      robot
      |> Kinematics.forward_kinematics(positions, target_link)
      |> Transform.get_quaternion()

    current
    |> Quaternion.rotate_vector(Vec3.unit_z())
    |> Quaternion.from_two_vectors(Vec3.normalise(axis_vec))
    |> Quaternion.multiply(current)
    |> Quaternion.to_rotation_matrix()
  end

  # How far past the chain's own reach the target sits. Positive means no pose
  # can touch it, however the joints are arranged.
  defp reach_shortfall(chain, target_point) do
    distance =
      target_point |> Nx.subtract(chain.root_point) |> Nx.LinAlg.norm() |> Nx.to_number()

    distance - chain.reach
  end

  @doc """
  Solve IK and update the state in-place.

  Convenience function that calls `solve/6` and applies the result
  to the given `BB.Robot.State`.

  Meant for a state of your own - one from `BB.Robot.State.new/1`, stepped
  through a planned motion. A running robot's state belongs to its sensors,
  which write it from `BB.Message.Sensor.JointState` messages, and writing a
  solved configuration into it claims the joints have arrived somewhere they
  have only been asked to go.

  ## Returns

  Same as `solve/6`, but on success the state's ETS table is updated.
  """
  @spec solve_and_update(
          Robot.t(),
          State.t(),
          atom(),
          atom(),
          BB.IK.Solver.target(),
          keyword()
        ) :: BB.IK.Solver.solve_result()
  def solve_and_update(
        %Robot{} = robot,
        %State{} = state,
        source_link,
        target_link,
        target,
        opts \\ []
      ) do
    case solve(robot, state, source_link, target_link, target, opts) do
      {:ok, configurations, meta} ->
        State.set_configurations(state, configurations)
        {:ok, configurations, meta}

      {:error, _error} = error ->
        error
    end
  end

  # Returns {position_tensor, orientation_target}
  defp normalize_target(%Vec3{} = vec) do
    {Vec3.tensor(vec), :none}
  end

  defp normalize_target({%Vec3{} = vec, orientation}) do
    {Vec3.tensor(vec), normalize_orientation(orientation)}
  end

  defp normalize_target(%Transform{} = transform) do
    pos_vec = Transform.get_translation(transform)
    orientation = {:quaternion, Transform.get_quaternion(transform)}
    {Vec3.tensor(pos_vec), orientation}
  end

  defp normalize_orientation(:none), do: :none
  defp normalize_orientation({:axis, %Vec3{} = vec}), do: {:axis, vec}
  defp normalize_orientation({:quaternion, %Quaternion{} = q}), do: {:quaternion, q}

  defp orientation_reached?(nil, _tolerance), do: true
  defp orientation_reached?(residual, tolerance), do: residual <= tolerance
end
