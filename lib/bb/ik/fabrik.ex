# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK do
  @moduledoc """
  FABRIK-based inverse kinematics solver for serial chains.

  FABRIK (Forward And Backward Reaching Inverse Kinematics) is an iterative
  solver that works by alternately reaching from the end-effector toward the
  target, then from the base back to maintain segment lengths.

  ## Features

  - Works with `BB.Robot.State` or plain position maps
  - Respects joint limits by clamping solved values
  - Uses Nx tensors for efficient computation
  - Returns best-effort positions even on failure

  ## Usage

      robot = MyRobot.robot()
      {:ok, state} = BB.Robot.State.new(robot)

      # Solve for end-effector to reach target position
      target = {0.4, 0.2, 0.1}

      case BB.IK.FABRIK.solve(robot, state, :end_effector, target) do
        {:ok, positions, meta} ->
          BB.Robot.State.set_positions(state, positions)
          IO.puts("Solved in \#{meta.iterations} iterations")

        {:error, %BB.Error.Kinematics.Unreachable{residual: residual}} ->
          IO.puts("Target unreachable, residual: \#{residual}m")
      end

  ## Options

  - `:max_iterations` - Maximum solver iterations (default: 50)
  - `:tolerance` - Convergence tolerance in metres (default: 1.0e-4)
  - `:respect_limits` - Whether to clamp to joint limits (default: true)

  ## Limitations

  - Position-only solving (orientation not yet supported)
  - Serial chains only (no branching topologies)
  - Revolute and prismatic joints (fixed joints are skipped)
  """

  @behaviour BB.IK.Solver

  alias BB.Error.Kinematics.NoSolution
  alias BB.Error.Kinematics.Unreachable
  alias BB.IK.FABRIK.{Chain, Math}
  alias BB.Quaternion
  alias BB.Robot
  alias BB.Robot.{Kinematics, State, Transform}
  alias BB.Vec3

  @default_max_iterations 50
  @default_tolerance 1.0e-4

  @impl true
  @spec solve(Robot.t(), State.t() | map(), atom(), BB.IK.Solver.target(), keyword()) ::
          BB.IK.Solver.solve_result()
  def solve(robot, state_or_positions, target_link, target, opts \\ [])

  def solve(%Robot{} = robot, %State{} = state, target_link, target, opts) do
    positions = State.get_all_positions(state)
    solve(robot, positions, target_link, target, opts)
  end

  def solve(%Robot{} = robot, positions, target_link, target, opts) when is_map(positions) do
    max_iterations = Keyword.get(opts, :max_iterations, @default_max_iterations)
    tolerance = Keyword.get(opts, :tolerance, @default_tolerance)
    respect_limits? = Keyword.get(opts, :respect_limits, true)

    {target_point, _orientation_target} = normalize_target(target)

    case Chain.build(robot, positions, target_link) do
      {:error, error} ->
        {:error, error}

      {:ok, chain} ->
        result =
          Math.fabrik(chain.points, chain.lengths, target_point, max_iterations, tolerance)

        case result do
          {:ok, solved_points, fabrik_meta} ->
            joint_positions =
              Chain.points_to_positions(robot, chain, solved_points, respect_limits?)

            merged_positions = Map.merge(positions, joint_positions)
            residual = compute_residual(robot, merged_positions, target_link, target_point)

            meta = %{
              iterations: fabrik_meta.iterations,
              residual: residual,
              orientation_residual: nil,
              reached: true
            }

            {:ok, merged_positions, meta}

          {:error, :unreachable, fabrik_meta} ->
            joint_positions =
              Chain.points_to_positions(robot, chain, fabrik_meta.points, respect_limits?)

            merged_positions = Map.merge(positions, joint_positions)
            residual = compute_residual(robot, merged_positions, target_link, target_point)

            {:error,
             %Unreachable{
               target_link: target_link,
               target_pose: target,
               reason: "Target beyond workspace",
               iterations: fabrik_meta.iterations,
               residual: residual,
               positions: merged_positions
             }}

          {:error, :max_iterations, fabrik_meta} ->
            joint_positions =
              Chain.points_to_positions(robot, chain, fabrik_meta.points, respect_limits?)

            merged_positions = Map.merge(positions, joint_positions)
            residual = compute_residual(robot, merged_positions, target_link, target_point)

            {:error,
             %NoSolution{
               target_link: target_link,
               target_pose: target,
               iterations: fabrik_meta.iterations,
               residual: residual,
               positions: merged_positions
             }}
        end
    end
  end

  @doc """
  Solve IK and update the state in-place.

  Convenience function that calls `solve/5` and applies the result
  to the given `BB.Robot.State`.

  ## Returns

  Same as `solve/5`, but on success the state's ETS table is updated.
  """
  @spec solve_and_update(Robot.t(), State.t(), atom(), BB.IK.Solver.target(), keyword()) ::
          BB.IK.Solver.solve_result()
  def solve_and_update(%Robot{} = robot, %State{} = state, target_link, target, opts \\ []) do
    case solve(robot, state, target_link, target, opts) do
      {:ok, positions, meta} ->
        State.set_positions(state, positions)
        {:ok, positions, meta}

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

  defp normalize_target(%Nx.Tensor{} = tensor) do
    case Nx.shape(tensor) do
      {4, 4} ->
        pos_vec = Transform.get_translation_vec3(tensor)
        orientation = {:quaternion, Transform.get_quaternion(tensor)}
        {Vec3.tensor(pos_vec), orientation}

      shape ->
        raise ArgumentError,
              "Invalid target tensor shape #{inspect(shape)}. Expected {4, 4}."
    end
  end

  defp normalize_orientation(:none), do: :none
  defp normalize_orientation({:axis, %Vec3{} = vec}), do: {:axis, vec}
  defp normalize_orientation({:quaternion, %Quaternion{} = q}), do: {:quaternion, q}

  defp compute_residual(robot, positions, target_link, target_point) do
    {x, y, z} = Kinematics.link_position(robot, positions, target_link)
    actual = Nx.tensor([x, y, z], type: :f64)
    Nx.subtract(actual, target_point) |> Nx.LinAlg.norm() |> Nx.to_number()
  end
end
