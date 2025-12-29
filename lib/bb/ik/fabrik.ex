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
  - Handles co-located joints via orientation-based angle extraction
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
          BB.Robot.State.set_positions(state, positions)
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
  - Orientation solving is heuristic and may not find optimal solutions for all geometries
  """

  @behaviour BB.IK.Solver

  alias BB.Error.Kinematics.NoSolution
  alias BB.Error.Kinematics.Unreachable
  alias BB.IK.FABRIK.{Chain, Math}
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot
  alias BB.Robot.{Kinematics, State}

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
    orientation_tolerance = Keyword.get(opts, :orientation_tolerance, 0.01)
    respect_limits? = Keyword.get(opts, :respect_limits, true)

    {target_point, orientation_target} = normalize_target(target)

    case Chain.build(robot, positions, target_link) do
      {:error, error} ->
        {:error, error}

      {:ok, chain} ->
        result =
          run_fabrik(
            chain,
            target_point,
            orientation_target,
            max_iterations,
            tolerance,
            orientation_tolerance
          )

        case result do
          {:ok, solved_data, fabrik_meta} ->
            joint_positions = extract_joint_positions(robot, chain, solved_data, respect_limits?)
            merged_positions = Map.merge(positions, joint_positions)
            residual = compute_residual(robot, merged_positions, target_link, target_point)

            meta = %{
              iterations: fabrik_meta.iterations,
              residual: residual,
              orientation_residual: fabrik_meta[:orientation_residual],
              reached: true
            }

            {:ok, merged_positions, meta}

          {:error, :unreachable, fabrik_meta} ->
            joint_positions = extract_joint_positions(robot, chain, fabrik_meta, respect_limits?)
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
            joint_positions = extract_joint_positions(robot, chain, fabrik_meta, respect_limits?)
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

  # Always use orientation-aware FABRIK internally, even for position-only targets.
  # This tracks orientations at each joint, enabling proper angle extraction for
  # co-located joints (like spherical shoulders/wrists) via frames_to_positions.
  defp run_fabrik(
         chain,
         target_point,
         orientation_target,
         max_iterations,
         tolerance,
         orientation_tolerance
       ) do
    frames = Chain.to_frames(chain)

    target_quaternion =
      case orientation_target do
        :none -> nil
        _ -> orientation_to_quaternion(orientation_target, frames)
      end

    result =
      Math.fabrik_with_orientation(
        frames,
        chain.lengths,
        target_point,
        target_quaternion,
        max_iterations,
        tolerance,
        orientation_tolerance: orientation_tolerance
      )

    case result do
      {:ok, frames, meta} -> {:ok, frames, meta}
      {:error, reason, meta} -> {:error, reason, Map.put(meta, :frames, meta.frames)}
    end
  end

  defp orientation_to_quaternion({:quaternion, q}, _frames), do: q

  defp orientation_to_quaternion({:axis, axis_vec}, frames) do
    # Get current end-effector orientation (last in chain)
    n = Nx.axis_size(frames.orientations, 0)

    current_quat_tensor =
      Nx.slice(frames.orientations, [n - 1, 0], [1, 4]) |> Nx.squeeze(axes: [0])

    current_quat = Quaternion.from_tensor(current_quat_tensor)

    # Current tool direction (Z-axis in end-effector's local frame)
    current_z = Quaternion.rotate_vector(current_quat, Vec3.unit_z())

    # Normalise target axis
    target_axis = Vec3.normalise(axis_vec)

    # Compute minimum rotation to align current Z with target axis
    rotation = Quaternion.from_two_vectors(current_z, target_axis)

    # Apply rotation to current orientation to get target orientation
    Quaternion.multiply(rotation, current_quat)
  end

  # Success - frames map with positions and orientations
  defp extract_joint_positions(
         robot,
         chain,
         %{positions: _, orientations: _} = frames,
         respect_limits?
       ) do
    Chain.frames_to_positions(robot, chain, frames, respect_limits?)
  end

  # Error case - meta map contains :frames key
  defp extract_joint_positions(robot, chain, %{frames: frames}, respect_limits?) do
    extract_joint_positions(robot, chain, frames, respect_limits?)
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

  defp normalize_target(%Transform{} = transform) do
    pos_vec = Transform.get_translation(transform)
    orientation = {:quaternion, Transform.get_quaternion(transform)}
    {Vec3.tensor(pos_vec), orientation}
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
