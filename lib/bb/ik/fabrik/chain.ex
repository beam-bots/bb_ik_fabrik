# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.Chain do
  @moduledoc false

  alias BB.Error.Kinematics.NoDofs
  alias BB.Error.Kinematics.UnknownLink
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Vec3
  alias BB.Robot
  alias BB.Robot.{Joint, Kinematics}

  defstruct [
    :joints,
    :joint_names,
    :joint_point_indices,
    :points,
    :orientations,
    :lengths,
    :limits,
    :root_transform,
    :original_positions
  ]

  @type t :: %__MODULE__{
          joints: [Joint.t()],
          joint_names: [atom()],
          joint_point_indices: [non_neg_integer()],
          points: Nx.Tensor.t(),
          orientations: Nx.Tensor.t(),
          lengths: Nx.Tensor.t(),
          limits: [Joint.limits() | nil],
          root_transform: Nx.Tensor.t(),
          original_positions: %{atom() => float()}
        }

  # Threshold for treating computed deltas as zero to avoid numerical drift
  # 1e-8 radians ≈ 5.7e-7 degrees - well below mechanical precision
  @delta_threshold 1.0e-8

  @doc """
  Build a kinematic chain from a robot topology for IK solving.

  Extracts the serial chain from root to the target link, computing
  joint positions in base frame and segment lengths.

  ## Parameters

  - `robot` - The BB.Robot struct
  - `positions` - Current joint positions as a map
  - `target_link` - The end-effector link name

  ## Returns

  - `{:ok, chain}` - Successfully built chain
  - `{:error, %UnknownLink{}}` - Target link not found
  - `{:error, %NoDofs{}}` - Chain has no movable joints
  """
  @spec build(Robot.t(), %{atom() => float()}, atom()) ::
          {:ok, t()} | {:error, UnknownLink.t() | NoDofs.t()}
  def build(%Robot{} = robot, positions, target_link) when is_map(positions) do
    case Robot.path_to(robot, target_link) do
      nil ->
        {:error, %UnknownLink{target_link: target_link}}

      path ->
        # Get all joints in the path
        all_joint_names =
          path
          |> Enum.filter(&Map.has_key?(robot.joints, &1))

        all_joints = Enum.map(all_joint_names, &Robot.get_joint(robot, &1))

        # Filter to only movable joints for the chain
        movable_joint_names =
          Enum.zip(all_joint_names, all_joints)
          |> Enum.filter(fn {_name, joint} -> Joint.movable?(joint) end)
          |> Enum.map(fn {name, _joint} -> name end)

        movable_joints = Enum.filter(all_joints, &Joint.movable?/1)

        if Enum.empty?(movable_joints) do
          {:error, %NoDofs{target_link: target_link, chain_length: length(path)}}
        else
          chain =
            build_chain_data(robot, positions, movable_joints, movable_joint_names, target_link)

          {:ok, chain}
        end
    end
  end

  @doc """
  Convert FABRIK solution points back to joint positions.

  Takes the solved point positions and computes the joint angles/distances
  that would produce those positions.

  ## Parameters

  - `robot` - The BB.Robot struct
  - `chain` - The original chain struct
  - `solved_points` - Nx tensor of solved positions `{n+1, 3}`
  - `respect_limits?` - Whether to clamp values to joint limits

  ## Returns

  Map of joint names to positions (radians for revolute, metres for prismatic).
  """
  @spec points_to_positions(Robot.t(), t(), Nx.Tensor.t(), boolean()) :: %{atom() => float()}
  def points_to_positions(%Robot{} = robot, %__MODULE__{} = chain, solved_points, respect_limits?) do
    chain.joint_names
    |> Enum.with_index()
    |> Enum.reduce(%{}, fn {joint_name, idx}, acc ->
      joint = Robot.get_joint(robot, joint_name)
      compute_and_add_position(acc, joint_name, idx, joint, chain, solved_points, respect_limits?)
    end)
  end

  @doc """
  Convert FABRIK solution frames back to joint positions.

  Like `points_to_positions/4` but accepts the frames map format
  from `Math.fabrik_with_orientation/7`.

  ## Parameters

  - `robot` - The BB.Robot struct
  - `chain` - The original chain struct
  - `frames` - Map with `:positions` and `:orientations` tensors
  - `respect_limits?` - Whether to clamp values to joint limits

  ## Returns

  Map of joint names to positions (radians for revolute, metres for prismatic).
  """
  @spec frames_to_positions(Robot.t(), t(), map(), boolean()) :: %{atom() => float()}
  def frames_to_positions(%Robot{} = robot, %__MODULE__{} = chain, frames, respect_limits?) do
    chain.joint_names
    |> Enum.with_index()
    |> Enum.reduce(%{}, fn {joint_name, idx}, acc ->
      joint = Robot.get_joint(robot, joint_name)

      compute_and_add_position_from_frames(
        acc,
        joint_name,
        idx,
        joint,
        chain,
        frames,
        respect_limits?
      )
    end)
  end

  defp compute_and_add_position_from_frames(
         acc,
         _joint_name,
         _idx,
         %Joint{} = joint,
         _chain,
         _frames,
         _respect_limits?
       )
       when joint.type == :fixed do
    acc
  end

  defp compute_and_add_position_from_frames(
         acc,
         joint_name,
         idx,
         joint,
         chain,
         frames,
         respect_limits?
       ) do
    point_idx = Enum.at(chain.joint_point_indices, idx, 0)
    num_points = Nx.axis_size(frames.positions, 0)

    # Check if this joint shares its point with other joints (co-located)
    is_colocated = colocated_joint?(chain.joint_point_indices, idx)

    # Get segment length (from point_idx to point_idx+1)
    segment_length =
      if point_idx < Nx.axis_size(chain.lengths, 0) do
        chain.lengths[point_idx] |> Nx.to_number()
      else
        0.0
      end

    delta =
      cond do
        # Fixed joint - no delta
        joint.type == :fixed ->
          0.0

        # Out of bounds - no delta
        point_idx >= num_points - 1 ->
          0.0

        # Co-located joint or zero-length segment - use orientation-based extraction
        # This handles spherical shoulders/wrists where multiple joints share a point
        is_colocated or segment_length < 1.0e-6 ->
          compute_angle_from_orientations(chain, frames, point_idx, joint)

        # Non-zero segment with unique point - use position-based extraction
        true ->
          compute_joint_position(chain, frames.positions, point_idx, joint)
      end

    delta = if abs(delta) < @delta_threshold, do: 0.0, else: delta

    original_position = Map.get(chain.original_positions, joint_name, 0.0)
    position = original_position + delta
    clamped = maybe_clamp(position, joint.limits, respect_limits?)
    Map.put(acc, joint_name, clamped)
  end

  defp colocated_joint?(joint_point_indices, idx) do
    # A joint is co-located if another joint shares the same point index
    point_idx = Enum.at(joint_point_indices, idx)

    joint_point_indices
    |> Enum.with_index()
    |> Enum.any?(fn {other_point_idx, other_idx} ->
      other_idx != idx and other_point_idx == point_idx
    end)
  end

  defp compute_angle_from_orientations(chain, frames, point_idx, joint) do
    # For co-located joints, extract joint angle from the direction change.
    #
    # Note: We use direction-based extraction instead of orientation at the point
    # because FABRIK pins the root orientation in the forward pass. The direction
    # change captures the actual rotation that occurred.

    # Direction from current point to next point
    p_joint = get_point(frames.positions, point_idx)
    p_next = get_point(frames.positions, point_idx + 1)
    direction = Nx.subtract(p_next, p_joint)

    orig_p_joint = get_point(chain.points, point_idx)
    orig_p_next = get_point(chain.points, point_idx + 1)
    orig_direction = Nx.subtract(orig_p_next, orig_p_joint)

    # Compute angle between original and solved directions, projected onto joint axis
    joint_axis = joint.axis || {0.0, 0.0, 1.0}

    joint_axis_tensor =
      Nx.tensor([elem(joint_axis, 0), elem(joint_axis, 1), elem(joint_axis, 2)], type: :f64)

    # Project directions onto plane perpendicular to joint axis
    orig_projected = project_onto_plane(orig_direction, joint_axis_tensor)
    solved_projected = project_onto_plane(direction, joint_axis_tensor)

    orig_norm = Nx.LinAlg.norm(orig_projected) |> Nx.to_number()
    solved_norm = Nx.LinAlg.norm(solved_projected) |> Nx.to_number()

    # When either projection is near-zero (singularity), return 0
    # This happens when direction aligns with joint axis
    if orig_norm < 1.0e-10 or solved_norm < 1.0e-10 do
      0.0
    else
      orig_unit = Nx.divide(orig_projected, orig_norm)
      solved_unit = Nx.divide(solved_projected, solved_norm)

      dot = Nx.dot(orig_unit, solved_unit) |> Nx.to_number()
      dot = max(-1.0, min(1.0, dot))

      cross = cross_product(orig_unit, solved_unit)
      cross_dot_axis = Nx.dot(cross, joint_axis_tensor) |> Nx.to_number()

      angle = :math.acos(dot)
      if cross_dot_axis < 0, do: -angle, else: angle
    end
  end

  defp compute_and_add_position(
         acc,
         _joint_name,
         _idx,
         %Joint{} = joint,
         _chain,
         _solved_points,
         _respect_limits?
       )
       when joint.type == :fixed do
    acc
  end

  defp compute_and_add_position(
         acc,
         joint_name,
         idx,
         joint,
         chain,
         solved_points,
         respect_limits?
       ) do
    # Get the point index for this joint (handles co-located joints)
    point_idx = Enum.at(chain.joint_point_indices, idx, 0)
    num_points = Nx.axis_size(solved_points, 0)

    # Ensure we have valid indices: need point_idx and point_idx+1
    delta =
      if point_idx < num_points - 1 do
        compute_joint_position(chain, solved_points, point_idx, joint)
      else
        0.0
      end

    # Apply deadband to avoid numerical drift from tiny computed deltas
    delta = if abs(delta) < @delta_threshold, do: 0.0, else: delta

    original_position = Map.get(chain.original_positions, joint_name, 0.0)
    position = original_position + delta
    clamped = maybe_clamp(position, joint.limits, respect_limits?)
    Map.put(acc, joint_name, clamped)
  end

  defp maybe_clamp(position, limits, true), do: clamp_to_limits(position, limits)
  defp maybe_clamp(position, _limits, false), do: position

  defp build_chain_data(robot, positions, joints, joint_names, target_link) do
    transforms = Kinematics.all_link_transforms(robot, positions)

    # Build points from joint child links plus the final target link
    # This gives us meaningful segment lengths between joints
    #
    # For a chain: base -> shoulder_joint -> link1 -> elbow_joint -> link2 -> tip_joint -> tip
    # We want points at: [link1, link2, tip] with segments between them
    #
    # Each joint controls the angle at its parent link, affecting the child link position

    # Start with the first joint's parent link (the base/root)
    first_joint = List.first(joints)
    root_transform = transforms[first_joint.parent_link]
    root_point = Transform.get_translation(root_transform)
    root_orientation = extract_orientation(root_transform)

    # Get positions and orientations of each joint's child link
    child_link_data =
      Enum.map(joints, fn joint ->
        transform = transforms[joint.child_link]
        {Transform.get_translation(transform), extract_orientation(transform)}
      end)

    child_link_points = Enum.map(child_link_data, &elem(&1, 0))
    child_link_orientations = Enum.map(child_link_data, &elem(&1, 1))

    # The final point is the target link
    end_effector_transform = transforms[target_link]
    end_effector_point = Transform.get_translation(end_effector_transform)
    end_effector_orientation = extract_orientation(end_effector_transform)

    # Combine all points and orientations, deduplicating consecutive identical points
    # but tracking which original index maps to which deduplicated index
    all_points = [root_point] ++ child_link_points ++ [end_effector_point]
    all_orientations = [root_orientation] ++ child_link_orientations ++ [end_effector_orientation]

    # Remove consecutive duplicate points (within tolerance), keeping corresponding orientations
    # and tracking the index mapping
    {points_list, orientations_list, original_to_deduped} =
      dedupe_consecutive_points_with_orientations_and_mapping(all_points, all_orientations)

    # Build joint_point_indices: for each joint, which deduped point index does it control?
    # Joint j's position = child_link_points[j] = original index (j + 1)
    # After deduplication, we need the deduped index for original point (j + 1)
    joint_point_indices =
      Enum.with_index(joints)
      |> Enum.map(fn {_joint, j} ->
        Map.get(original_to_deduped, j + 1, 0)
      end)

    points =
      points_list
      |> Enum.map(fn %Vec3{} = v -> Vec3.to_list(v) end)
      |> Nx.tensor(type: :f64)

    orientations =
      orientations_list
      |> Enum.map(&Nx.to_flat_list/1)
      |> Nx.tensor(type: :f64)

    n = length(points_list)

    lengths =
      if n > 1 do
        0..(n - 2)
        |> Enum.map(fn i ->
          p1 = Nx.slice(points, [i, 0], [1, 3]) |> Nx.squeeze(axes: [0])
          p2 = Nx.slice(points, [i + 1, 0], [1, 3]) |> Nx.squeeze(axes: [0])
          Nx.subtract(p2, p1) |> Nx.LinAlg.norm() |> Nx.to_number()
        end)
        |> Nx.tensor(type: :f64)
      else
        Nx.tensor([], type: :f64)
      end

    limits = Enum.map(joints, & &1.limits)

    %__MODULE__{
      joints: joints,
      joint_names: joint_names,
      joint_point_indices: joint_point_indices,
      points: points,
      orientations: orientations,
      lengths: lengths,
      limits: limits,
      root_transform: root_transform,
      original_positions: positions
    }
  end

  defp extract_orientation(transform) do
    Quaternion.tensor(Transform.get_quaternion(transform))
  end

  @doc """
  Convert chain to frames format for use with Math.fabrik_with_orientation.
  """
  @spec to_frames(t()) :: %{positions: Nx.Tensor.t(), orientations: Nx.Tensor.t()}
  def to_frames(%__MODULE__{} = chain) do
    %{positions: chain.points, orientations: chain.orientations}
  end

  defp dedupe_consecutive_points_with_orientations_and_mapping(points, orientations) do
    combined = Enum.zip(points, orientations) |> Enum.with_index()

    {deduped, mapping, _last_point, _deduped_idx} =
      Enum.reduce(combined, {[], %{}, nil, 0}, fn {{point, ori}, orig_idx},
                                                  {acc, map, last_point, deduped_idx} ->
        if last_point == nil or not points_equal?(last_point, point) do
          # New unique point - add it and update mapping
          new_map = Map.put(map, orig_idx, deduped_idx)
          {[{point, ori} | acc], new_map, point, deduped_idx + 1}
        else
          # Duplicate point - map to previous deduped index
          new_map = Map.put(map, orig_idx, deduped_idx - 1)
          {acc, new_map, last_point, deduped_idx}
        end
      end)

    deduped = Enum.reverse(deduped)
    {Enum.map(deduped, &elem(&1, 0)), Enum.map(deduped, &elem(&1, 1)), mapping}
  end

  defp points_equal?(%Vec3{} = p1, %Vec3{} = p2) do
    tolerance = 1.0e-6

    abs(Vec3.x(p1) - Vec3.x(p2)) < tolerance and
      abs(Vec3.y(p1) - Vec3.y(p2)) < tolerance and
      abs(Vec3.z(p1) - Vec3.z(p2)) < tolerance
  end

  defp compute_joint_position(chain, solved_points, idx, joint) do
    # Points are: [root, child_of_joint0, child_of_joint1, ..., end_effector]
    # For joint at point index idx:
    # - The joint controls the direction from its point (idx) to the next point (idx+1)
    # - This represents the segment from the joint's child link to the next link
    p_joint = get_point(solved_points, idx)
    p_next = get_point(solved_points, idx + 1)

    orig_p_joint = get_point(chain.points, idx)
    orig_p_next = get_point(chain.points, idx + 1)

    case joint.type do
      type when type in [:revolute, :continuous] ->
        compute_revolute_angle(orig_p_joint, orig_p_next, p_joint, p_next, joint.axis)

      :prismatic ->
        compute_prismatic_distance(orig_p_joint, orig_p_next, p_joint, p_next, joint.axis)

      _ ->
        0.0
    end
  end

  defp compute_revolute_angle(orig_joint, orig_child, new_joint, new_child, axis) do
    axis = axis || {0.0, 0.0, 1.0}
    axis_tensor = Nx.tensor([elem(axis, 0), elem(axis, 1), elem(axis, 2)], type: :f64)

    orig_dir = Nx.subtract(orig_child, orig_joint)
    new_dir = Nx.subtract(new_child, new_joint)

    orig_projected = project_onto_plane(orig_dir, axis_tensor)
    new_projected = project_onto_plane(new_dir, axis_tensor)

    orig_norm = Nx.LinAlg.norm(orig_projected) |> Nx.to_number()
    new_norm = Nx.LinAlg.norm(new_projected) |> Nx.to_number()

    if orig_norm < 1.0e-10 or new_norm < 1.0e-10 do
      0.0
    else
      orig_unit = Nx.divide(orig_projected, orig_norm)
      new_unit = Nx.divide(new_projected, new_norm)

      dot = Nx.dot(orig_unit, new_unit) |> Nx.to_number()
      dot = max(-1.0, min(1.0, dot))

      cross = cross_product(orig_unit, new_unit)
      cross_dot_axis = Nx.dot(cross, axis_tensor) |> Nx.to_number()

      angle = :math.acos(dot)
      if cross_dot_axis < 0, do: -angle, else: angle
    end
  end

  defp compute_prismatic_distance(_orig_joint, _orig_child, new_joint, new_child, axis) do
    axis = axis || {0.0, 0.0, 1.0}
    axis_tensor = Nx.tensor([elem(axis, 0), elem(axis, 1), elem(axis, 2)], type: :f64)

    displacement = Nx.subtract(new_child, new_joint)
    Nx.dot(displacement, axis_tensor) |> Nx.to_number()
  end

  defp project_onto_plane(vector, normal) do
    dot = Nx.dot(vector, normal)
    projection = Nx.multiply(normal, dot)
    Nx.subtract(vector, projection)
  end

  defp cross_product(a, b) do
    a0 = Nx.slice(a, [0], [1]) |> Nx.squeeze()
    a1 = Nx.slice(a, [1], [1]) |> Nx.squeeze()
    a2 = Nx.slice(a, [2], [1]) |> Nx.squeeze()

    b0 = Nx.slice(b, [0], [1]) |> Nx.squeeze()
    b1 = Nx.slice(b, [1], [1]) |> Nx.squeeze()
    b2 = Nx.slice(b, [2], [1]) |> Nx.squeeze()

    Nx.stack([
      Nx.subtract(Nx.multiply(a1, b2), Nx.multiply(a2, b1)),
      Nx.subtract(Nx.multiply(a2, b0), Nx.multiply(a0, b2)),
      Nx.subtract(Nx.multiply(a0, b1), Nx.multiply(a1, b0))
    ])
  end

  defp get_point(points, index) do
    Nx.slice(points, [index, 0], [1, 3]) |> Nx.squeeze(axes: [0])
  end

  defp clamp_to_limits(value, nil), do: value
  defp clamp_to_limits(value, %{lower: nil, upper: nil}), do: value

  defp clamp_to_limits(value, %{lower: lower, upper: upper}) do
    value
    |> maybe_clamp_lower(lower)
    |> maybe_clamp_upper(upper)
  end

  defp maybe_clamp_lower(value, nil), do: value
  defp maybe_clamp_lower(value, lower), do: max(value, lower)

  defp maybe_clamp_upper(value, nil), do: value
  defp maybe_clamp_upper(value, upper), do: min(value, upper)
end
