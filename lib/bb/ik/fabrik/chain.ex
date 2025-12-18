# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.Chain do
  @moduledoc false

  alias BB.Robot
  alias BB.Robot.{Joint, Kinematics, Transform}

  defstruct [
    :joints,
    :joint_names,
    :points,
    :lengths,
    :limits,
    :root_transform,
    :original_positions
  ]

  @type t :: %__MODULE__{
          joints: [Joint.t()],
          joint_names: [atom()],
          points: Nx.Tensor.t(),
          lengths: Nx.Tensor.t(),
          limits: [Joint.limits() | nil],
          root_transform: Nx.Tensor.t(),
          original_positions: %{atom() => float()}
        }

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
  - `{:error, :unknown_link}` - Target link not found
  - `{:error, :no_dofs}` - Chain has no movable joints
  """
  @spec build(Robot.t(), %{atom() => float()}, atom()) ::
          {:ok, t()} | {:error, :unknown_link | :no_dofs}
  def build(%Robot{} = robot, positions, target_link) when is_map(positions) do
    case Robot.path_to(robot, target_link) do
      nil ->
        {:error, :unknown_link}

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
          {:error, :no_dofs}
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

  # Threshold for treating computed deltas as zero to avoid numerical drift
  # 1e-8 radians ≈ 5.7e-7 degrees - well below mechanical precision
  @delta_threshold 1.0e-8

  defp compute_and_add_position(
         acc,
         joint_name,
         idx,
         joint,
         chain,
         solved_points,
         respect_limits?
       ) do
    # compute_joint_position returns a DELTA angle/distance from the original configuration
    # We need to add it to the original joint position to get the absolute position
    delta = compute_joint_position(chain, solved_points, idx, joint)

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
    root_point = Transform.get_translation(transforms[first_joint.parent_link])

    # Get positions of each joint's child link
    child_link_points =
      Enum.map(joints, fn joint ->
        Transform.get_translation(transforms[joint.child_link])
      end)

    # The final point is the target link
    end_effector_point = Transform.get_translation(transforms[target_link])

    # Combine all points, but deduplicate consecutive identical points
    all_points = [root_point] ++ child_link_points ++ [end_effector_point]

    # Remove consecutive duplicate points (within tolerance)
    points_list = dedupe_consecutive_points(all_points)

    points =
      points_list
      |> Enum.map(fn {x, y, z} -> [x, y, z] end)
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
      points: points,
      lengths: lengths,
      limits: limits,
      root_transform: Transform.identity(),
      original_positions: positions
    }
  end

  defp dedupe_consecutive_points(points) do
    points
    |> Enum.reduce([], fn point, acc -> maybe_add_point(acc, point) end)
    |> Enum.reverse()
  end

  defp maybe_add_point([], point), do: [point]

  defp maybe_add_point([last | _] = acc, point) do
    if points_equal?(last, point), do: acc, else: [point | acc]
  end

  defp points_equal?({x1, y1, z1}, {x2, y2, z2}) do
    tolerance = 1.0e-6
    abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance and abs(z1 - z2) < tolerance
  end

  defp compute_joint_position(chain, solved_points, idx, joint) do
    # Points are: [root, child_of_joint0, child_of_joint1, ..., end_effector]
    # For joint at index idx:
    # - The joint pivot is at point idx (where we rotate from)
    # - The direction to child is toward point idx + 1
    p_joint = get_point(solved_points, idx)
    p_child = get_point(solved_points, idx + 1)

    orig_p_joint = get_point(chain.points, idx)
    orig_p_child = get_point(chain.points, idx + 1)

    case joint.type do
      type when type in [:revolute, :continuous] ->
        compute_revolute_angle(orig_p_joint, orig_p_child, p_joint, p_child, joint.axis)

      :prismatic ->
        compute_prismatic_distance(orig_p_joint, orig_p_child, p_joint, p_child, joint.axis)

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
