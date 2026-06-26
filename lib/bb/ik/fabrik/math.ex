# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.Math do
  @moduledoc """
  Pure Nx implementation of the FABRIK algorithm.

  Provides both position-only solving (`fabrik/5`) and full frame-based
  solving with orientation (`fabrik_with_orientation/7`).
  """

  import Nx.Defn

  alias BB.Math.Quaternion

  @doc """
  Run the FABRIK algorithm on a chain of points.

  ## Parameters

  - `points` - Nx tensor of shape `{n+1, 3}` representing joint/link positions
  - `lengths` - Nx tensor of shape `{n}` representing segment lengths
  - `target` - Nx tensor of shape `{3}` representing target position
  - `max_iterations` - Maximum number of iterations
  - `tolerance` - Convergence tolerance (distance to target)

  ## Returns

  - `{:ok, new_points, meta}` - Converged successfully
  - `{:error, :unreachable, meta}` - Target is beyond reach
  - `{:error, :max_iterations, meta}` - Did not converge within max iterations

  In all cases, `meta` contains:
  - `:points` - Final point positions
  - `:iterations` - Number of iterations performed
  - `:residual` - Final distance to target
  """
  @spec fabrik(
          points :: Nx.Tensor.t(),
          lengths :: Nx.Tensor.t(),
          target :: Nx.Tensor.t(),
          max_iterations :: pos_integer(),
          tolerance :: float()
        ) ::
          {:ok, Nx.Tensor.t(), map()}
          | {:error, :unreachable | :max_iterations, map()}
  def fabrik(points, lengths, target, max_iterations, tolerance) do
    root = get_point(points, 0)
    total_length = Nx.sum(lengths) |> Nx.to_number()
    dist_to_target = distance(root, target) |> Nx.to_number()

    if dist_to_target > total_length do
      {:error, :unreachable,
       %{
         points: stretch_toward_target(points, lengths, target),
         iterations: 0,
         residual: dist_to_target - total_length
       }}
    else
      {solved_points, iterations, residual} =
        solve(points, lengths, target, max_iterations, tolerance)

      meta = %{
        points: solved_points,
        iterations: Nx.to_number(iterations),
        residual: Nx.to_number(residual)
      }

      if meta.residual <= tolerance do
        {:ok, solved_points, meta}
      else
        {:error, :max_iterations, meta}
      end
    end
  end

  @doc """
  Run the FABRIK iteration loop until convergence or `max_iterations`.

  Composable `defn` entry point. Returns `{points, iterations, residual}` as
  tensors — `fabrik/5` wraps this with the reachability check and `{:ok, ...}`
  classification. Each iteration runs `backward_pass/3` then `forward_pass/3`.
  Built from `defn` so it can be composed into larger numerical pipelines and
  vectorised over a leading batch axis (e.g. solving many legs of a gait at
  once).
  """
  @spec solve(Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t(), pos_integer(), float()) ::
          {Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t()}
  defn solve(points, lengths, target, max_iterations, tolerance) do
    root = points[0]

    {solved, _lengths, _target, _root, _max, _tol, iterations, residual} =
      while {pts = points, lens = lengths, tgt = target, rt = root, max = max_iterations,
             tol = tolerance, i = 0, residual = Nx.as_type(tolerance, :f64) + 1.0},
            i < max and residual > tol do
        pts = backward_pass(pts, lens, tgt)
        pts = forward_pass(pts, lens, rt)
        tip = pts[Nx.axis_size(pts, 0) - 1]
        {pts, lens, tgt, rt, max, tol, i + 1, distance(tip, tgt)}
      end

    {solved, iterations, residual}
  end

  @doc """
  FABRIK backward reaching pass.

  Pins the end effector to `target`, then walks toward the root placing each
  joint at its segment length from the next. `points` is `{n, 3}`, `lengths`
  `{n - 1}`. Returns the updated `{n, 3}` points.
  """
  @spec backward_pass(Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t()) :: Nx.Tensor.t()
  defn backward_pass(points, lengths, target) do
    n = Nx.axis_size(points, 0)
    points = Nx.put_slice(points, [n - 1, 0], Nx.new_axis(target, 0))

    {result, _lengths, _i} =
      while {pts = points, lens = lengths, i = n - 2}, i >= 0 do
        new_point = move_point_toward(pts[i], pts[i + 1], lens[i])
        {Nx.put_slice(pts, [i, 0], Nx.new_axis(new_point, 0)), lens, i - 1}
      end

    result
  end

  @doc """
  FABRIK forward reaching pass.

  Pins the root to `root`, then walks toward the end effector placing each joint
  at its segment length from the previous. `points` is `{n, 3}`, `lengths`
  `{n - 1}`. Returns the updated `{n, 3}` points.
  """
  @spec forward_pass(Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t()) :: Nx.Tensor.t()
  defn forward_pass(points, lengths, root) do
    n = Nx.axis_size(points, 0)
    points = Nx.put_slice(points, [0, 0], Nx.new_axis(root, 0))

    {result, _lengths, _i} =
      while {pts = points, lens = lengths, i = 0}, i < n - 1 do
        new_point = move_point_toward(pts[i + 1], pts[i], lens[i])
        {Nx.put_slice(pts, [i + 1, 0], Nx.new_axis(new_point, 0)), lens, i + 1}
      end

    result
  end

  # ===========================================================================
  # Frame-based FABRIK with orientation support
  # ===========================================================================

  @doc """
  Convert points tensor to frames with identity orientations.

  Useful for using the orientation-aware algorithm with position-only data.
  """
  @spec points_to_frames(Nx.Tensor.t()) :: frames()
  def points_to_frames(points) do
    n = Nx.axis_size(points, 0)
    identity = Quaternion.identity_tensor()
    orientations = Nx.broadcast(identity, {n, 4})
    %{positions: points, orientations: orientations}
  end

  @doc """
  Extract positions tensor from frames.
  """
  @spec frames_to_points(frames()) :: Nx.Tensor.t()
  def frames_to_points(frames) do
    frames.positions
  end

  @typedoc """
  Frame representation for orientation-aware FABRIK.

  - `:positions` - Joint positions as `{n+1, 3}` tensor
  - `:orientations` - Joint orientations as `{n+1, 4}` tensor (WXYZ quaternions)
  """
  @type frames :: %{positions: Nx.Tensor.t(), orientations: Nx.Tensor.t()}

  @doc """
  Run FABRIK with full orientation support.

  ## Parameters

  - `frames` - Map with `:positions` `{n+1, 3}` and `:orientations` `{n+1, 4}` tensors
  - `lengths` - Segment lengths as `{n}` tensor
  - `target_position` - Target position as `{3}` tensor
  - `target_orientation` - Target quaternion as `Quaternion.t()` or `nil` for position-only
  - `max_iterations` - Maximum solver iterations
  - `tolerance` - Position convergence tolerance (metres)
  - `opts` - Options including `:orientation_tolerance` (radians, default 0.01)

  ## Returns

  - `{:ok, frames, meta}` - Converged successfully
  - `{:error, :unreachable, meta}` - Target beyond reach
  - `{:error, :max_iterations, meta}` - Did not converge

  Meta includes:
  - `:frames` - Final frame state
  - `:iterations` - Iterations performed
  - `:residual` - Position residual (metres)
  - `:orientation_residual` - Orientation residual (radians) or `nil`
  """
  @spec fabrik_with_orientation(
          frames :: frames(),
          lengths :: Nx.Tensor.t(),
          target_position :: Nx.Tensor.t(),
          target_orientation :: Quaternion.t() | nil,
          max_iterations :: pos_integer(),
          tolerance :: float(),
          opts :: keyword()
        ) ::
          {:ok, frames(), map()}
          | {:error, :unreachable | :max_iterations, map()}
  def fabrik_with_orientation(
        frames,
        lengths,
        target_position,
        target_orientation,
        max_iterations,
        tolerance,
        opts \\ []
      ) do
    orientation_tolerance = Keyword.get(opts, :orientation_tolerance, 0.01)

    root_position = get_point(frames.positions, 0)
    root_orientation = get_orientation(frames.orientations, 0)

    total_length = Nx.sum(lengths) |> Nx.to_number()
    dist_to_target = distance(root_position, target_position) |> Nx.to_number()

    if dist_to_target > total_length do
      stretched = stretch_frames_toward_target(frames, lengths, target_position)
      residual = dist_to_target - total_length
      orientation_residual = compute_orientation_residual(stretched, target_orientation)

      {:error, :unreachable,
       %{
         frames: stretched,
         iterations: 0,
         residual: residual,
         orientation_residual: orientation_residual
       }}
    else
      config = %{
        lengths: lengths,
        root_position: root_position,
        root_orientation: root_orientation,
        target_position: target_position,
        target_orientation: target_orientation,
        max_iterations: max_iterations,
        tolerance: tolerance,
        orientation_tolerance: orientation_tolerance
      }

      iterate_with_orientation(frames, config, 0)
    end
  end

  defp iterate_with_orientation(frames, config, iteration)
       when iteration >= config.max_iterations do
    end_effector = get_end_effector(frames.positions)
    residual = distance(end_effector, config.target_position) |> Nx.to_number()
    orientation_residual = compute_orientation_residual(frames, config.target_orientation)

    {:error, :max_iterations,
     %{
       frames: frames,
       iterations: iteration,
       residual: residual,
       orientation_residual: orientation_residual
     }}
  end

  defp iterate_with_orientation(frames, config, iteration) do
    frames =
      backward_pass_with_orientation(
        frames,
        config.lengths,
        config.target_position,
        config.target_orientation
      )

    frames =
      forward_pass_with_orientation(
        frames,
        config.lengths,
        config.root_position,
        config.root_orientation
      )

    end_effector = get_end_effector(frames.positions)
    residual = distance(end_effector, config.target_position) |> Nx.to_number()
    orientation_residual = compute_orientation_residual(frames, config.target_orientation)

    position_converged = residual <= config.tolerance

    orientation_converged =
      is_nil(config.target_orientation) or
        (orientation_residual != nil and orientation_residual <= config.orientation_tolerance)

    if position_converged and orientation_converged do
      {:ok, frames,
       %{
         frames: frames,
         iterations: iteration + 1,
         residual: residual,
         orientation_residual: orientation_residual
       }}
    else
      iterate_with_orientation(frames, config, iteration + 1)
    end
  end

  defp backward_pass_with_orientation(frames, lengths, target_position, target_orientation) do
    n = Nx.axis_size(frames.positions, 0)
    num_segments = n - 1

    # Set end-effector to target
    positions = put_point(frames.positions, n - 1, target_position)

    orientations =
      if target_orientation do
        put_orientation(frames.orientations, n - 1, Quaternion.tensor(target_orientation))
      else
        frames.orientations
      end

    # Work backward from end-effector to root
    {positions, orientations} =
      Enum.reduce((num_segments - 1)..0//-1, {positions, orientations}, fn i, {pos, ori} ->
        p_next = get_point(pos, i + 1)
        p_curr = get_point(pos, i)
        len = Nx.slice(lengths, [i], [1]) |> Nx.squeeze()

        # Move position toward next joint at correct distance
        new_point = move_point_toward(p_curr, p_next, len)
        pos = put_point(pos, i, new_point)

        # Compute orientation: align local Z-axis with segment direction
        direction = compute_direction(new_point, p_next)
        new_ori = orientation_from_direction(direction, get_orientation(ori, i + 1))
        ori = put_orientation(ori, i, new_ori)

        {pos, ori}
      end)

    %{positions: positions, orientations: orientations}
  end

  defp forward_pass_with_orientation(frames, lengths, root_position, root_orientation) do
    n = Nx.axis_size(frames.positions, 0)
    num_segments = n - 1

    # Pin root
    positions = put_point(frames.positions, 0, root_position)
    orientations = put_orientation(frames.orientations, 0, root_orientation)

    # Work forward from root to end-effector
    {positions, orientations} =
      Enum.reduce(0..(num_segments - 1)//1, {positions, orientations}, fn i, {pos, ori} ->
        p_curr = get_point(pos, i)
        p_next = get_point(pos, i + 1)
        len = Nx.slice(lengths, [i], [1]) |> Nx.squeeze()

        # Move next position to correct distance from current
        new_point = move_point_toward(p_next, p_curr, len)
        pos = put_point(pos, i + 1, new_point)

        # Propagate orientation: accumulate from parent
        direction = compute_direction(p_curr, new_point)
        parent_ori = get_orientation(ori, i)
        new_ori = propagate_orientation(parent_ori, direction)
        ori = put_orientation(ori, i + 1, new_ori)

        {pos, ori}
      end)

    %{positions: positions, orientations: orientations}
  end

  defp stretch_frames_toward_target(frames, lengths, target) do
    n = Nx.axis_size(frames.positions, 0)
    num_segments = n - 1
    root = get_point(frames.positions, 0)
    root_ori = get_orientation(frames.orientations, 0)

    direction = Nx.subtract(target, root)
    dir_norm = Nx.LinAlg.norm(direction)
    unit_dir = Nx.divide(direction, dir_norm)

    {positions, orientations} =
      Enum.reduce(
        0..(num_segments - 1)//1,
        {put_point(frames.positions, 0, root), frames.orientations},
        fn i, {pos, ori} ->
          p_curr = get_point(pos, i)
          len = Nx.slice(lengths, [i], [1]) |> Nx.squeeze()

          new_point = Nx.add(p_curr, Nx.multiply(unit_dir, len))
          pos = put_point(pos, i + 1, new_point)

          # Orientation follows the stretch direction
          new_ori = propagate_orientation(get_orientation(ori, i), unit_dir)
          ori = put_orientation(ori, i + 1, new_ori)

          {pos, ori}
        end
      )

    # Pin root orientation
    orientations = put_orientation(orientations, 0, root_ori)

    %{positions: positions, orientations: orientations}
  end

  defp compute_orientation_residual(_frames, nil), do: nil

  defp compute_orientation_residual(frames, target_orientation) do
    n = Nx.axis_size(frames.positions, 0)
    end_ori_tensor = get_orientation(frames.orientations, n - 1)
    end_ori = Quaternion.from_tensor(end_ori_tensor)
    Quaternion.angular_distance(end_ori, target_orientation)
  end

  defp compute_direction(from, to) do
    direction = Nx.subtract(to, from)
    norm = Nx.LinAlg.norm(direction)

    Nx.select(
      Nx.less(norm, 1.0e-10),
      Nx.tensor([0.0, 0.0, 1.0], type: :f64),
      Nx.divide(direction, norm)
    )
  end

  defp orientation_from_direction(direction, child_orientation) do
    # Create rotation that aligns Z-axis with direction
    # Start from child orientation and adjust
    z_axis = Nx.tensor([0.0, 0.0, 1.0], type: :f64)

    # If direction is close to Z-axis, use child orientation
    dot = Nx.dot(direction, z_axis)

    Nx.select(
      Nx.greater(Nx.abs(dot), 0.9999),
      child_orientation,
      quaternion_from_two_vectors(z_axis, direction)
    )
  end

  defp propagate_orientation(parent_orientation, direction) do
    # Compute orientation that points in the given direction
    # relative to the parent's frame
    z_axis = Nx.tensor([0.0, 0.0, 1.0], type: :f64)
    dot = Nx.dot(direction, z_axis)

    Nx.select(
      Nx.greater(Nx.abs(dot), 0.9999),
      parent_orientation,
      quaternion_from_two_vectors(z_axis, direction)
    )
  end

  defp quaternion_from_two_vectors(from, to) do
    # Compute quaternion that rotates 'from' to 'to'
    # Both vectors should be normalised
    cross = cross_product(from, to)
    dot = Nx.dot(from, to)

    # w = 1 + dot, xyz = cross
    w = Nx.add(1.0, dot)

    # Handle anti-parallel case (dot ≈ -1)
    is_antiparallel = Nx.less(w, 1.0e-6)

    # For anti-parallel, rotate 180° around perpendicular axis
    perp =
      Nx.select(
        Nx.greater(Nx.abs(from[0]), 0.9),
        cross_product(from, Nx.tensor([0.0, 1.0, 0.0], type: :f64)),
        cross_product(from, Nx.tensor([1.0, 0.0, 0.0], type: :f64))
      )

    perp_norm = Nx.LinAlg.norm(perp)
    perp_normalised = Nx.divide(perp, perp_norm)

    # Normal case quaternion (unnormalised)
    normal_quat = Nx.concatenate([Nx.reshape(w, {1}), cross])

    # Anti-parallel case quaternion (180° rotation)
    antiparallel_quat = Nx.concatenate([Nx.tensor([0.0], type: :f64), perp_normalised])

    quat = Nx.select(is_antiparallel, antiparallel_quat, normal_quat)

    # Normalise
    norm = Nx.LinAlg.norm(quat)
    Nx.divide(quat, norm)
  end

  defp cross_product(a, b) do
    Nx.stack([
      Nx.subtract(Nx.multiply(a[1], b[2]), Nx.multiply(a[2], b[1])),
      Nx.subtract(Nx.multiply(a[2], b[0]), Nx.multiply(a[0], b[2])),
      Nx.subtract(Nx.multiply(a[0], b[1]), Nx.multiply(a[1], b[0]))
    ])
  end

  defp get_orientation(orientations, index) do
    Nx.slice(orientations, [index, 0], [1, 4]) |> Nx.squeeze(axes: [0])
  end

  defp put_orientation(orientations, index, orientation) do
    orientation_2d = Nx.reshape(orientation, {1, 4})
    indices = Nx.tensor([[index, 0], [index, 1], [index, 2], [index, 3]])
    values = Nx.flatten(orientation_2d)
    Nx.indexed_put(orientations, indices, values)
  end

  # ===========================================================================
  # Position-only helpers (existing)
  # ===========================================================================

  defp stretch_toward_target(points, lengths, target) do
    n = Nx.axis_size(points, 0)
    num_segments = n - 1
    root = Nx.slice(points, [0, 0], [1, 3]) |> Nx.squeeze(axes: [0])

    direction = Nx.subtract(target, root)
    dir_norm = Nx.LinAlg.norm(direction)
    unit_dir = Nx.divide(direction, dir_norm)

    Enum.reduce(0..(num_segments - 1)//1, put_point(points, 0, root), fn i, points ->
      p_curr = get_point(points, i)
      len = Nx.slice(lengths, [i], [1]) |> Nx.squeeze()

      new_point = Nx.add(p_curr, Nx.multiply(unit_dir, len))
      put_point(points, i + 1, new_point)
    end)
  end

  @doc """
  Place a point at `desired_distance` from `anchor`, along the direction from
  `anchor` toward `point_to_move`. The per-joint reaching step shared by both
  passes; a `defn` so it composes into them and is reusable on its own.
  """
  defn move_point_toward(point_to_move, anchor, desired_distance) do
    direction = point_to_move - anchor
    current_distance = Nx.LinAlg.norm(direction)
    safe_distance = Nx.select(current_distance < 1.0e-10, 1.0, current_distance)
    unit_dir = direction / safe_distance

    anchor + unit_dir * desired_distance
  end

  defp get_point(points, index) do
    Nx.slice(points, [index, 0], [1, 3]) |> Nx.squeeze(axes: [0])
  end

  defp put_point(points, index, point) do
    point_2d = Nx.reshape(point, {1, 3})
    indices = Nx.tensor([[index, 0], [index, 1], [index, 2]])
    values = Nx.flatten(point_2d)
    Nx.indexed_put(points, indices, values)
  end

  defp get_end_effector(points) do
    n = Nx.axis_size(points, 0)
    get_point(points, n - 1)
  end

  defn distance(p1, p2) do
    Nx.LinAlg.norm(p1 - p2)
  end
end
