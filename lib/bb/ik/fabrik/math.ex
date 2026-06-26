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

    {enforce_ori, target_ori_tensor} =
      case target_orientation do
        nil -> {0.0, Quaternion.identity_tensor()}
        %Quaternion{} = q -> {1.0, Quaternion.tensor(q)}
      end

    positions = frames.positions
    orientations = frames.orientations

    total_length = Nx.sum(lengths) |> Nx.to_number()
    dist_to_target = distance(get_point(positions, 0), target_position) |> Nx.to_number()

    if dist_to_target > total_length do
      {stretched_positions, stretched_orientations, ori_residual} =
        stretch_with_orientation(
          positions,
          orientations,
          lengths,
          target_position,
          target_ori_tensor
        )

      {:error, :unreachable,
       %{
         frames: %{positions: stretched_positions, orientations: stretched_orientations},
         iterations: 0,
         residual: dist_to_target - total_length,
         orientation_residual: nilify_orientation(ori_residual, enforce_ori)
       }}
    else
      {solved_positions, solved_orientations, iterations, residual, ori_residual} =
        solve_with_orientation(
          positions,
          orientations,
          lengths,
          target_position,
          target_ori_tensor,
          enforce_ori,
          max_iterations,
          tolerance,
          orientation_tolerance
        )

      meta = %{
        frames: %{positions: solved_positions, orientations: solved_orientations},
        iterations: Nx.to_number(iterations),
        residual: Nx.to_number(residual),
        orientation_residual: nilify_orientation(Nx.to_number(ori_residual), enforce_ori)
      }

      orientation_converged =
        enforce_ori == 0.0 or meta.orientation_residual <= orientation_tolerance

      if meta.residual <= tolerance and orientation_converged do
        {:ok, meta.frames, meta}
      else
        {:error, :max_iterations, meta}
      end
    end
  end

  defp nilify_orientation(_residual, enforce) when enforce == 0.0, do: nil
  defp nilify_orientation(residual, _enforce), do: residual

  @doc """
  Run the orientation-aware FABRIK loop until convergence or `max_iterations`.

  Composable `defn` companion to `solve/5` that also tracks a `{n, 4}` quaternion
  per joint. `enforce_ori` (`1.0`/`0.0`) selects whether the end-effector
  orientation is pinned to `target_orientation` and whether orientation
  convergence is required. Returns
  `{positions, orientations, iterations, residual, orientation_residual}` as
  tensors; `fabrik_with_orientation/7` wraps this with reachability and `{:ok,
  ...}` classification.
  """
  @spec solve_with_orientation(
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          number(),
          pos_integer(),
          float(),
          float()
        ) :: {Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t()}
  defn solve_with_orientation(
         positions,
         orientations,
         lengths,
         target_position,
         target_orientation,
         enforce_ori,
         max_iterations,
         tolerance,
         orientation_tolerance
       ) do
    root_position = positions[0]
    root_orientation = orientations[0]

    {pos, ori, _l, _tp, _to, _rp, _ro, _ef, _max, _tol, _otol, iterations, residual, ori_residual} =
      while {pos = positions, ori = orientations, lens = lengths, tp = target_position,
             to = target_orientation, rp = root_position, ro = root_orientation, ef = enforce_ori,
             max = max_iterations, tol = tolerance, otol = orientation_tolerance, i = 0,
             residual = Nx.as_type(tolerance, :f64) + 1.0,
             ori_residual = Nx.as_type(orientation_tolerance, :f64) + 1.0},
            i < max and (residual > tol or (ef > 0.5 and ori_residual > otol)) do
        {pos, ori} = backward_pass_with_orientation(pos, ori, lens, tp, to, ef)
        {pos, ori} = forward_pass_with_orientation(pos, ori, lens, rp, ro)
        last = Nx.axis_size(pos, 0) - 1

        {pos, ori, lens, tp, to, rp, ro, ef, max, tol, otol, i + 1, distance(pos[last], tp),
         angular_distance(ori[last], to)}
      end

    {pos, ori, iterations, residual, ori_residual}
  end

  @doc """
  Orientation-aware FABRIK backward reaching pass.

  Pins the end-effector to `target_position` (and, when `enforce_ori > 0.5`, to
  `target_orientation`), then walks toward the root placing each joint and
  aligning its frame's Z-axis with the segment direction. `positions` is
  `{n, 3}`, `orientations` `{n, 4}` (WXYZ), `lengths` `{n - 1}`. Returns
  `{positions, orientations}`.
  """
  @spec backward_pass_with_orientation(
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          number()
        ) :: {Nx.Tensor.t(), Nx.Tensor.t()}
  defn backward_pass_with_orientation(
         positions,
         orientations,
         lengths,
         target_position,
         target_orientation,
         enforce_ori
       ) do
    n = Nx.axis_size(positions, 0)
    positions = Nx.put_slice(positions, [n - 1, 0], Nx.new_axis(target_position, 0))

    end_orientation = Nx.select(enforce_ori > 0.5, target_orientation, orientations[n - 1])
    orientations = Nx.put_slice(orientations, [n - 1, 0], Nx.new_axis(end_orientation, 0))

    {pos, ori, _lengths, _i} =
      while {pos = positions, ori = orientations, lens = lengths, i = n - 2}, i >= 0 do
        p_next = pos[i + 1]
        new_point = move_point_toward(pos[i], p_next, lens[i])
        pos = Nx.put_slice(pos, [i, 0], Nx.new_axis(new_point, 0))

        direction = compute_direction(new_point, p_next)
        new_ori = orientation_from_direction(direction, ori[i + 1])
        ori = Nx.put_slice(ori, [i, 0], Nx.new_axis(new_ori, 0))

        {pos, ori, lens, i - 1}
      end

    {pos, ori}
  end

  @doc """
  Orientation-aware FABRIK forward reaching pass.

  Pins the root to `root_position`/`root_orientation`, then walks toward the
  end-effector placing each joint and propagating orientation from its parent
  along the segment direction. `positions` is `{n, 3}`, `orientations` `{n, 4}`,
  `lengths` `{n - 1}`. Returns `{positions, orientations}`.
  """
  @spec forward_pass_with_orientation(
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t()
        ) :: {Nx.Tensor.t(), Nx.Tensor.t()}
  defn forward_pass_with_orientation(
         positions,
         orientations,
         lengths,
         root_position,
         root_orientation
       ) do
    n = Nx.axis_size(positions, 0)
    positions = Nx.put_slice(positions, [0, 0], Nx.new_axis(root_position, 0))
    orientations = Nx.put_slice(orientations, [0, 0], Nx.new_axis(root_orientation, 0))

    {pos, ori, _lengths, _i} =
      while {pos = positions, ori = orientations, lens = lengths, i = 0}, i < n - 1 do
        p_curr = pos[i]
        new_point = move_point_toward(pos[i + 1], p_curr, lens[i])
        pos = Nx.put_slice(pos, [i + 1, 0], Nx.new_axis(new_point, 0))

        direction = compute_direction(p_curr, new_point)
        new_ori = propagate_orientation(ori[i], direction)
        ori = Nx.put_slice(ori, [i + 1, 0], Nx.new_axis(new_ori, 0))

        {pos, ori, lens, i + 1}
      end

    {pos, ori}
  end

  @doc """
  Stretch the chain straight toward an unreachable target.

  Lays joints along the root→target direction at their segment lengths,
  propagating orientation along that direction. Returns
  `{positions, orientations, orientation_residual}` (the residual is the angular
  distance of the end-effector orientation from `target_orientation`).
  """
  @spec stretch_with_orientation(
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t(),
          Nx.Tensor.t()
        ) :: {Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t()}
  defn stretch_with_orientation(
         positions,
         orientations,
         lengths,
         target_position,
         target_orientation
       ) do
    n = Nx.axis_size(positions, 0)
    root = positions[0]
    root_orientation = orientations[0]
    unit_dir = (target_position - root) |> safe_normalise()

    {pos, ori, _lengths, _unit_dir, _i} =
      while {pos = positions, ori = orientations, lens = lengths, dir = unit_dir, i = 0},
            i < n - 1 do
        new_point = pos[i] + dir * lens[i]
        pos = Nx.put_slice(pos, [i + 1, 0], Nx.new_axis(new_point, 0))

        new_ori = propagate_orientation(ori[i], dir)
        ori = Nx.put_slice(ori, [i + 1, 0], Nx.new_axis(new_ori, 0))

        {pos, ori, lens, dir, i + 1}
      end

    ori = Nx.put_slice(ori, [0, 0], Nx.new_axis(root_orientation, 0))

    {pos, ori, angular_distance(ori[n - 1], target_orientation)}
  end

  defnp safe_normalise(vector) do
    norm = Nx.LinAlg.norm(vector)
    vector / Nx.select(norm < 1.0e-10, 1.0, norm)
  end

  # Angular distance between two unit quaternions: 2·acos(|⟨q1, q2⟩|), matching
  # BB.Math.Quaternion.angular_distance/2.
  defnp angular_distance(q1, q2) do
    dot = Nx.abs(Nx.dot(q1, q2))
    2.0 * Nx.acos(Nx.clip(dot, 0.0, 1.0))
  end

  defnp compute_direction(from, to) do
    direction = Nx.subtract(to, from)
    norm = Nx.LinAlg.norm(direction)

    Nx.select(
      Nx.less(norm, 1.0e-10),
      Nx.tensor([0.0, 0.0, 1.0], type: :f64),
      Nx.divide(direction, norm)
    )
  end

  defnp orientation_from_direction(direction, child_orientation) do
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

  defnp propagate_orientation(parent_orientation, direction) do
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

  defnp quaternion_from_two_vectors(from, to) do
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

  defnp cross_product(a, b) do
    Nx.stack([
      Nx.subtract(Nx.multiply(a[1], b[2]), Nx.multiply(a[2], b[1])),
      Nx.subtract(Nx.multiply(a[2], b[0]), Nx.multiply(a[0], b[2])),
      Nx.subtract(Nx.multiply(a[0], b[1]), Nx.multiply(a[1], b[0]))
    ])
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

  defn distance(p1, p2) do
    Nx.LinAlg.norm(p1 - p2)
  end
end
