# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.Math do
  @moduledoc """
  Pure Nx implementation of FABRIK, constrained to a robot's real joint axes.

  `solve_constrained/3` is the entry point. `backward_pass/3` is the classic
  FABRIK reach it builds on, kept separate because it is the one half of the
  algorithm that needs no knowledge of how the joints are allowed to move.
  """

  import Nx.Defn

  alias BB.Robot.Kinematics

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

  defnp safe_normalise(vector) do
    norm = Nx.LinAlg.norm(vector)

    Nx.select(
      Nx.less(norm, 1.0e-10),
      Nx.tensor([0.0, 0.0, 1.0], type: :f64),
      Nx.divide(vector, norm)
    )
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

  defn distance(p1, p2) do
    Nx.LinAlg.norm(p1 - p2)
  end

  # ===========================================================================
  # Constrained FABRIK
  # ===========================================================================

  @doc """
  Run FABRIK against a chain whose joints each turn about one fixed axis.

  Classic FABRIK moves points freely, as though every joint were a ball joint. A
  robot's are not, so the point configuration it converges on generally has no
  counterpart in any pose the robot can hold, and reading joint values back out
  of it is a fit that starts centimetres wrong.

  This keeps the backward pass — the reach that pins the end effector to the
  target and distributes the correction back along the chain — but treats its
  answer as a set of *desired directions* rather than positions. The forward pass
  then walks base to tip choosing, for each joint, the rotation about its real
  world axis that carries its links closest to those directions, clamped to its
  limits, and regenerates the positions beyond it by forward kinematics. Every
  pose considered is therefore one the robot can hold, and the joint values are
  the solver's output rather than something recovered afterwards.

  ## Arguments

  - `chain` - the description from `BB.IK.FABRIK.Chain.kinematics/1`
  - `target` - `%{position: {3}, rotation: {3, 3}, enforce: scalar}`, where
    `enforce` above `0.5` asks for the orientation as well as the position
  - `opts` - `%{max_iterations:, tolerance:, orientation_tolerance:, lever:}`

  All geometry is in the chain root's frame, `target` included. Returns
  `{positions, iterations, residual, orientation_residual}` as tensors, and
  vectorises over a leading batch axis so a fleet of identical chains — the legs
  of a gait, say — solves in one call with a lane per chain.
  """
  @spec solve_constrained(map(), map(), map()) ::
          {Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t(), Nx.Tensor.t()}
  defn solve_constrained(chain, target, opts) do
    # `chain.positions` seeds the loop and is not read again — `q` is the live
    # configuration from here on.
    {solved, _chain, _target, _opts, iterations, residual, orientation_residual} =
      while {q = chain.positions, c = chain, t = target, o = opts, i = 0,
             residual = tip_distance(chain.positions, chain, target),
             orientation_residual = tip_angle(chain.positions, chain, target)},
            i < o.max_iterations and
              (residual > o.tolerance or
                 (t.enforce > 0.5 and orientation_residual > o.orientation_tolerance)) do
        swept = sweep(q, c, t, o)
        {swept, c, t, o, i + 1, tip_distance(swept, c, t), tip_angle(swept, c, t)}
      end

    {solved, iterations, residual, orientation_residual}
  end

  defnp tip_transform(positions, chain) do
    Nx.take(chain_transforms(positions, chain), chain.target_row, axis: 0)
  end

  defnp tip_distance(positions, chain, target) do
    distance(tip_transform(positions, chain)[[0..2, 3]], target.position)
  end

  defnp tip_angle(positions, chain, target) do
    frame_angle(tip_transform(positions, chain)[[0..2, 0..2]], target.rotation)
  end

  # The angle of the rotation taking one frame onto the other, read off the trace
  # of their relative rotation. Avoids needing a quaternion inside `defn`.
  defnp frame_angle(rotation, target_rotation) do
    relative = Nx.dot(Nx.transpose(rotation), target_rotation)
    cosine = (relative[0][0] + relative[1][1] + relative[2][2] - 1.0) / 2.0

    Nx.acos(Nx.clip(cosine, -1.0, 1.0))
  end

  defnp chain_transforms(positions, chain) do
    Kinematics.Defn.link_transforms(
      positions,
      chain.origin_rpy,
      chain.origin_xyz,
      chain.axes,
      chain.is_revolute,
      chain.is_prismatic,
      chain.stored,
      chain.deltas,
      chain.parent_idx
    )
  end

  # One backward reach to say where the chain should go, then one constrained
  # forward sweep to go as far there as the joints allow.
  defnp sweep(positions, chain, target, opts) do
    transforms = chain_transforms(positions, chain)
    tip = Nx.take(transforms, chain.target_row, axis: 0)
    points = chain_points(transforms, chain)

    # The frame points hang off where the end effector *is*, not where it is
    # wanted, so the only thing they ask for is the turn — the reach to the
    # target stays the real points' job and the two do not fight over it.
    desired =
      Nx.concatenate([
        backward_pass(points, segment_lengths(points), target.position),
        frame_points(tip[[0..2, 3]], target.rotation, opts.lever)
      ])

    share = orientation_share(tip, target, opts)

    {swept, _chain, _desired, _share, _opts, _j} =
      while {q = positions, c = chain, d = desired, w = share, o = opts, j = 0},
            j < Nx.axis_size(chain.joint_rows, 0) do
        {fit_joint(q, c, d, w, o, j), c, d, w, o, j + 1}
      end

    swept
  end

  # How much of this sweep belongs to orientation, as the share of the total
  # overshoot that orientation accounts for once each error is measured against
  # its own tolerance. A caller who passes a slack `:orientation_tolerance` is
  # saying they do not much care, and this lets position have the sweep.
  defnp orientation_share(tip, target, opts) do
    position = distance(tip[[0..2, 3]], target.position) / opts.tolerance
    orientation = frame_angle(tip[[0..2, 0..2]], target.rotation) / opts.orientation_tolerance

    target.enforce * orientation / Nx.max(orientation + position, 1.0e-12)
  end

  defnp chain_points(transforms, chain) do
    Nx.take(transforms, chain.point_rows, axis: 0)[[.., 0..2, 3]]
  end

  # Three points rigidly attached to a frame, one along each of its axes. Putting
  # them where they are wanted is the same as pointing the frame where it is
  # wanted, which lets one position fit serve orientation too. The lever sets how
  # loudly orientation argues against position.
  defnp frame_points(position, rotation, lever) do
    position + lever * Nx.transpose(rotation)
  end

  defnp segment_lengths(points) do
    n = Nx.axis_size(points, 0)
    from = Nx.slice_along_axis(points, 0, n - 1, axis: 0)
    to = Nx.slice_along_axis(points, 1, n - 1, axis: 0)

    Nx.sum((to - from) ** 2, axes: [1]) |> Nx.sqrt()
  end

  # Choose this joint's value against the links it can actually move, measured
  # under the values already chosen upstream. Recomputing the transforms is what
  # keeps the choice relative to the joint's parent rather than to the world: a
  # link's world displacement is the accumulated work of every joint above it.
  defnp fit_joint(positions, chain, desired, share, opts, index) do
    row = Nx.take(chain.joint_rows, index)
    transforms = chain_transforms(positions, chain)
    child = Nx.take(transforms, row, axis: 0)
    tip = Nx.take(transforms, chain.target_row, axis: 0)

    pivot = child[[0..2, 3]]
    axis = safe_normalise(Nx.dot(child[[0..2, 0..2]], Nx.take(chain.axes, row, axis: 0)))

    current =
      Nx.concatenate([
        chain_points(transforms, chain),
        frame_points(tip[[0..2, 3]], tip[[0..2, 0..2]], opts.lever)
      ])

    # A joint turns every link beyond it. The frame points ride on the target
    # link itself, so the last joint moves them even though it moves no point
    # past its own.
    downstream =
      Nx.concatenate([
        chain.point_rows > row,
        Nx.broadcast(chain.target_row >= row, {3})
      ])

    offsets = current - pivot
    wanted = desired - pivot

    # A link lying on the joint's axis is carried around it without moving, so it
    # cannot say what the angle should be and the others have to answer.
    radial = offsets - outer(Nx.dot(offsets, axis), axis)
    lever = Nx.sqrt(Nx.sum(radial ** 2, axes: [1]))
    turnable = downstream and lever > 1.0e-6

    prismatic? = Nx.take(chain.is_prismatic, row) > 0.5
    usable = Nx.select(prismatic?, downstream, turnable)

    weight =
      Nx.as_type(usable, {:f, 64}) *
        Nx.concatenate([
          Nx.broadcast(1.0 - share, {Nx.axis_size(chain.point_rows, 0)}),
          Nx.broadcast(share, {3})
        ])

    delta =
      Nx.select(
        prismatic?,
        slide_distance(current, desired, axis, weight),
        turn_angle(radial, wanted, axis, weight)
      )

    delta = Nx.select(Nx.any(usable), delta, 0.0)

    value =
      Nx.clip(
        Nx.take(positions, row) + delta,
        Nx.take(chain.limits_lower, index),
        Nx.take(chain.limits_upper, index)
      )

    Nx.put_slice(positions, [row], Nx.reshape(value, {1}))
  end

  # Rotating by θ about the axis sends a perpendicular offset u to
  # u·cos θ + (axis × u)·sin θ, so the θ carrying every movable link closest to
  # where it is wanted maximises Σ (R u)·v — a stationary point at atan2 of the
  # two accumulated terms. No search, no Jacobian.
  #
  # Every such link votes, weighted by its lever arm, because a single one cannot
  # be trusted: the nearest carries the joint's own contribution but reads a
  # backward pass position that may encode a bend this joint cannot make, while
  # the end effector reads the true target but lets a joint claim work belonging
  # to its children.
  defnp turn_angle(radial, wanted, axis, weight) do
    perpendicular = wanted - outer(Nx.dot(wanted, axis), axis)

    Nx.atan2(
      Nx.sum(weight * Nx.sum(cross_rows(axis, radial) * perpendicular, axes: [1])),
      Nx.sum(weight * Nx.sum(radial * perpendicular, axes: [1]))
    )
  end

  # A prismatic joint carries every link beyond it equally, so the distance to
  # slide is the mean of what those links each ask for along the axis.
  defnp slide_distance(current, desired, axis, weight) do
    asked = Nx.sum((desired - current) * axis, axes: [1])

    Nx.sum(weight * asked) / Nx.max(Nx.sum(weight), 1.0)
  end

  defnp(outer(scalars, vector), do: Nx.new_axis(scalars, 1) * vector)

  defnp cross_rows(axis, rows) do
    Nx.stack(
      [
        axis[1] * rows[[.., 2]] - axis[2] * rows[[.., 1]],
        axis[2] * rows[[.., 0]] - axis[0] * rows[[.., 2]],
        axis[0] * rows[[.., 1]] - axis[1] * rows[[.., 0]]
      ],
      axis: 1
    )
  end
end
