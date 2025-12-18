# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.Math do
  @moduledoc false

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
    root = Nx.slice(points, [0, 0], [1, 3]) |> Nx.squeeze(axes: [0])
    total_length = Nx.sum(lengths) |> Nx.to_number()
    dist_to_target = distance(root, target) |> Nx.to_number()

    if dist_to_target > total_length do
      stretched_points = stretch_toward_target(points, lengths, target)
      residual = dist_to_target - total_length

      {:error, :unreachable,
       %{
         points: stretched_points,
         iterations: 0,
         residual: residual
       }}
    else
      iterate(points, lengths, root, target, max_iterations, tolerance, 0)
    end
  end

  defp iterate(points, _lengths, _root, target, max_iterations, _tolerance, iteration)
       when iteration >= max_iterations do
    end_effector = get_end_effector(points)
    residual = distance(end_effector, target) |> Nx.to_number()

    {:error, :max_iterations,
     %{
       points: points,
       iterations: iteration,
       residual: residual
     }}
  end

  defp iterate(points, lengths, root, target, max_iterations, tolerance, iteration) do
    points = backward_pass(points, lengths, target)
    points = forward_pass(points, lengths, root)

    end_effector = get_end_effector(points)
    residual = distance(end_effector, target) |> Nx.to_number()

    if residual <= tolerance do
      {:ok, points,
       %{
         points: points,
         iterations: iteration + 1,
         residual: residual
       }}
    else
      iterate(points, lengths, root, target, max_iterations, tolerance, iteration + 1)
    end
  end

  defp backward_pass(points, lengths, target) do
    n = Nx.axis_size(points, 0)
    num_segments = n - 1

    points = put_point(points, n - 1, target)

    Enum.reduce((num_segments - 1)..0//-1, points, fn i, points ->
      p_next = get_point(points, i + 1)
      p_curr = get_point(points, i)
      len = Nx.slice(lengths, [i], [1]) |> Nx.squeeze()

      new_point = move_point_toward(p_curr, p_next, len)
      put_point(points, i, new_point)
    end)
  end

  defp forward_pass(points, lengths, root) do
    n = Nx.axis_size(points, 0)
    num_segments = n - 1

    points = put_point(points, 0, root)

    Enum.reduce(0..(num_segments - 1)//1, points, fn i, points ->
      p_curr = get_point(points, i)
      p_next = get_point(points, i + 1)
      len = Nx.slice(lengths, [i], [1]) |> Nx.squeeze()

      new_point = move_point_toward(p_next, p_curr, len)
      put_point(points, i + 1, new_point)
    end)
  end

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

  defp move_point_toward(point_to_move, anchor, desired_distance) do
    direction = Nx.subtract(point_to_move, anchor)
    current_distance = Nx.LinAlg.norm(direction)

    safe_distance =
      Nx.select(
        Nx.less(current_distance, 1.0e-10),
        Nx.tensor(1.0, type: :f64),
        current_distance
      )

    unit_dir = Nx.divide(direction, safe_distance)
    Nx.add(anchor, Nx.multiply(unit_dir, desired_distance))
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

  defp distance(p1, p2) do
    Nx.subtract(p1, p2) |> Nx.LinAlg.norm()
  end
end
