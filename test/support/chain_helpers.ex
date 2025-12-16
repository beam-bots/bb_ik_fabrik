# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.TestHelpers.Chain do
  @moduledoc """
  Test helpers for creating chain point tensors.

  These helpers make it easier to reason about test data by providing
  semantic functions for common chain configurations.
  """

  @doc """
  Convert a list of {x, y, z} tuples to an Nx tensor.

  ## Example

      iex> points([{0, 0, 0}, {1, 0, 0}, {2, 0, 0}])
      #Nx.Tensor<f64[3][3]>
  """
  @spec points([{number(), number(), number()}]) :: Nx.Tensor.t()
  def points(tuples) when is_list(tuples) do
    tuples
    |> Enum.map(fn {x, y, z} -> [x, y, z] end)
    |> Nx.tensor(type: :f64)
  end

  @doc """
  Generate points for a straight chain along the +X axis starting at origin.

  ## Options

  - `:lengths` - list of segment lengths (required)
  - `:start` - starting point (default: `{0, 0, 0}`)

  ## Example

      iex> straight_chain(lengths: [0.3, 0.2])
      # Points at: {0,0,0}, {0.3,0,0}, {0.5,0,0}
  """
  @spec straight_chain(keyword()) :: Nx.Tensor.t()
  def straight_chain(opts) do
    lengths = Keyword.fetch!(opts, :lengths)
    {sx, sy, sz} = Keyword.get(opts, :start, {0.0, 0.0, 0.0})

    {point_list, _} =
      Enum.reduce(lengths, {[{sx, sy, sz}], sx}, fn len, {acc, x} ->
        new_x = x + len
        {acc ++ [{new_x, sy, sz}], new_x}
      end)

    points(point_list)
  end

  @doc """
  Generate points for a chain with bends at each joint.

  Each joint bends in the XY plane (around Z axis) by the specified angle.
  Angles are cumulative - each segment's direction is the previous direction
  plus the bend angle.

  ## Options

  - `:lengths` - list of segment lengths (required)
  - `:angles` - list of bend angles in radians, one per segment (required)
                First angle is absolute direction from origin,
                subsequent angles are relative bends.
  - `:start` - starting point (default: `{0, 0, 0}`)

  ## Example

      # Straight arm then 90° elbow bend
      iex> bent_chain(lengths: [0.3, 0.2], angles: [0.0, :math.pi/2])
      # Points at: {0,0,0}, {0.3,0,0}, {0.3,0.2,0}
  """
  @spec bent_chain(keyword()) :: Nx.Tensor.t()
  def bent_chain(opts) do
    lengths = Keyword.fetch!(opts, :lengths)
    angles = Keyword.fetch!(opts, :angles)
    {sx, sy, sz} = Keyword.get(opts, :start, {0.0, 0.0, 0.0})

    if length(lengths) != length(angles) do
      raise ArgumentError, "lengths and angles must have same length"
    end

    {point_list, _, _} =
      Enum.zip(lengths, angles)
      |> Enum.reduce({[{sx, sy, sz}], {sx, sy}, 0.0}, fn {len, bend}, {acc, {x, y}, dir} ->
        new_dir = dir + bend
        new_x = x + len * :math.cos(new_dir)
        new_y = y + len * :math.sin(new_dir)
        {acc ++ [{new_x, new_y, sz}], {new_x, new_y}, new_dir}
      end)

    points(point_list)
  end

  @doc """
  Rotate all points around an axis passing through the origin.

  ## Parameters

  - `points` - Nx tensor of shape {n, 3}
  - `angle` - rotation angle in radians
  - `axis` - `:x`, `:y`, or `:z`

  ## Example

      iex> straight_chain(lengths: [1.0]) |> rotate(:math.pi/2, :z)
      # Rotates from +X to +Y direction
  """
  @spec rotate(Nx.Tensor.t(), number(), :x | :y | :z) :: Nx.Tensor.t()
  def rotate(points, angle, axis) do
    rotation_matrix = rotation_matrix(angle, axis)
    Nx.dot(points, Nx.transpose(rotation_matrix))
  end

  @doc """
  Generate a chain that's straight for the first segment, then bends.

  Convenience function for the common case of testing elbow-like joints.

  ## Options

  - `:lengths` - list of segment lengths (required)
  - `:bend_at` - index of joint where bend occurs (0-indexed, default: 1)
  - `:bend_angle` - angle of bend in radians (required)

  ## Example

      iex> elbow_bend(lengths: [0.3, 0.2], bend_angle: :math.pi/2)
      # Straight along X, then bent 90° at elbow
  """
  @spec elbow_bend(keyword()) :: Nx.Tensor.t()
  def elbow_bend(opts) do
    lengths = Keyword.fetch!(opts, :lengths)
    bend_angle = Keyword.fetch!(opts, :bend_angle)
    bend_at = Keyword.get(opts, :bend_at, 1)

    angles =
      lengths
      |> Enum.with_index()
      |> Enum.map(fn {_len, idx} ->
        if idx >= bend_at, do: bend_angle, else: 0.0
      end)

    bent_chain(lengths: lengths, angles: angles)
  end

  @doc """
  Generate points that are all at the same location (degenerate case).

  Useful for testing edge cases where direction vectors have zero length.
  """
  @spec collapsed(non_neg_integer(), {number(), number(), number()}) :: Nx.Tensor.t()
  def collapsed(n, point \\ {0.0, 0.0, 0.0}) do
    List.duplicate(point, n) |> points()
  end

  defp rotation_matrix(angle, :z) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    Nx.tensor(
      [
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0]
      ],
      type: :f64
    )
  end

  defp rotation_matrix(angle, :y) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    Nx.tensor(
      [
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c]
      ],
      type: :f64
    )
  end

  defp rotation_matrix(angle, :x) do
    c = :math.cos(angle)
    s = :math.sin(angle)

    Nx.tensor(
      [
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c]
      ],
      type: :f64
    )
  end
end
