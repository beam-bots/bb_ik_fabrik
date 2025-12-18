# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.MathTest do
  use ExUnit.Case, async: true

  alias BB.IK.FABRIK.Math

  describe "fabrik/5" do
    test "converges for simple 2-segment chain to reachable target" do
      # Two segments: base at origin, lengths 1.0 and 1.0
      points =
        Nx.tensor(
          [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0]
          ],
          type: :f64
        )

      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      target = Nx.tensor([1.5, 1.0, 0.0], type: :f64)

      assert {:ok, solved_points, meta} = Math.fabrik(points, lengths, target, 50, 1.0e-4)

      assert meta.iterations > 0
      assert meta.residual < 1.0e-4

      # End effector should be at target
      end_effector = Nx.slice(solved_points, [2, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      diff = Nx.subtract(end_effector, target) |> Nx.LinAlg.norm() |> Nx.to_number()
      assert diff < 1.0e-4

      # Root should remain at origin
      root = Nx.slice(solved_points, [0, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      root_diff = Nx.LinAlg.norm(root) |> Nx.to_number()
      assert root_diff < 1.0e-6
    end

    test "returns unreachable error when target is too far" do
      points =
        Nx.tensor(
          [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0]
          ],
          type: :f64
        )

      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      # Target at distance 5.0, but chain can only reach 2.0
      target = Nx.tensor([5.0, 0.0, 0.0], type: :f64)

      assert {:error, :unreachable, meta} = Math.fabrik(points, lengths, target, 50, 1.0e-4)

      assert meta.residual > 2.0
      assert Map.has_key?(meta, :points)
    end

    test "returns max_iterations error when not converged" do
      points =
        Nx.tensor(
          [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0]
          ],
          type: :f64
        )

      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      target = Nx.tensor([1.5, 1.0, 0.0], type: :f64)

      # Only 1 iteration with tight tolerance
      assert {:error, :max_iterations, meta} = Math.fabrik(points, lengths, target, 1, 1.0e-10)

      assert meta.iterations == 1
    end

    test "handles single-segment chain" do
      points =
        Nx.tensor(
          [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0]
          ],
          type: :f64
        )

      lengths = Nx.tensor([1.0], type: :f64)
      target = Nx.tensor([0.0, 1.0, 0.0], type: :f64)

      assert {:ok, solved_points, meta} = Math.fabrik(points, lengths, target, 50, 1.0e-4)

      assert meta.residual < 1.0e-4

      # End effector should be at target
      end_effector = Nx.slice(solved_points, [1, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      diff = Nx.subtract(end_effector, target) |> Nx.LinAlg.norm() |> Nx.to_number()
      assert diff < 1.0e-4
    end

    test "handles 3D target" do
      points =
        Nx.tensor(
          [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 2.0]
          ],
          type: :f64
        )

      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      target = Nx.tensor([1.0, 1.0, 1.0], type: :f64)

      assert {:ok, solved_points, meta} = Math.fabrik(points, lengths, target, 50, 1.0e-4)

      assert meta.residual < 1.0e-4

      end_effector = Nx.slice(solved_points, [2, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      diff = Nx.subtract(end_effector, target) |> Nx.LinAlg.norm() |> Nx.to_number()
      assert diff < 1.0e-4
    end

    test "converges for longer chain" do
      # 5-segment chain
      points =
        Nx.tensor(
          [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [5.0, 0.0, 0.0]
          ],
          type: :f64
        )

      lengths = Nx.tensor([1.0, 1.0, 1.0, 1.0, 1.0], type: :f64)
      target = Nx.tensor([3.0, 3.0, 0.0], type: :f64)

      assert {:ok, _solved_points, meta} = Math.fabrik(points, lengths, target, 100, 1.0e-4)

      assert meta.residual < 1.0e-4
    end

    test "preserves segment lengths" do
      points =
        Nx.tensor(
          [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0]
          ],
          type: :f64
        )

      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      target = Nx.tensor([1.0, 1.5, 0.0], type: :f64)

      assert {:ok, solved_points, _meta} = Math.fabrik(points, lengths, target, 50, 1.0e-4)

      # Check segment lengths are preserved
      p0 = Nx.slice(solved_points, [0, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      p1 = Nx.slice(solved_points, [1, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      p2 = Nx.slice(solved_points, [2, 0], [1, 3]) |> Nx.squeeze(axes: [0])

      len1 = Nx.subtract(p1, p0) |> Nx.LinAlg.norm() |> Nx.to_number()
      len2 = Nx.subtract(p2, p1) |> Nx.LinAlg.norm() |> Nx.to_number()

      assert_in_delta len1, 1.0, 1.0e-6
      assert_in_delta len2, 1.0, 1.0e-6
    end
  end
end
