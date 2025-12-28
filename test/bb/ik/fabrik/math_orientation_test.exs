# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.MathOrientationTest do
  use ExUnit.Case, async: true

  alias BB.IK.FABRIK.Math
  alias BB.Math.Quaternion
  alias BB.Math.Vec3

  describe "points_to_frames/1" do
    test "creates frames with identity orientations" do
      points = Nx.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], type: :f64)

      frames = Math.points_to_frames(points)

      assert frames.positions == points
      assert Nx.shape(frames.orientations) == {3, 4}

      # All orientations should be identity [1, 0, 0, 0]
      for i <- 0..2 do
        ori = Nx.slice(frames.orientations, [i, 0], [1, 4]) |> Nx.squeeze(axes: [0])
        assert_in_delta Nx.to_number(ori[0]), 1.0, 0.0001
        assert_in_delta Nx.to_number(ori[1]), 0.0, 0.0001
        assert_in_delta Nx.to_number(ori[2]), 0.0, 0.0001
        assert_in_delta Nx.to_number(ori[3]), 0.0, 0.0001
      end
    end
  end

  describe "frames_to_points/1" do
    test "extracts positions from frames" do
      points = Nx.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], type: :f64)
      frames = Math.points_to_frames(points)

      result = Math.frames_to_points(frames)

      assert result == points
    end
  end

  describe "fabrik_with_orientation/7" do
    test "solves position-only target (nil orientation)" do
      # Simple 2-segment chain along X-axis
      points = Nx.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], type: :f64)
      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      frames = Math.points_to_frames(points)

      # Target off-axis
      target_position = Nx.tensor([1.5, 0.5, 0.0], type: :f64)

      assert {:ok, solved_frames, meta} =
               Math.fabrik_with_orientation(frames, lengths, target_position, nil, 50, 0.01)

      # Should converge
      assert meta.residual < 0.01
      assert meta.orientation_residual == nil

      # End-effector should be at target
      end_pos = Nx.slice(solved_frames.positions, [2, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      assert_in_delta Nx.to_number(end_pos[0]), 1.5, 0.02
      assert_in_delta Nx.to_number(end_pos[1]), 0.5, 0.02
    end

    test "reports orientation residual when orientation target is specified" do
      # 2-segment chain - not enough DOF for arbitrary orientation
      points = Nx.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], type: :f64)
      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      frames = Math.points_to_frames(points)

      # Target with specific orientation (won't converge with 2 segments)
      target_position = Nx.tensor([1.5, 0.5, 0.0], type: :f64)
      target_orientation = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 4)

      # Use loose orientation tolerance - main test is that residual is computed
      result =
        Math.fabrik_with_orientation(
          frames,
          lengths,
          target_position,
          target_orientation,
          100,
          0.01,
          orientation_tolerance: 10.0
        )

      # Either converges (with loose tolerance) or hits max iterations
      meta =
        case result do
          {:ok, _frames, meta} -> meta
          {:error, :max_iterations, meta} -> meta
        end

      # Position should converge
      assert meta.residual < 0.01

      # Orientation residual should be reported (not nil)
      assert is_float(meta.orientation_residual)
      assert meta.orientation_residual >= 0
    end

    test "returns unreachable for target beyond reach" do
      points = Nx.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], type: :f64)
      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      frames = Math.points_to_frames(points)

      # Target way beyond reach (chain can reach max 2.0m)
      target_position = Nx.tensor([5.0, 0.0, 0.0], type: :f64)

      assert {:error, :unreachable, meta} =
               Math.fabrik_with_orientation(frames, lengths, target_position, nil, 50, 0.01)

      assert meta.residual > 2.0
      assert meta.iterations == 0
    end

    test "returns max_iterations when not converged" do
      points = Nx.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], type: :f64)
      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      frames = Math.points_to_frames(points)

      target_position = Nx.tensor([1.5, 0.5, 0.0], type: :f64)

      # Use impossibly tight tolerance with few iterations
      assert {:error, :max_iterations, meta} =
               Math.fabrik_with_orientation(frames, lengths, target_position, nil, 2, 0.0000001)

      assert meta.iterations == 2
    end

    test "maintains segment lengths during solve" do
      points = Nx.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], type: :f64)
      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      frames = Math.points_to_frames(points)

      target_position = Nx.tensor([1.0, 1.0, 0.0], type: :f64)

      {:ok, solved_frames, _meta} =
        Math.fabrik_with_orientation(frames, lengths, target_position, nil, 50, 0.01)

      # Check segment lengths are preserved
      p0 = Nx.slice(solved_frames.positions, [0, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      p1 = Nx.slice(solved_frames.positions, [1, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      p2 = Nx.slice(solved_frames.positions, [2, 0], [1, 3]) |> Nx.squeeze(axes: [0])

      len1 = Nx.LinAlg.norm(Nx.subtract(p1, p0)) |> Nx.to_number()
      len2 = Nx.LinAlg.norm(Nx.subtract(p2, p1)) |> Nx.to_number()

      assert_in_delta len1, 1.0, 0.001
      assert_in_delta len2, 1.0, 0.001
    end

    test "root position is preserved" do
      points = Nx.tensor([[1.0, 2.0, 3.0], [2.0, 2.0, 3.0], [3.0, 2.0, 3.0]], type: :f64)
      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      frames = Math.points_to_frames(points)

      target_position = Nx.tensor([2.5, 2.5, 3.0], type: :f64)

      {:ok, solved_frames, _meta} =
        Math.fabrik_with_orientation(frames, lengths, target_position, nil, 50, 0.01)

      # Root should be at original position
      root = Nx.slice(solved_frames.positions, [0, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      assert_in_delta Nx.to_number(root[0]), 1.0, 0.0001
      assert_in_delta Nx.to_number(root[1]), 2.0, 0.0001
      assert_in_delta Nx.to_number(root[2]), 3.0, 0.0001
    end

    test "3D target solving" do
      # 3-segment chain
      points =
        Nx.tensor(
          [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
          type: :f64
        )

      lengths = Nx.tensor([1.0, 1.0, 1.0], type: :f64)
      frames = Math.points_to_frames(points)

      # 3D target
      target_position = Nx.tensor([1.5, 1.0, 1.0], type: :f64)

      {:ok, solved_frames, meta} =
        Math.fabrik_with_orientation(frames, lengths, target_position, nil, 100, 0.05)

      assert meta.residual < 0.05

      # End-effector should be near target
      end_pos = Nx.slice(solved_frames.positions, [3, 0], [1, 3]) |> Nx.squeeze(axes: [0])
      assert_in_delta Nx.to_number(end_pos[0]), 1.5, 0.1
      assert_in_delta Nx.to_number(end_pos[1]), 1.0, 0.1
      assert_in_delta Nx.to_number(end_pos[2]), 1.0, 0.1
    end
  end

  describe "repeated solve stability with orientation" do
    test "repeated solves to same target remain stable" do
      points = Nx.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], type: :f64)
      lengths = Nx.tensor([1.0, 1.0], type: :f64)
      frames = Math.points_to_frames(points)

      target_position = Nx.tensor([1.5, 0.5, 0.0], type: :f64)
      tolerance = 0.01

      # First solve
      {:ok, solved_frames, _meta} =
        Math.fabrik_with_orientation(frames, lengths, target_position, nil, 50, tolerance)

      initial_end_pos =
        Nx.slice(solved_frames.positions, [2, 0], [1, 3]) |> Nx.squeeze(axes: [0])

      # Solve again from solved position
      final_frames =
        Enum.reduce(1..5, solved_frames, fn _i, current_frames ->
          {:ok, new_frames, _meta} =
            Math.fabrik_with_orientation(
              current_frames,
              lengths,
              target_position,
              nil,
              50,
              tolerance
            )

          new_frames
        end)

      final_end_pos = Nx.slice(final_frames.positions, [2, 0], [1, 3]) |> Nx.squeeze(axes: [0])

      # Should not have drifted
      assert_in_delta Nx.to_number(final_end_pos[0]), Nx.to_number(initial_end_pos[0]), tolerance
      assert_in_delta Nx.to_number(final_end_pos[1]), Nx.to_number(initial_end_pos[1]), tolerance
      assert_in_delta Nx.to_number(final_end_pos[2]), Nx.to_number(initial_end_pos[2]), tolerance
    end
  end
end
