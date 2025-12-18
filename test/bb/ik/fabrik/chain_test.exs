# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.ChainTest do
  use ExUnit.Case, async: true

  alias BB.IK.FABRIK.Chain
  alias BB.IK.TestHelpers.Chain, as: ChainHelpers
  alias BB.IK.TestRobots.FixedOnlyChain
  alias BB.IK.TestRobots.ThreeLinkArm
  alias BB.IK.TestRobots.TwoLinkArm

  describe "build/3" do
    test "builds chain for valid target link" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      assert {:ok, chain} = Chain.build(robot, positions, :tip)

      assert length(chain.joint_names) == 2
      assert :shoulder_joint in chain.joint_names
      assert :elbow_joint in chain.joint_names
      assert Nx.shape(chain.points) == {3, 3}
      assert Nx.shape(chain.lengths) == {2}
    end

    test "returns error for unknown link" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      assert {:error, :unknown_link} = Chain.build(robot, positions, :nonexistent)
    end

    test "returns error for chain with only fixed joints" do
      robot = FixedOnlyChain.robot()
      positions = %{}

      assert {:error, :no_dofs} = Chain.build(robot, positions, :end_link)
    end

    test "filters out fixed joints from chain" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      {:ok, chain} = Chain.build(robot, positions, :tip)

      # tip_joint is fixed, should not be in the chain
      refute :tip_joint in chain.joint_names
    end

    test "computes segment lengths correctly" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      {:ok, chain} = Chain.build(robot, positions, :tip)

      lengths = Nx.to_flat_list(chain.lengths)

      # TwoLinkArm has segments of 0.3m and 0.2m
      assert_in_delta Enum.at(lengths, 0), 0.3, 0.01
      assert_in_delta Enum.at(lengths, 1), 0.2, 0.01
    end

    test "stores joint limits" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      {:ok, chain} = Chain.build(robot, positions, :tip)

      assert length(chain.limits) == 2

      # First joint (shoulder) has -180 to 180 degree limits
      shoulder_limits = Enum.at(chain.limits, 0)
      assert_in_delta shoulder_limits.lower, -:math.pi(), 0.01
      assert_in_delta shoulder_limits.upper, :math.pi(), 0.01

      # Second joint (elbow) has -135 to 135 degree limits
      elbow_limits = Enum.at(chain.limits, 1)
      assert_in_delta elbow_limits.lower, -2.356, 0.01
      assert_in_delta elbow_limits.upper, 2.356, 0.01
    end

    test "builds chain for 3-DOF arm" do
      robot = ThreeLinkArm.robot()
      positions = %{joint1: 0.0, joint2: 0.0, joint3: 0.0}

      assert {:ok, chain} = Chain.build(robot, positions, :tip)

      assert length(chain.joint_names) == 3
      assert :joint1 in chain.joint_names
      assert :joint2 in chain.joint_names
      assert :joint3 in chain.joint_names
    end

    test "points reflect current joint positions" do
      robot = TwoLinkArm.robot()

      # At zero position, chain is straight along X axis
      positions_zero = %{shoulder_joint: 0.0, elbow_joint: 0.0}
      {:ok, chain_zero} = Chain.build(robot, positions_zero, :tip)

      # At 90 degrees shoulder rotation, chain should be along Y axis
      positions_rotated = %{shoulder_joint: :math.pi() / 2, elbow_joint: 0.0}
      {:ok, chain_rotated} = Chain.build(robot, positions_rotated, :tip)

      # Get end points
      end_zero = Nx.slice(chain_zero.points, [2, 0], [1, 3]) |> Nx.to_flat_list()
      end_rotated = Nx.slice(chain_rotated.points, [2, 0], [1, 3]) |> Nx.to_flat_list()

      # Zero position: end should be along +X
      [x0, y0, _z0] = end_zero
      assert x0 > 0.4
      assert abs(y0) < 0.01

      # Rotated: end should be along +Y
      [x1, y1, _z1] = end_rotated
      assert abs(x1) < 0.01
      assert y1 > 0.4
    end
  end

  describe "points_to_positions/4" do
    test "computes revolute joint angles from solved points" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}
      {:ok, chain} = Chain.build(robot, positions, :tip)

      # Rotate the whole chain 45 degrees around Z axis
      angle = :math.pi() / 4

      solved_points =
        ChainHelpers.straight_chain(lengths: [0.3, 0.2])
        |> ChainHelpers.rotate(angle, :z)

      result = Chain.points_to_positions(robot, chain, solved_points, true)

      # Shoulder should be approximately 45 degrees
      assert_in_delta result[:shoulder_joint], angle, 0.1
      # Elbow computed - result depends on relative angle change
      assert is_float(result[:elbow_joint])
    end

    test "computes elbow bend angle correctly" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}
      {:ok, chain} = Chain.build(robot, positions, :tip)

      # Straight first segment, then 90° bend at elbow
      solved_points = ChainHelpers.elbow_bend(lengths: [0.3, 0.2], bend_angle: :math.pi() / 2)

      result = Chain.points_to_positions(robot, chain, solved_points, true)

      # Shoulder should be near zero (first segment still along X)
      assert_in_delta result[:shoulder_joint], 0.0, 0.1
      # Elbow should be approximately 90 degrees (pi/2)
      assert_in_delta result[:elbow_joint], :math.pi() / 2, 0.1
    end

    test "clamps values to limits when respect_limits is true" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}
      {:ok, chain} = Chain.build(robot, positions, :tip)

      # Create points that would require angles beyond limits
      # 180° shoulder rotation and extreme elbow position
      solved_points =
        ChainHelpers.points([
          {0.0, 0.0, 0.0},
          {-0.3, 0.0, 0.0},
          {-0.1, 0.0, 0.0}
        ])

      result = Chain.points_to_positions(robot, chain, solved_points, true)

      # Elbow limit is -135 to 135 degrees (-2.356 to 2.356 rad)
      assert result[:elbow_joint] >= -2.356 - 0.01
      assert result[:elbow_joint] <= 2.356 + 0.01
    end

    test "does not clamp when respect_limits is false" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}
      {:ok, chain} = Chain.build(robot, positions, :tip)

      # Points representing a full 180° elbow bend (exceeds 135° limit)
      solved_points =
        ChainHelpers.points([
          {0.0, 0.0, 0.0},
          {0.3, 0.0, 0.0},
          {0.1, 0.0, 0.0}
        ])

      result_unclamped = Chain.points_to_positions(robot, chain, solved_points, false)
      result_clamped = Chain.points_to_positions(robot, chain, solved_points, true)

      # Without clamping, elbow angle should be larger magnitude
      # With clamping, it should be limited to 2.356
      assert abs(result_unclamped[:elbow_joint]) >= abs(result_clamped[:elbow_joint]) - 0.01
    end

    test "handles zero-length direction vectors gracefully" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}
      {:ok, chain} = Chain.build(robot, positions, :tip)

      # All points collapsed to origin (degenerate case)
      solved_points = ChainHelpers.collapsed(3)

      # Should not crash, returns some value
      result = Chain.points_to_positions(robot, chain, solved_points, true)
      assert is_map(result)
      assert Map.has_key?(result, :shoulder_joint)
      assert Map.has_key?(result, :elbow_joint)
    end

    test "handles negative bend angles" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}
      {:ok, chain} = Chain.build(robot, positions, :tip)

      # Bend elbow in the opposite direction (-90°)
      solved_points = ChainHelpers.elbow_bend(lengths: [0.3, 0.2], bend_angle: -:math.pi() / 2)

      result = Chain.points_to_positions(robot, chain, solved_points, true)

      # Elbow should be approximately -90 degrees
      assert_in_delta result[:elbow_joint], -:math.pi() / 2, 0.1
    end
  end
end
