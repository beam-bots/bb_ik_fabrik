# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.ChainTest do
  use ExUnit.Case, async: true

  alias BB.Error.Kinematics.NoDofs
  alias BB.Error.Kinematics.UnknownLink
  alias BB.IK.FABRIK.Chain
  alias BB.IK.TestRobots.ContinuousJointArm
  alias BB.IK.TestRobots.FixedOnlyChain
  alias BB.IK.TestRobots.ThreeLinkArm
  alias BB.IK.TestRobots.TwoLinkArm
  alias BB.Math.Transform
  alias BB.Robot.Kinematics

  describe "build/3" do
    test "builds chain for valid target link" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      assert {:ok, chain} = Chain.build(robot, positions, :base_link, :tip)

      assert length(chain.joint_names) == 2
      assert :shoulder_joint in chain.joint_names
      assert :elbow_joint in chain.joint_names
    end

    test "returns error for unknown link" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      assert {:error, %UnknownLink{link: :nonexistent, role: :target}} =
               Chain.build(robot, positions, :base_link, :nonexistent)
    end

    test "returns error for chain with only fixed joints" do
      robot = FixedOnlyChain.robot()
      positions = %{}

      assert {:error, %NoDofs{target_link: :end_link}} =
               Chain.build(robot, positions, :base_link, :end_link)
    end

    test "filters out fixed joints from chain" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      {:ok, chain} = Chain.build(robot, positions, :base_link, :tip)

      # tip_joint is fixed, should not be in the chain
      refute :tip_joint in chain.joint_names
    end

    # Reach is what the solver needs the point chain for: a target further from
    # the root than this cannot be touched however the joints are arranged.
    test "measures how far the chain can reach" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      {:ok, chain} = Chain.build(robot, positions, :base_link, :tip)

      # Segments of 0.3m and 0.2m, laid out straight.
      assert_in_delta chain.reach, 0.5, 0.01
      assert Nx.to_flat_list(chain.root_point) == [0.0, 0.0, 0.0]
    end

    test "stores joint limits" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      {:ok, chain} = Chain.build(robot, positions, :base_link, :tip)
      k = Chain.kinematics(chain)

      [shoulder_lower, elbow_lower] = Nx.to_flat_list(k.limits_lower)
      [shoulder_upper, elbow_upper] = Nx.to_flat_list(k.limits_upper)

      assert_in_delta shoulder_lower, -:math.pi(), 0.01
      assert_in_delta shoulder_upper, :math.pi(), 0.01
      assert_in_delta elbow_lower, -2.356, 0.01
      assert_in_delta elbow_upper, 2.356, 0.01
    end

    test "builds chain for 3-DOF arm" do
      robot = ThreeLinkArm.robot()
      positions = %{joint1: 0.0, joint2: 0.0, joint3: 0.0}

      assert {:ok, chain} = Chain.build(robot, positions, :base_link, :tip)

      assert length(chain.joint_names) == 3
      assert :joint1 in chain.joint_names
      assert :joint2 in chain.joint_names
      assert :joint3 in chain.joint_names
    end
  end

  describe "kinematics/1" do
    # Fitting a joint needs its world axis and pivot under the values already
    # chosen upstream, which means every intermediate link transform rather than
    # just the tip. This pins the tensor description handed to bb's `defn`
    # kinematics against bb's own eager forward kinematics.
    test "describes the chain so bb's defn kinematics reproduces eager FK" do
      robot = ThreeLinkArm.robot()
      configurations = %{joint1: 0.3, joint2: 0.4, joint3: 0.5}

      {:ok, chain} = Chain.build(robot, configurations, :base_link, :tip)
      k = Chain.kinematics(chain)

      transforms =
        Kinematics.Defn.link_transforms(
          k.positions,
          k.origin_rpy,
          k.origin_xyz,
          k.axes,
          k.is_revolute,
          k.is_prismatic,
          k.stored,
          k.deltas,
          k.parent_idx
        )

      # The scan is relative to the chain's root, so compose the source link's
      # own world transform back on before comparing.
      tip = Nx.dot(Transform.tensor(chain.root_transform), transforms[k.target_row])
      {x, y, z} = Kinematics.link_position(robot, configurations, :tip)

      for {actual, expected} <- Enum.zip(Nx.to_flat_list(tip[[0..2, 3]]), [x, y, z]) do
        assert_in_delta actual, expected, 1.0e-12
      end
    end

    test "maps rows back to joints, points and the target link" do
      robot = ThreeLinkArm.robot()

      {:ok, chain} =
        Chain.build(robot, %{joint1: 0.0, joint2: 0.0, joint3: 0.0}, :base_link, :tip)

      k = Chain.kinematics(chain)

      # base_link, link1, link2, link3, tip. The fixed tip_joint earns a row like
      # any other, which is what makes a target link sitting past one ordinary
      # rather than a special case.
      assert Nx.to_flat_list(k.joint_rows) == [1, 2, 3]
      assert Nx.to_number(k.target_row) == 4
      assert Nx.to_flat_list(k.point_rows) == [0, 1, 2, 3, 4]
    end

    # Clamping should be the same arithmetic whether or not a limit was declared,
    # so an unlimited joint gets an infinite bound rather than a sentinel.
    test "gives a continuous joint infinite bounds" do
      robot = ContinuousJointArm.robot()
      {:ok, chain} = Chain.build(robot, %{wheel_joint: 0.0}, :base_link, :tip)
      k = Chain.kinematics(chain)

      assert Nx.to_flat_list(k.limits_lower) == [:neg_infinity]
      assert Nx.to_flat_list(k.limits_upper) == [:infinity]
    end
  end
end
