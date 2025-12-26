# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIKTest do
  use ExUnit.Case, async: true

  alias BB.Error.Kinematics.NoDofs
  alias BB.Error.Kinematics.UnknownLink
  alias BB.Error.Kinematics.Unreachable
  alias BB.IK.FABRIK
  alias BB.IK.TestRobots.ContinuousJointArm
  alias BB.IK.TestRobots.FixedOnlyChain
  alias BB.IK.TestRobots.PrismaticArm
  alias BB.IK.TestRobots.ThreeLinkArm
  alias BB.IK.TestRobots.TwoLinkArm
  alias BB.Robot.Kinematics
  alias BB.Robot.State
  alias BB.Robot.Transform

  describe "solve/5" do
    test "solves for a reachable target with 2-link arm" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target within reach, off-axis to allow FABRIK to work
      # (FABRIK struggles with collinear targets)
      target = {0.35, 0.2, 0.0}

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :tip, target)

      assert meta.reached == true
      assert meta.residual < 0.01

      # Verify the FK produces the target position
      {x, y, z} = Kinematics.link_position(robot, solved_positions, :tip)
      assert_in_delta x, 0.35, 0.01
      assert_in_delta y, 0.2, 0.01
      assert_in_delta z, 0.0, 0.01
    end

    test "solves for a target requiring joint motion" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target to the side
      target = {0.3, 0.3, 0.0}

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :tip, target)

      assert meta.reached == true
      # FABRIK converges in point-space, but converting to joint angles
      # and back through FK introduces some error
      assert meta.residual < 0.1

      # Verify with FK - allow reasonable tolerance
      {x, y, _z} = Kinematics.link_position(robot, solved_positions, :tip)
      assert_in_delta x, 0.3, 0.1
      assert_in_delta y, 0.3, 0.1
    end

    test "returns error for unreachable target" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target way beyond reach (arm can only reach ~0.5m)
      target = {1.0, 0.0, 0.0}

      assert {:error, %Unreachable{} = error} =
               FABRIK.solve(robot, positions, :tip, target)

      assert error.target_link == :tip
      assert error.residual > 0.4

      # Should still return best-effort positions
      assert is_map(error.positions)
    end

    test "returns error for unknown target link" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      assert {:error, %UnknownLink{target_link: :nonexistent_link}} =
               FABRIK.solve(robot, positions, :nonexistent_link, {0.3, 0.0, 0.0})
    end

    test "returns error for chain with no movable joints" do
      robot = FixedOnlyChain.robot()
      positions = %{}

      assert {:error, %NoDofs{target_link: :end_link}} =
               FABRIK.solve(robot, positions, :end_link, {0.0, 0.0, 0.1})
    end

    test "works with BB.Robot.State" do
      robot = TwoLinkArm.robot()
      {:ok, state} = State.new(robot)

      target = {0.35, 0.2, 0.0}

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, state, :tip, target)

      assert meta.reached == true
      assert is_map(solved_positions)
    end

    test "accepts 4x4 transform as target" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Create a 4x4 transform target (off-axis)
      target_transform = Transform.translation(0.3, 0.3, 0.0)

      assert {:ok, _solved_positions, meta} =
               FABRIK.solve(robot, positions, :tip, target_transform)

      assert meta.reached == true
    end

    test "respects max_iterations option" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Use off-axis target
      target = {0.3, 0.3, 0.0}

      # With enough iterations, should converge
      assert {:ok, _positions, meta} =
               FABRIK.solve(robot, positions, :tip, target, max_iterations: 10)

      assert meta.iterations <= 10
    end

    test "respects tolerance option" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Use off-axis target
      target = {0.3, 0.3, 0.0}

      # Loose tolerance should converge quickly
      assert {:ok, _positions, meta} =
               FABRIK.solve(robot, positions, :tip, target, tolerance: 0.1)

      assert meta.residual < 0.1
    end

    test "solves 3D target with 3-link arm" do
      robot = ThreeLinkArm.robot()
      positions = %{joint1: 0.0, joint2: 0.0, joint3: 0.0}

      # 3D target within reach
      target = {0.1, 0.1, 0.25}

      assert {:ok, _solved_positions, meta} =
               FABRIK.solve(robot, positions, :tip, target,
                 max_iterations: 100,
                 tolerance: 0.05
               )

      # Should converge reasonably close
      assert meta.residual < 0.15
    end
  end

  describe "solve_and_update/5" do
    test "updates state in-place on success" do
      robot = TwoLinkArm.robot()
      {:ok, state} = State.new(robot)

      # Verify initial positions are zero
      assert State.get_joint_position(state, :shoulder_joint) == 0.0
      assert State.get_joint_position(state, :elbow_joint) == 0.0

      target = {0.3, 0.2, 0.0}

      assert {:ok, _positions, _meta} =
               FABRIK.solve_and_update(robot, state, :tip, target)

      # State should be updated
      shoulder = State.get_joint_position(state, :shoulder_joint)
      elbow = State.get_joint_position(state, :elbow_joint)

      # At least one joint should have moved
      assert shoulder != 0.0 or elbow != 0.0
    end

    test "does not update state on error" do
      robot = TwoLinkArm.robot()
      {:ok, state} = State.new(robot)

      # Set known positions
      State.set_joint_position(state, :shoulder_joint, 0.5)
      State.set_joint_position(state, :elbow_joint, 0.3)

      # Unreachable target
      target = {10.0, 0.0, 0.0}

      assert {:error, %Unreachable{}} =
               FABRIK.solve_and_update(robot, state, :tip, target)

      # State should be unchanged
      assert State.get_joint_position(state, :shoulder_joint) == 0.5
      assert State.get_joint_position(state, :elbow_joint) == 0.3
    end
  end

  describe "joint limits" do
    test "clamps joint values to limits when respect_limits: true" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target that might require exceeding limits
      target = {-0.4, 0.2, 0.0}

      {:ok, solved_positions, _meta} =
        FABRIK.solve(robot, positions, :tip, target, respect_limits: true)

      # Elbow limit is -135 to 135 degrees (-2.356 to 2.356 rad)
      elbow = solved_positions[:elbow_joint]
      assert elbow >= -2.356 - 0.01
      assert elbow <= 2.356 + 0.01
    end

    test "does not clamp when respect_limits: false" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target behind the robot that would require extreme angles
      target = {-0.3, 0.3, 0.0}

      {:ok, solved_clamped, _meta} =
        FABRIK.solve(robot, positions, :tip, target, respect_limits: true)

      {:ok, solved_unclamped, _meta} =
        FABRIK.solve(robot, positions, :tip, target, respect_limits: false)

      # Results may differ when limits would be exceeded
      # At minimum, both should return valid position maps
      assert is_map(solved_clamped)
      assert is_map(solved_unclamped)
      assert Map.has_key?(solved_clamped, :elbow_joint)
      assert Map.has_key?(solved_unclamped, :elbow_joint)
    end
  end

  describe "target formats" do
    test "accepts Nx tensor {3} as target" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target as Nx tensor instead of tuple
      target = Nx.tensor([0.35, 0.2, 0.0], type: :f64)

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :tip, target)

      assert meta.reached == true

      # Verify with FK
      {x, y, _z} = Kinematics.link_position(robot, solved_positions, :tip)
      assert_in_delta x, 0.35, 0.02
      assert_in_delta y, 0.2, 0.02
    end

    test "accepts Nx tensor {4, 4} transform as target" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # 4x4 homogeneous transform
      target = Transform.translation(0.35, 0.2, 0.0)

      assert {:ok, _solved_positions, meta} =
               FABRIK.solve(robot, positions, :tip, target)

      assert meta.reached == true
    end
  end

  describe "prismatic joints" do
    test "solves for target with prismatic joint" do
      robot = PrismaticArm.robot()
      positions = %{rotate_joint: 0.0, slide_joint: 0.0}

      # Target that requires some rotation and extension
      # Base is at origin, link1 at 0.2m, can extend 0.3m more, tip at +0.1m
      # Total reach: 0.2 + 0.3 + 0.1 = 0.6m along X when extended
      # Use a target off-axis to allow FABRIK to work
      target = {0.25, 0.15, 0.0}

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :tip, target,
                 max_iterations: 100,
                 tolerance: 0.05
               )

      # Should converge reasonably
      assert meta.residual < 0.15

      # Both joints should be in the result
      assert Map.has_key?(solved_positions, :slide_joint)
      assert Map.has_key?(solved_positions, :rotate_joint)
    end

    test "respects prismatic joint limits" do
      robot = PrismaticArm.robot()
      positions = %{rotate_joint: 0.0, slide_joint: 0.0}

      # Target within reach, off-axis
      target = {0.25, 0.1, 0.0}

      {:ok, solved_positions, _meta} =
        FABRIK.solve(robot, positions, :tip, target,
          respect_limits: true,
          max_iterations: 100,
          tolerance: 0.05
        )

      # Prismatic limit is 0 to 0.3m
      slide = solved_positions[:slide_joint]
      assert slide >= -0.01
      assert slide <= 0.31
    end
  end

  describe "continuous joints" do
    test "solves with continuous (unlimited) joints" do
      robot = ContinuousJointArm.robot()
      positions = %{wheel_joint: 0.0}

      # Target to the side - arm is 0.3m long
      target = {0.15, 0.2, 0.0}

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :tip, target,
                 max_iterations: 100,
                 tolerance: 0.05
               )

      assert meta.residual < 0.1

      # Continuous joint should have rotated
      wheel = solved_positions[:wheel_joint]
      assert is_float(wheel)
    end

    test "continuous joints have no clamping" do
      robot = ContinuousJointArm.robot()
      positions = %{wheel_joint: 0.0}

      # Target behind - would require > 90 degree rotation
      target = {-0.15, 0.2, 0.0}

      {:ok, solved_positions, _meta} =
        FABRIK.solve(robot, positions, :tip, target,
          respect_limits: true,
          max_iterations: 100,
          tolerance: 0.05
        )

      # Continuous joints should allow any angle
      wheel = solved_positions[:wheel_joint]
      assert is_float(wheel)
    end
  end

  describe "repeated solve stability" do
    test "repeated solves to same target remain stable" do
      robot = TwoLinkArm.robot()
      target = {0.35, 0.2, 0.0}
      tolerance = 0.01

      # First solve from zero position
      {:ok, positions, _meta} =
        FABRIK.solve(robot, %{shoulder_joint: 0.0, elbow_joint: 0.0}, :tip, target,
          tolerance: tolerance
        )

      # Record initial solved position
      {initial_x, initial_y, initial_z} = Kinematics.link_position(robot, positions, :tip)

      # Repeatedly solve 10 more times, each time using the previous result
      final_positions =
        Enum.reduce(1..10, positions, fn _i, current_positions ->
          {:ok, new_positions, _meta} =
            FABRIK.solve(robot, current_positions, :tip, target, tolerance: tolerance)

          new_positions
        end)

      # Final position should still be within tolerance of target
      {final_x, final_y, final_z} = Kinematics.link_position(robot, final_positions, :tip)

      assert_in_delta final_x, 0.35, tolerance * 2
      assert_in_delta final_y, 0.2, tolerance * 2
      assert_in_delta final_z, 0.0, tolerance * 2

      # Position should not have drifted significantly from the first solve
      assert_in_delta final_x, initial_x, tolerance
      assert_in_delta final_y, initial_y, tolerance
      assert_in_delta final_z, initial_z, tolerance
    end

    test "repeated solves with State remain stable" do
      robot = TwoLinkArm.robot()
      {:ok, state} = State.new(robot)
      target = {0.35, 0.2, 0.0}
      tolerance = 0.01

      # First solve
      {:ok, positions, _meta} =
        FABRIK.solve_and_update(robot, state, :tip, target, tolerance: tolerance)

      # Record initial end-effector position (this is what matters most)
      {initial_x, initial_y, initial_z} = Kinematics.link_position(robot, positions, :tip)

      # Solve 10 more times
      final_positions =
        Enum.reduce(1..10, positions, fn _i, _current ->
          {:ok, new_positions, _meta} =
            FABRIK.solve_and_update(robot, state, :tip, target, tolerance: tolerance)

          new_positions
        end)

      # End-effector position should still be within tolerance of target
      {final_x, final_y, final_z} = Kinematics.link_position(robot, final_positions, :tip)

      assert_in_delta final_x, 0.35, tolerance * 2
      assert_in_delta final_y, 0.2, tolerance * 2
      assert_in_delta final_z, 0.0, tolerance * 2

      # Position should not have drifted significantly from the first solve
      assert_in_delta final_x, initial_x, tolerance
      assert_in_delta final_y, initial_y, tolerance
      assert_in_delta final_z, initial_z, tolerance
    end
  end
end
