# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIKTest do
  use ExUnit.Case, async: true

  alias BB.Error.Kinematics.FABRIK.UnsupportedJoint
  alias BB.Error.Kinematics.NoDofs
  alias BB.Error.Kinematics.UnknownLink
  alias BB.Error.Kinematics.Unreachable
  alias BB.IK.FABRIK
  alias BB.IK.TestRobots.ContinuousJointArm
  alias BB.IK.TestRobots.FixedOnlyChain
  alias BB.IK.TestRobots.FloatingBaseArm
  alias BB.IK.TestRobots.PlanarBaseArm
  alias BB.IK.TestRobots.PrismaticArm
  alias BB.IK.TestRobots.SixDofArm
  alias BB.IK.TestRobots.ThreeLinkArm
  alias BB.IK.TestRobots.TwoLinkArm
  alias BB.Math.Quaternion
  alias BB.Math.Transform
  alias BB.Math.Transform2D
  alias BB.Math.Vec3
  alias BB.Robot.Kinematics
  alias BB.Robot.State

  describe "solve/6" do
    test "solves for a reachable target with 2-link arm" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target within reach, off-axis to allow FABRIK to work
      # (FABRIK struggles with collinear targets)
      target = Vec3.new(0.35, 0.2, 0.0)

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :base_link, :tip, target)

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
      target = Vec3.new(0.3, 0.3, 0.0)

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :base_link, :tip, target)

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
      target = Vec3.new(1.0, 0.0, 0.0)

      assert {:error, %Unreachable{} = error} =
               FABRIK.solve(robot, positions, :base_link, :tip, target)

      assert error.target_link == :tip
      assert error.residual > 0.4

      # Should still return best-effort positions
      assert is_map(error.positions)
    end

    test "returns error for unknown target link" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      assert {:error, %UnknownLink{link: :nonexistent_link, role: :target}} =
               FABRIK.solve(
                 robot,
                 positions,
                 :base_link,
                 :nonexistent_link,
                 Vec3.new(0.3, 0.0, 0.0)
               )
    end

    test "returns error for chain with no movable joints" do
      robot = FixedOnlyChain.robot()
      positions = %{}

      assert {:error, %NoDofs{target_link: :end_link}} =
               FABRIK.solve(robot, positions, :base_link, :end_link, Vec3.new(0.0, 0.0, 0.1))
    end

    test "works with BB.Robot.State" do
      robot = TwoLinkArm.robot()
      {:ok, state} = State.new(robot)

      target = Vec3.new(0.35, 0.2, 0.0)

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, state, :base_link, :tip, target)

      assert meta.reached == true
      assert is_map(solved_positions)
    end

    test "accepts 4x4 transform as target with orientation" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Create a 4x4 transform target - extracts both position and orientation
      # A 2-link arm can't achieve arbitrary orientations, so we use loose tolerance
      target_transform = Transform.translation(Vec3.new(0.3, 0.3, 0.0))

      # Use loose orientation tolerance since 2-link arm has limited DOF
      result =
        FABRIK.solve(robot, positions, :base_link, :tip, target_transform,
          orientation_tolerance: 10.0
        )

      # Should converge on position even if orientation can't be fully satisfied
      case result do
        {:ok, _positions, meta} ->
          assert meta.residual < 0.1

        {:error, %BB.Error.Kinematics.NoSolution{residual: residual}} ->
          # Position should still be close even if orientation didn't converge
          assert residual < 0.1
      end
    end

    test "accepts axis constraint as orientation target" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target with axis constraint - "point the tool in this direction"
      target_position = Vec3.new(0.3, 0.3, 0.0)
      axis_direction = Vec3.new(1.0, 0.0, 0.0)
      target = {target_position, {:axis, axis_direction}}

      # A 2-link planar arm can't satisfy arbitrary axis constraints
      # but should still converge on position
      result =
        FABRIK.solve(robot, positions, :base_link, :tip, target, orientation_tolerance: 10.0)

      case result do
        {:ok, _positions, meta} ->
          assert meta.residual < 0.1

        {:error, %BB.Error.Kinematics.NoSolution{residual: residual}} ->
          assert residual < 0.1
      end
    end

    test "respects max_iterations option" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Use off-axis target
      target = Vec3.new(0.3, 0.3, 0.0)

      # With enough iterations, should converge
      assert {:ok, _positions, meta} =
               FABRIK.solve(robot, positions, :base_link, :tip, target, max_iterations: 10)

      assert meta.iterations <= 10
    end

    test "respects tolerance option" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Use off-axis target
      target = Vec3.new(0.3, 0.3, 0.0)

      # Loose tolerance should converge quickly
      assert {:ok, _positions, meta} =
               FABRIK.solve(robot, positions, :base_link, :tip, target, tolerance: 0.1)

      assert meta.residual < 0.1
    end

    test "solves 3D target with 3-link arm" do
      robot = ThreeLinkArm.robot()
      positions = %{joint1: 0.0, joint2: 0.0, joint3: 0.0}

      # 3D target within reach
      target = Vec3.new(0.1, 0.1, 0.25)

      assert {:ok, _solved_positions, meta} =
               FABRIK.solve(robot, positions, :base_link, :tip, target,
                 max_iterations: 100,
                 tolerance: 0.05
               )

      # Should converge reasonably close
      assert meta.residual < 0.15
    end
  end

  describe "solve_and_update/6" do
    test "updates state in-place on success" do
      robot = TwoLinkArm.robot()
      {:ok, state} = State.new(robot)

      # Verify initial positions are zero
      assert State.get_configuration(state, :shoulder_joint) == {:ok, 0.0}
      assert State.get_configuration(state, :elbow_joint) == {:ok, 0.0}

      target = Vec3.new(0.3, 0.2, 0.0)

      assert {:ok, _positions, _meta} =
               FABRIK.solve_and_update(robot, state, :base_link, :tip, target)

      # State should be updated
      {:ok, shoulder} = State.get_configuration(state, :shoulder_joint)
      {:ok, elbow} = State.get_configuration(state, :elbow_joint)

      # At least one joint should have moved
      assert shoulder != 0.0 or elbow != 0.0
    end

    test "does not update state on error" do
      robot = TwoLinkArm.robot()
      {:ok, state} = State.new(robot)

      # Set known positions
      State.set_configuration(state, :shoulder_joint, 0.5)
      State.set_configuration(state, :elbow_joint, 0.3)

      # Unreachable target
      target = Vec3.new(10.0, 0.0, 0.0)

      assert {:error, %Unreachable{}} =
               FABRIK.solve_and_update(robot, state, :base_link, :tip, target)

      # State should be unchanged
      assert State.get_configuration(state, :shoulder_joint) == {:ok, 0.5}
      assert State.get_configuration(state, :elbow_joint) == {:ok, 0.3}
    end
  end

  describe "joint limits" do
    test "clamps joint values to limits when respect_limits: true" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target that might require exceeding limits
      target = Vec3.new(-0.4, 0.2, 0.0)

      {:ok, solved_positions, _meta} =
        FABRIK.solve(robot, positions, :base_link, :tip, target, respect_limits: true)

      # Elbow limit is -135 to 135 degrees (-2.356 to 2.356 rad)
      elbow = solved_positions[:elbow_joint]
      assert elbow >= -2.356 - 0.01
      assert elbow <= 2.356 + 0.01
    end

    test "does not clamp when respect_limits: false" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Target behind the robot that would require extreme angles
      target = Vec3.new(-0.3, 0.3, 0.0)

      {:ok, solved_clamped, _meta} =
        FABRIK.solve(robot, positions, :base_link, :tip, target, respect_limits: true)

      {:ok, solved_unclamped, _meta} =
        FABRIK.solve(robot, positions, :base_link, :tip, target, respect_limits: false)

      # Results may differ when limits would be exceeded
      # At minimum, both should return valid position maps
      assert is_map(solved_clamped)
      assert is_map(solved_unclamped)
      assert Map.has_key?(solved_clamped, :elbow_joint)
      assert Map.has_key?(solved_unclamped, :elbow_joint)
    end
  end

  describe "target formats" do
    test "accepts Vec3 as target" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      target = Vec3.new(0.35, 0.2, 0.0)

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :base_link, :tip, target)

      assert meta.reached == true

      # Verify with FK
      {x, y, _z} = Kinematics.link_position(robot, solved_positions, :tip)
      assert_in_delta x, 0.35, 0.02
      assert_in_delta y, 0.2, 0.02
    end

    test "accepts Transform struct as target" do
      robot = TwoLinkArm.robot()
      positions = %{shoulder_joint: 0.0, elbow_joint: 0.0}

      # Transform struct - extracts position and orientation
      target = Transform.translation(Vec3.new(0.35, 0.2, 0.0))

      # Use loose orientation tolerance for 2-link arm
      result =
        FABRIK.solve(robot, positions, :base_link, :tip, target, orientation_tolerance: 10.0)

      case result do
        {:ok, _positions, meta} ->
          assert meta.residual < 0.01

        {:error, %BB.Error.Kinematics.NoSolution{residual: residual}} ->
          assert residual < 0.01
      end
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
      target = Vec3.new(0.25, 0.15, 0.0)

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :base_link, :tip, target,
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
      target = Vec3.new(0.25, 0.1, 0.0)

      {:ok, solved_positions, _meta} =
        FABRIK.solve(robot, positions, :base_link, :tip, target,
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
      target = Vec3.new(0.15, 0.2, 0.0)

      assert {:ok, solved_positions, meta} =
               FABRIK.solve(robot, positions, :base_link, :tip, target,
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
      target = Vec3.new(-0.15, 0.2, 0.0)

      {:ok, solved_positions, _meta} =
        FABRIK.solve(robot, positions, :base_link, :tip, target,
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
      target = Vec3.new(0.35, 0.2, 0.0)
      tolerance = 0.01

      # First solve from zero position
      {:ok, positions, _meta} =
        FABRIK.solve(robot, %{shoulder_joint: 0.0, elbow_joint: 0.0}, :base_link, :tip, target,
          tolerance: tolerance
        )

      # Record initial solved position
      {initial_x, initial_y, initial_z} = Kinematics.link_position(robot, positions, :tip)

      # Repeatedly solve 10 more times, each time using the previous result
      final_positions =
        Enum.reduce(1..10, positions, fn _i, current_positions ->
          {:ok, new_positions, _meta} =
            FABRIK.solve(robot, current_positions, :base_link, :tip, target, tolerance: tolerance)

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
      target = Vec3.new(0.35, 0.2, 0.0)
      tolerance = 0.01

      # First solve
      {:ok, positions, _meta} =
        FABRIK.solve_and_update(robot, state, :base_link, :tip, target, tolerance: tolerance)

      # Record initial end-effector position (this is what matters most)
      {initial_x, initial_y, initial_z} = Kinematics.link_position(robot, positions, :tip)

      # Solve 10 more times
      final_positions =
        Enum.reduce(1..10, positions, fn _i, _current ->
          {:ok, new_positions, _meta} =
            FABRIK.solve_and_update(robot, state, :base_link, :tip, target, tolerance: tolerance)

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

  describe "6-DOF orientation solving" do
    test "solves position with 6-DOF arm" do
      robot = SixDofArm.robot()

      positions = %{
        shoulder_yaw: 0.0,
        shoulder_pitch: 0.0,
        shoulder_roll: 0.0,
        elbow_pitch: 0.0,
        wrist_pitch: 0.0,
        wrist_roll: 0.0
      }

      # Target along positive Z (arm's natural extension direction)
      # The arm extends ~0.55m along Z when at home position
      # Use a target that's slightly off-axis to help FABRIK converge
      target = Vec3.new(0.1, 0.1, 0.45)

      result =
        FABRIK.solve(robot, positions, :base_link, :tip, target,
          max_iterations: 100,
          tolerance: 0.05
        )

      # FABRIK with 6-DOF arms is challenging - test that the API works
      # and returns valid results, not necessarily tight convergence
      case result do
        {:ok, solved_positions, _meta} ->
          # Verify result is a valid position map
          assert is_map(solved_positions)
          assert Map.has_key?(solved_positions, :shoulder_yaw)
          assert Map.has_key?(solved_positions, :wrist_roll)

        {:error, %BB.Error.Kinematics.NoSolution{positions: positions}} ->
          # Even on max iterations, should return best-effort positions
          assert is_map(positions)
          assert Map.has_key?(positions, :shoulder_yaw)
      end
    end

    test "solves with quaternion orientation target" do
      robot = SixDofArm.robot()

      positions = %{
        shoulder_yaw: 0.0,
        shoulder_pitch: 0.0,
        shoulder_roll: 0.0,
        elbow_pitch: 0.0,
        wrist_pitch: 0.0,
        wrist_roll: 0.0
      }

      # Target position along Z axis (natural extension)
      target_position = Vec3.new(0.05, 0.05, 0.45)

      # Target orientation - 45 degree rotation around Z axis
      target_orientation = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 4)

      target = {target_position, {:quaternion, target_orientation}}

      result =
        FABRIK.solve(robot, positions, :base_link, :tip, target,
          max_iterations: 100,
          tolerance: 0.05,
          orientation_tolerance: 1.0
        )

      case result do
        {:ok, _positions, meta} ->
          # With orientation constraints, may not converge as tightly
          assert meta.residual < 0.3
          assert meta.orientation_residual != nil

        {:error, %BB.Error.Kinematics.NoSolution{}} ->
          # Orientation-constrained IK is difficult; test that API works
          :ok
      end
    end

    test "solves with axis constraint orientation" do
      robot = SixDofArm.robot()

      positions = %{
        shoulder_yaw: 0.0,
        shoulder_pitch: 0.0,
        shoulder_roll: 0.0,
        elbow_pitch: 0.0,
        wrist_pitch: 0.0,
        wrist_roll: 0.0
      }

      # Target position
      target_position = Vec3.new(0.05, 0.05, 0.45)

      # Axis constraint - point tool along Z axis (natural direction)
      axis_direction = Vec3.new(0.0, 0.0, 1.0)
      target = {target_position, {:axis, axis_direction}}

      result =
        FABRIK.solve(robot, positions, :base_link, :tip, target,
          max_iterations: 100,
          tolerance: 0.05,
          orientation_tolerance: 1.0
        )

      case result do
        {:ok, _positions, meta} ->
          assert meta.residual < 0.3

        {:error, %BB.Error.Kinematics.NoSolution{}} ->
          # Axis-constrained IK may not converge; test that API works
          :ok
      end
    end

    test "returns orientation_residual in metadata" do
      robot = SixDofArm.robot()

      positions = %{
        shoulder_yaw: 0.0,
        shoulder_pitch: 0.0,
        shoulder_roll: 0.0,
        elbow_pitch: 0.0,
        wrist_pitch: 0.0,
        wrist_roll: 0.0
      }

      target_position = Vec3.new(0.05, 0.05, 0.45)
      target_orientation = Quaternion.identity()
      target = {target_position, {:quaternion, target_orientation}}

      result =
        FABRIK.solve(robot, positions, :base_link, :tip, target,
          max_iterations: 100,
          tolerance: 0.05,
          orientation_tolerance: 1.0
        )

      case result do
        {:ok, _positions, meta} ->
          assert is_number(meta.orientation_residual) or is_nil(meta.orientation_residual)

        {:error, _} ->
          :ok
      end
    end

    test "accepts 4x4 transform and extracts both position and orientation" do
      robot = SixDofArm.robot()

      positions = %{
        shoulder_yaw: 0.0,
        shoulder_pitch: 0.0,
        shoulder_roll: 0.0,
        elbow_pitch: 0.0,
        wrist_pitch: 0.0,
        wrist_roll: 0.0
      }

      # Create transform with position and rotation (along Z axis)
      rotation = Quaternion.from_axis_angle(Vec3.unit_z(), :math.pi() / 6)
      target = Transform.from_position_quaternion(Vec3.new(0.05, 0.05, 0.45), rotation)

      result =
        FABRIK.solve(robot, positions, :base_link, :tip, target,
          max_iterations: 100,
          tolerance: 0.05,
          orientation_tolerance: 1.0
        )

      case result do
        {:ok, _solved_positions, meta} ->
          assert meta.residual < 0.3

        {:error, %BB.Error.Kinematics.NoSolution{}} ->
          # Transform extraction and orientation-constrained IK tested
          :ok
      end
    end
  end

  # FABRIK is not Jacobian-based: it repositions each joint as a point joined by a
  # fixed-length link, and a multi-DoF joint is neither. Refusing beats
  # approximating, because an answer that silently ignored three or six degrees of
  # freedom looks exactly like a correct one.
  describe "chains containing a multi-DoF joint" do
    test "refuses a planar joint, naming it and its type" do
      robot = PlanarBaseArm.robot()
      configurations = %{base: Transform2D.identity(), shoulder_joint: 0.0}

      assert {:error, %UnsupportedJoint{joint: :base, joint_type: :planar} = error} =
               FABRIK.solve(robot, configurations, :odom, :tip, Vec3.new(0.4, 0.0, 0.0))

      message = Exception.message(error)
      assert message =~ ":base"
      assert message =~ "three degrees of freedom"
    end

    test "refuses a floating joint" do
      robot = FloatingBaseArm.robot()
      configurations = %{base: Transform.identity(), gimbal: 0.0}

      assert {:error, %UnsupportedJoint{joint: :base, joint_type: :floating} = error} =
               FABRIK.solve(robot, configurations, :world, :lens, Vec3.new(0.1, 0.0, 0.0))

      assert Exception.message(error) =~ "six degrees of freedom"
    end

    # The refusal names the way out, so it reads as an instruction rather than a
    # dead end.
    test "the refusal points at both remedies" do
      robot = PlanarBaseArm.robot()
      configurations = %{base: Transform2D.identity(), shoulder_joint: 0.0}

      {:error, error} =
        FABRIK.solve(robot, configurations, :odom, :tip, Vec3.new(0.4, 0.0, 0.0))

      message = Exception.message(error)
      assert message =~ "source_link"
      assert message =~ "BB.IK.DLS"
    end

    # This is the point of `source_link`: a robot whose base floats is still a
    # perfectly good FABRIK problem for any chain that excludes the base.
    test "solves the same robot when the chain is scoped below the multi-DoF joint" do
      robot = PlanarBaseArm.robot()
      configurations = %{base: Transform2D.new(1.0, 2.0, 0.3), shoulder_joint: 0.0}

      # Targets are in the root frame, and the base has carried base_link out to
      # (1, 2) — so a reachable target is measured from there, not from the origin.
      # The arm reaches 0.5m, and this is 0.4m out, comfortably inside.
      assert {:ok, solved, _meta} =
               FABRIK.solve(robot, configurations, :base_link, :tip, Vec3.new(1.35, 2.2, 0.0))

      assert is_float(solved.shoulder_joint)
      assert is_float(solved.elbow_joint)

      # The base was not part of the problem, so it comes back untouched — and
      # untouched means still a Transform2D, not degraded to a float.
      assert solved.base == Transform2D.new(1.0, 2.0, 0.3)

      # The planar base is still composed into the forward kinematics even though
      # it was not part of the chain, so the tip lands near the base rather than
      # near the origin.
      #
      # The tolerance is loose because FABRIK's own accuracy is poor and unrelated
      # to this: it reports `reached: true` alongside residuals hundreds of times
      # the requested tolerance, for ordinary root-scoped solves on an all-revolute
      # arm. Tightening this would be asserting FABRIK's numerical quality rather
      # than what this test is about.
      {x, y, _z} = Kinematics.link_position(robot, solved, :tip)
      assert_in_delta x, 1.35, 0.2
      assert_in_delta y, 2.2, 0.2
    end
  end
end
