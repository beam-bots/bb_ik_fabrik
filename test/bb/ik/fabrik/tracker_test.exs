# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.TrackerTest do
  use ExUnit.Case, async: true

  alias BB.IK.FABRIK.Tracker

  defmodule TrackerTestRobot do
    @moduledoc false
    use BB
    import BB.Unit

    settings do
      name(:tracker_test_robot)
    end

    topology do
      link :base_link do
        joint :shoulder_joint do
          type(:revolute)

          axis do
          end

          limit do
            lower(~u(-180 degree))
            upper(~u(180 degree))
            effort(~u(10 newton_meter))
            velocity(~u(90 degree_per_second))
          end

          actuator(:shoulder_servo, BB.IK.TestRobots.MockActuator)

          link :link1 do
            joint :elbow_joint do
              type(:revolute)

              origin do
                x(~u(0.3 meter))
              end

              axis do
              end

              limit do
                lower(~u(-135 degree))
                upper(~u(135 degree))
                effort(~u(5 newton_meter))
                velocity(~u(90 degree_per_second))
              end

              actuator(:elbow_servo, BB.IK.TestRobots.MockActuator)

              link :link2 do
                joint :tip_joint do
                  type(:fixed)

                  origin do
                    x(~u(0.2 meter))
                  end

                  link(:tip)
                end
              end
            end
          end
        end
      end
    end
  end

  describe "start_link/1" do
    test "starts a tracker process" do
      start_supervised!(TrackerTestRobot)

      assert {:ok, pid} =
               Tracker.start_link(
                 robot: TrackerTestRobot,
                 target_link: :tip,
                 initial_target: {0.35, 0.2, 0.0}
               )

      assert Process.alive?(pid)
      Tracker.stop(pid)
    end

    test "accepts optional name registration" do
      start_supervised!(TrackerTestRobot)

      assert {:ok, _pid} =
               Tracker.start_link(
                 robot: TrackerTestRobot,
                 target_link: :tip,
                 initial_target: {0.35, 0.2, 0.0},
                 name: :test_tracker
               )

      assert Process.whereis(:test_tracker) != nil
      Tracker.stop(:test_tracker)
    end
  end

  describe "update_target/2" do
    test "updates the current target" do
      start_supervised!(TrackerTestRobot)

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          initial_target: {0.35, 0.2, 0.0}
        )

      assert {:ok, _state} = Tracker.update_target(pid, {0.3, 0.3, 0.0})

      status = Tracker.status(pid)
      assert status.target == {0.3, 0.3, 0.0}

      Tracker.stop(pid)
    end
  end

  describe "status/1" do
    test "returns tracking status" do
      start_supervised!(TrackerTestRobot)

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          initial_target: {0.35, 0.2, 0.0},
          update_rate: 10
        )

      # Give it a moment to run at least one solve
      Process.sleep(150)

      status = Tracker.status(pid)

      assert status.tracking == true
      assert status.target == {0.35, 0.2, 0.0}
      assert status.update_rate == 10
      assert is_integer(status.iterations)

      Tracker.stop(pid)
    end
  end

  describe "stop/2" do
    test "stops the tracker and returns final positions" do
      start_supervised!(TrackerTestRobot)

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          initial_target: {0.35, 0.2, 0.0}
        )

      # Give it a moment to solve
      Process.sleep(100)

      assert {:ok, positions} = Tracker.stop(pid)
      assert is_map(positions)

      # Process should be stopped
      refute Process.alive?(pid)
    end

    test "can request hold commands on stop" do
      start_supervised!(TrackerTestRobot)

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          initial_target: {0.35, 0.2, 0.0}
        )

      Process.sleep(100)

      # Should not raise even with hold: true
      assert {:ok, _positions} = Tracker.stop(pid, hold: true)
    end
  end

  describe "continuous tracking" do
    test "tracks target updates over time" do
      start_supervised!(TrackerTestRobot)

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          initial_target: {0.35, 0.2, 0.0},
          update_rate: 20
        )

      # Let it run for a bit
      Process.sleep(100)

      status1 = Tracker.status(pid)
      assert status1.target == {0.35, 0.2, 0.0}

      # Update to a new target
      Tracker.update_target(pid, {0.25, 0.25, 0.0})

      # Let it track the new target
      Process.sleep(100)

      status2 = Tracker.status(pid)

      # Target should have changed
      assert status2.target == {0.25, 0.25, 0.0}

      # Should still be tracking
      assert status2.tracking == true

      Tracker.stop(pid)
    end
  end

  describe "solver options" do
    test "passes options to FABRIK solver" do
      start_supervised!(TrackerTestRobot)

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          initial_target: {0.35, 0.2, 0.0},
          max_iterations: 10,
          tolerance: 0.01,
          respect_limits: true
        )

      # Give it time to run
      Process.sleep(100)

      status = Tracker.status(pid)
      # Should have run with limited iterations
      assert status.iterations <= 10

      Tracker.stop(pid)
    end
  end
end
