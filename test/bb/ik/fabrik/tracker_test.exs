# SPDX-FileCopyrightText: 2025 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.IK.FABRIK.TrackerTest do
  use ExUnit.Case, async: true

  alias BB.IK.FABRIK.Tracker
  alias BB.Math.Vec3
  alias BB.Robot.Kinematics
  alias BB.Robot.Runtime

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

          sensor(
            :shoulder_position,
            {BB.Sensor.OpenLoopPositionEstimator, actuator: :shoulder_servo}
          )

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

              sensor(
                :elbow_position,
                {BB.Sensor.OpenLoopPositionEstimator, actuator: :elbow_servo}
              )

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
                 source_link: :base_link,
                 initial_target: Vec3.new(0.35, 0.2, 0.0)
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
                 source_link: :base_link,
                 initial_target: Vec3.new(0.35, 0.2, 0.0),
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
          source_link: :base_link,
          initial_target: Vec3.new(0.35, 0.2, 0.0)
        )

      assert {:ok, _state} = Tracker.update_target(pid, Vec3.new(0.3, 0.3, 0.0))

      status = Tracker.status(pid)
      assert status.target == Vec3.new(0.3, 0.3, 0.0)

      Tracker.stop(pid)
    end
  end

  describe "status/1" do
    test "returns tracking status" do
      start_supervised!(TrackerTestRobot)
      notify_on_tracker_tick()

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          source_link: :base_link,
          initial_target: Vec3.new(0.35, 0.2, 0.0),
          update_rate: 10
        )

      assert_receive :tracker_ticked, 5_000

      status = Tracker.status(pid)

      assert status.tracking == true
      assert status.target == Vec3.new(0.35, 0.2, 0.0)
      assert status.update_rate == 10
      assert is_integer(status.iterations)
      assert status.iterations > 0

      Tracker.stop(pid)
    end
  end

  describe "stop/2" do
    test "stops the tracker and returns final positions" do
      start_supervised!(TrackerTestRobot)
      notify_on_tracker_tick()

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          source_link: :base_link,
          initial_target: Vec3.new(0.35, 0.2, 0.0)
        )

      assert_receive :tracker_ticked, 5_000

      assert {:ok, positions} = Tracker.stop(pid)
      assert is_map(positions)

      # Process should be stopped
      refute Process.alive?(pid)
    end

    test "returns the configuration the last solve arrived at, not the robot's" do
      start_supervised!(TrackerTestRobot)
      notify_on_tracker_tick()

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          source_link: :base_link,
          initial_target: Vec3.new(0.35, 0.2, 0.0)
        )

      assert_receive :tracker_ticked, 5_000

      assert {:ok, solved} = Tracker.stop(pid)

      # The mock actuators discard their commands and publish no motion for the
      # estimators to interpolate, so the robot has not moved and its own
      # configuration is still the one it booted in. Reading `stop/1`'s answer
      # off the robot - as the tracker used to - would therefore hand back these
      # zeroes rather than anything the solver worked out.
      measured = Runtime.configurations(TrackerTestRobot)
      assert Enum.all?(Map.values(measured), &(&1 == 0.0))

      assert solved.shoulder_joint != 0.0
      assert solved.elbow_joint != 0.0

      # And it really is a solution, not just a non-zero pair: the tracked link
      # sits on the target when the robot is in the configuration handed back.
      {x, y, z} = Kinematics.link_position(TrackerTestRobot.robot(), solved, :tip)
      assert Vec3.distance(Vec3.new(x, y, z), Vec3.new(0.35, 0.2, 0.0)) < 0.01
    end

    test "can request hold commands on stop" do
      start_supervised!(TrackerTestRobot)
      notify_on_tracker_tick()

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          source_link: :base_link,
          initial_target: Vec3.new(0.35, 0.2, 0.0)
        )

      assert_receive :tracker_ticked, 5_000

      # Should not raise even with hold: true
      assert {:ok, _positions} = Tracker.stop(pid, hold: true)
    end
  end

  describe "continuous tracking" do
    test "tracks target updates over time" do
      start_supervised!(TrackerTestRobot)
      notify_on_tracker_tick()

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          source_link: :base_link,
          initial_target: Vec3.new(0.35, 0.2, 0.0),
          update_rate: 20
        )

      status1 = Tracker.status(pid)
      assert status1.target == Vec3.new(0.35, 0.2, 0.0)

      Tracker.update_target(pid, Vec3.new(0.25, 0.25, 0.0))

      # Any tick reported before `update_target/2` replied may have solved for the
      # old target; only one started after it is proof of a solve for the new one.
      flush_tracker_ticks()
      assert_receive :tracker_ticked, 5_000

      status2 = Tracker.status(pid)

      # Target should have changed
      assert status2.target == Vec3.new(0.25, 0.25, 0.0)

      # Should still be tracking
      assert status2.tracking == true

      assert {:ok, solved} = Tracker.stop(pid)
      {x, y, z} = Kinematics.link_position(TrackerTestRobot.robot(), solved, :tip)
      assert Vec3.distance(Vec3.new(x, y, z), Vec3.new(0.25, 0.25, 0.0)) < 0.01
    end
  end

  describe "solver options" do
    test "passes options to FABRIK solver" do
      start_supervised!(TrackerTestRobot)
      notify_on_tracker_tick()

      {:ok, pid} =
        Tracker.start_link(
          robot: TrackerTestRobot,
          target_link: :tip,
          source_link: :base_link,
          initial_target: Vec3.new(0.35, 0.2, 0.0),
          max_iterations: 10,
          tolerance: 0.01,
          respect_limits: true
        )

      assert_receive :tracker_ticked, 5_000

      status = Tracker.status(pid)
      # Should have run with limited iterations
      assert status.iterations <= 10

      Tracker.stop(pid)
    end
  end

  # `BB.Loop` emits nothing on its first tick, so the first event arrives at the
  # start of the second - inside the tracker, after the first solve has finished.
  defp notify_on_tracker_tick do
    handler_id = "fabrik-tracker-test-#{System.unique_integer([:positive])}"
    test_pid = self()

    :telemetry.attach(
      handler_id,
      [:bb, :loop, :tick],
      fn
        _event, _measurements, %{robot: TrackerTestRobot, path: [:ik_tracker, :tip]}, _config ->
          send(test_pid, :tracker_ticked)

        _event, _measurements, _metadata, _config ->
          :ok
      end,
      nil
    )

    on_exit(fn -> :telemetry.detach(handler_id) end)
  end

  defp flush_tracker_ticks do
    receive do
      :tracker_ticked -> flush_tracker_ticks()
    after
      0 -> :ok
    end
  end
end
