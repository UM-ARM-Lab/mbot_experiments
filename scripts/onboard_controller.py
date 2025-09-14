#!/usr/bin/env python3
"""
Robot-side Motion Controller for MBot Experiments

This controller runs on the robot and listens for MOTION_COMMAND messages from the desktop.
It executes motion segments using the same API as execute_motion_segment and reports status back.

"""

import threading
import time

import lcm
from config import *
from exp_utils import import_lcm_messages, send_velocity_cmd

# Import LCM messages before class definition (needed for type annotations)
import_lcm_messages()

import os

# Import message types for local use (needed in class methods)
import socket
import sys

_hostname = socket.gethostname()
if _hostname == "arm-mbot":
    from motion_command_t import motion_command_t
    from motion_status_t import motion_status_t
    from vicon_twist_t import vicon_twist_t
else:
    from mbot_lcm_msgs.motion_command_t import motion_command_t
    from mbot_lcm_msgs.motion_status_t import motion_status_t

    # Desktop needs to import from vicon_msgs
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(current_dir)
    vicon_msgs_path = os.path.join(
        workspace_root, "external", "vicon2lcm", "vicon_msgs"
    )
    if vicon_msgs_path not in sys.path:
        sys.path.insert(0, vicon_msgs_path)
    from vicon_msgs.vicon_twist_t import vicon_twist_t


class OnboardController:
    """
    Onboard controller that executes commands from desktop
    """

    def __init__(self):
        """
        Initialize the onboard controller
        """
        self.control_period = 1.0 / CONTROL_RATE_HZ

        # LCM setup
        print(f"🌐 Connecting to LCM at: {DEFAULT_LCM_ADDRESS}")
        self.lc = lcm.LCM(DEFAULT_LCM_ADDRESS)
        print(f"✅ LCM connection established")

        # Motion state
        self.current_command = None
        self.executing_command = None  # Command currently being executed
        self.is_executing = False
        self.stop_current_motion = False  # Flag to immediately stop current motion
        self.execution_thread = None  # Thread for motion execution

        # Velocity state from Vicon
        self.vicon_body = VICON_BODY_NAME
        self.has_velocity = False
        self.vx = 0.0  # Body-frame linear velocity [m/s]
        self.vy = (
            0.0  # Body-frame lateral velocity [m/s] (unused for differential drive)
        )
        self.wz = 0.0  # Body-frame angular velocity [rad/s]
        self.velocity_last_update = 0.0  # Timestamp of last velocity update

        # Status publishing
        self.status_channel = MOTION_STATUS_CHANNEL

        # Subscribe to channels
        print(f"📡 Subscribing to channel: {MOTION_CMD_CHANNEL}")
        self.lc.subscribe(MOTION_CMD_CHANNEL, self._motion_command_handler)
        print(f"✅ Subscribed to {MOTION_CMD_CHANNEL}")

        print(f"📡 Subscribing to channel: {DEFAULT_VELOCITY_CHANNEL}")
        self.lc.subscribe(DEFAULT_VELOCITY_CHANNEL, self._vicon_twist_handler)
        print(f"✅ Subscribed to {DEFAULT_VELOCITY_CHANNEL}")

        # Start command listener thread
        self.running = True
        self.command_thread = threading.Thread(
            target=self._command_listener_loop, daemon=True
        )
        self.command_thread.start()

        # Start LCM message handling thread
        self.lcm_thread = threading.Thread(target=self._lcm_handler_loop, daemon=True)
        self.lcm_thread.start()

        print(f"✓ Robot motion controller initialized")
        print(f"  Control rate: {CONTROL_RATE_HZ} Hz")
        print(f"  Listening on: {MOTION_CMD_CHANNEL}")
        print(f"  Velocity from: {DEFAULT_VELOCITY_CHANNEL} (body: {self.vicon_body})")
        print(f"  Status channel: {self.status_channel}")

        # Validate Vicon configuration
        if not self.vicon_body or self.vicon_body.strip() == "":
            print(
                f"⚠️  WARNING: No Vicon body name configured - all motions will use zero initial velocities"
            )

        # Publish initial status
        self.publish_status("IDLE", "Robot motion controller ready")

    def _is_stop_command(self, test_name: str) -> bool:
        """Check if a command is an emergency/stop command that needs special handling"""
        test_name_upper = test_name.upper()
        return (
            "STOP" in test_name_upper
            or "EMERGENCY" in test_name_upper
            or "SAFETY" in test_name_upper
            or test_name_upper == "EXPERIMENT_COMPLETE"
        )

    def _get_initial_velocities(self, is_stop_command: bool) -> tuple[float, float]:
        """Get appropriate initial velocities based on command type and velocity data freshness"""
        if is_stop_command:
            print(
                f"🛑 Stop command detected, forcing zero initial velocities: vx0=0.000, wz0=0.000"
            )
            return 0.0, 0.0

        if not self.has_velocity:
            print(
                f"🏃 Using initial velocities: vx0=0.000, wz0=0.000 (no velocity data)"
            )
            return 0.0, 0.0

        # Check velocity data freshness
        current_time = time.time()
        velocity_age = current_time - self.velocity_last_update
        max_velocity_age = 1.0  # Maximum age in seconds before considering data stale

        if velocity_age > max_velocity_age:
            print(
                f"🏃 Using initial velocities: vx0=0.000, wz0=0.000 (velocity data stale: {velocity_age:.1f}s)"
            )
            return 0.0, 0.0
        else:
            print(f"🏃 Using initial velocities: vx0={self.vx:.3f}, wz0={self.wz:.3f}")
            return self.vx, self.wz

    def _motion_command_handler(self, channel, data):
        """Handle incoming motion commands"""
        print(f"📡 RAW LCM message received on channel: {channel}")
        try:
            command = motion_command_t.decode(data)

            print(f"📥 Successfully decoded motion command: {command.test_name}")
            print(f"  ax={command.ax:.3f}, az={command.az:.3f}")
            print(f"  duration={command.duration:.3f}s")

            # Store command for execution (will override any current command)
            self.current_command = command
            print(f"✅ Command stored for execution")

        except Exception as e:
            print(f"❌ ERROR: Failed to decode motion command: {e}")
            import traceback

            traceback.print_exc()

    def _vicon_twist_handler(self, channel, data):
        """Handle incoming vicon twist messages for velocity estimation"""
        try:
            msg = vicon_twist_t.decode(data)

            # Check if we have any bodies in the message
            if msg.num_bodies <= 0:
                return

            # Find the robot body in the message
            robot_idx = -1
            if self.vicon_body:
                for i in range(int(msg.num_bodies)):
                    name_i = msg.body_names[i]
                    if self.vicon_body.lower() in name_i.lower():
                        robot_idx = i
                        break

            # If robot body not found, show warning only once
            if robot_idx == -1:
                if not hasattr(self, "_velocity_warning_shown"):
                    print(
                        f"⚠️  Robot body '{self.vicon_body}' not found in Vicon velocity data"
                    )
                    print(
                        f"    Available bodies: {[msg.body_names[i] for i in range(int(msg.num_bodies))]}"
                    )
                    self._velocity_warning_shown = True
                return

            # Extract SE(2) body velocities: vx (forward), vy (lateral), wz (yaw rate)
            self.vx = float(msg.vx[robot_idx])
            self.vy = float(msg.vy[robot_idx])
            self.wz = float(msg.wz[robot_idx])
            self.velocity_last_update = time.time()
            self.has_velocity = True

        except Exception as e:
            print(f"❌ ERROR: Failed to decode vicon twist message: {e}")
            import traceback

            traceback.print_exc()

    def _command_listener_loop(self):
        """Main loop for listening to commands and executing them"""
        print("🔄 Command listener loop started")
        while self.running:
            try:
                # Execute any pending command
                if self.current_command:
                    # Check if this is an emergency command that can interrupt current motion
                    is_emergency = self._is_stop_command(self.current_command.test_name)

                    if self.is_executing and is_emergency:
                        # Only interrupt for emergency commands
                        print(
                            f"🔄 Interrupting current motion for emergency command: {self.current_command.test_name}"
                        )
                        self.stop_current_motion = True
                        # Don't wait - trust the interrupt mechanism and proceed immediately
                        print("⚡ Stop signal sent, proceeding without timeout wait")
                    elif self.is_executing:
                        # Normal command - wait silently for current to finish
                        time.sleep(self.control_period)
                        continue

                    # If not executing OR emergency was handled, start execution
                    print(f"🚀 Starting execution: {self.current_command.test_name}")
                    self.executing_command = self.current_command  # Store for execution
                    self.current_command = (
                        None  # Clear immediately so new commands can arrive
                    )
                    self.stop_current_motion = (
                        False  # Clear stop flag before starting new execution
                    )

                    # Start execution in separate thread to keep command listener responsive
                    self.execution_thread = threading.Thread(
                        target=self._execute_command,
                        args=(self.executing_command,),
                        daemon=True,
                    )
                    self.execution_thread.start()

                time.sleep(
                    self.control_period
                )  # Sleep at control rate to allow LCM thread to process messages

            except Exception as e:
                print(f"❌ ERROR in command listener: {e}")
                import traceback

                traceback.print_exc()
                break
        print("🛑 Command listener loop ended")

    def _lcm_handler_loop(self):
        """Main loop for handling LCM messages"""
        print("📡 LCM handler loop started")
        while self.running:
            try:
                # Handle LCM messages with timeout
                self.lc.handle_timeout(50)  # 50ms timeout
            except Exception as e:
                print(f"❌ ERROR in LCM handler: {e}")
                import traceback

                traceback.print_exc()
                break
        print("🛑 LCM handler loop ended")

    def _execute_command(self, command):
        """Execute a motion command using execute_motion_segment API"""
        print(f"🚀 Executing motion: {command.test_name}")
        self.is_executing = True

        # Publish status with segment ID
        self.publish_status(
            "EXECUTING", f"Executing {command.test_name}", command.test_name
        )

        # Determine initial velocities for motion execution
        is_stop_command = self._is_stop_command(command.test_name)
        vx0, wz0 = self._get_initial_velocities(is_stop_command)

        success = self._execute_motion_segment(
            vx0=vx0,  # Use validated velocity from Vicon or zero if stale/missing
            wz0=wz0,  # Use validated velocity from Vicon or zero if stale/missing
            ax=command.ax,
            az=command.az,
            duration=command.duration,
        )

        # Add conditional stop for commands that need full stops after completion
        if success and (is_stop_command or ("from_rest" in command.test_name.lower() and "sysid" in command.test_name.lower())):
            # Stop for: explicit stop commands OR sysID "from_rest" tests that require starting from rest
            send_velocity_cmd(self.lc, 0.0, 0.0)
            print(f"    🛑 Stopped robot (stop command or sysID from_rest test)")

        if success:
            print(f"✅ Motion completed: {command.test_name}")
            self.publish_status(
                "COMPLETED",
                f"{command.test_name} completed successfully",
                command.test_name,
            )
        else:
            # Motion was interrupted - check if it was a safety/emergency command or regular segment
            if "SAFETY_STOP" in command.test_name:
                print(f"❌ Safety stop failed: {command.test_name}")
                self.publish_status(
                    "FAILED_SAFETY",
                    f"Safety violation during {command.test_name}",
                    command.test_name,
                )
            elif "EMERGENCY_STOP" in command.test_name:
                print(f"❌ Emergency stop failed: {command.test_name}")
                self.publish_status(
                    "FAILED_EMERGENCY",
                    f"Emergency stop: {command.test_name}",
                    command.test_name,
                )
            else:
                # Regular segment was interrupted - don't send status to avoid double-counting
                print(
                    f"⚠️ Motion interrupted: {command.test_name} (no status sent to avoid double-counting)"
                )

        self.is_executing = False
        self.executing_command = None  # Clear the executing command

    def _execute_motion_segment(
        self, vx0: float, wz0: float, ax: float, az: float, duration: float
    ) -> bool:
        """
        Execute a motion segment with the same API as RobotController.execute_motion_segment

        Args:
            vx0: Initial linear velocity [m/s]
            wz0: Initial angular velocity [rad/s]
            ax: Linear acceleration [m/s²]
            az: Angular acceleration [rad/s²]
            duration: Duration [s]

        Returns:
            True if successful, False if interrupted
        """
        print(
            f"    Executing motion: vx0={vx0:.3f}, wz0={wz0:.3f}, ax={ax:.3f}, az={az:.3f}, dur={duration:.3f}s"
        )

        # Use time-based loop with better timing compensation
        start_time = time.time()
        elapsed = 0.0
        iteration = 0

        while elapsed < duration:
            # Check for interrupt signal from command listener
            if self.stop_current_motion:
                print(
                    f"    🛑 Stop signal received, interrupting current motion at t={elapsed:.2f}s"
                )
                # Send immediate stop command before returning
                send_velocity_cmd(self.lc, 0.0, 0.0)
                return False

            # Calculate current velocities at this time (use actual elapsed time)
            if abs(ax) < 1e-6 and abs(az) < 1e-6:
                # Constant velocity (hold)
                current_vx = vx0
                current_wz = wz0
            else:
                # Constant acceleration (ramp from initial to final velocities)
                current_vx = vx0 + ax * elapsed
                current_wz = wz0 + az * elapsed

            # Send velocity command
            send_velocity_cmd(self.lc, current_vx, current_wz)

            # Adaptive timing: calculate next target time and sleep precisely
            iteration += 1
            target_time = start_time + iteration * self.control_period
            current_time = time.time()

            # Check if next iteration would exceed duration
            next_elapsed = target_time - start_time
            if next_elapsed > duration:
                # Final partial iteration - sleep only until duration is reached
                final_target = start_time + duration
                sleep_duration = final_target - current_time
            else:
                # Normal iteration
                sleep_duration = target_time - current_time

            if sleep_duration > 0:
                time.sleep(sleep_duration)
            # If sleep_duration <= 0, we're running behind - skip the sleep

            elapsed = time.time() - start_time

        # Note: Not sending stop command to allow continuous motion between segments
        # Stop commands are handled separately in _execute_command based on experiment requirements
        print(
            f"    ✓ Motion segment completed successfully after {iteration} iterations"
        )
        return True

    def publish_status(self, status: str, info: str = "", segment_id: str = ""):
        """Publish motion status to desktop with segment tracking"""
        try:
            msg = motion_status_t()
            msg.utime = int(time.time() * 1e6)
            msg.motion_type = status
            msg.motion_info = info
            msg.segment_id = segment_id  # Track which segment this status relates to
            self.lc.publish(self.status_channel, msg.encode())
        except Exception as e:
            print(f"ERROR publishing status: {e}")

    def shutdown(self):
        """Shutdown the controller cleanly"""
        print("🔄 Shutting down robot motion controller...")
        self.running = False

        # Stop any current motion
        self.stop_current_motion = True

        # Join execution thread
        if (
            hasattr(self, "execution_thread")
            and self.execution_thread
            and self.execution_thread.is_alive()
        ):
            print("⏳ Waiting for motion execution to stop...")
            self.execution_thread.join(timeout=2.0)

        # Join command thread
        if hasattr(self, "command_thread"):
            self.command_thread.join(timeout=2.0)

        print("✅ Robot motion controller shut down")


def main():
    """Main entry point"""
    print("🤖 MBot Robot Motion Controller")
    print("=" * 40)

    # Create and run controller
    controller = OnboardController()

    try:
        print("✅ Robot motion controller running...")
        print(f"💡 Listening for {MOTION_CMD_CHANNEL} messages from desktop")
        print("⚠️  Press Ctrl+C to shutdown")
        print()

        # Keep running until interrupted
        while controller.running:
            pass

    except KeyboardInterrupt:
        print("\n🔄 Shutdown requested by user (Ctrl+C)")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    main()
