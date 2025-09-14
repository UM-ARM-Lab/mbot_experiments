import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from threading import Event, Thread
from typing import Optional
from collections import deque
import numpy as np

import lcm
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Button
from typing import Dict, List, Tuple, Optional

from config import *

def now_utime() -> int:
    return int(time.time() * 1e6)

def import_lcm_messages():
    """Import LCM message types and set up Python path for both desktop and robot environments."""
    global motion_command_t, motion_status_t, twist2D_t, vicon_state_t, vicon_twist_t, mbot_analog_t
    
    # Add the necessary paths to Python path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(current_dir)
    
    # Add workspace root to Python path so we can import mbot_lcm_msgs package
    if workspace_root not in sys.path:
        sys.path.insert(0, workspace_root)
    
    _hostname = os.getenv("HOSTNAME")
    if _hostname is None:
        # Fallback to socket.gethostname() if HOSTNAME env var is not set
        import socket
        _hostname = socket.gethostname()
    
    # Add vicon_msgs to Python path (only needed on desktop)
    if _hostname != ROBOT_HOSTNAME:
        vicon_msgs_path = os.path.join(workspace_root, "external", "vicon2lcm", "vicon_msgs")
        if vicon_msgs_path not in sys.path:
            sys.path.insert(0, vicon_msgs_path)
    
    if _hostname != ROBOT_HOSTNAME:
        # Desktop environment - import from generated packages
        from mbot_lcm_msgs.motion_command_t import motion_command_t
        from mbot_lcm_msgs.motion_status_t import motion_status_t
        from mbot_lcm_msgs.twist2D_t import twist2D_t
        from mbot_lcm_msgs.mbot_analog_t import mbot_analog_t
        from vicon_msgs.vicon_state_t import vicon_state_t
        from vicon_msgs.vicon_twist_t import vicon_twist_t
    else: # On robot (onboard)
        # Robot environment - import from local files (no Vicon needed)
        from motion_command_t import motion_command_t
        from motion_status_t import motion_status_t
        from twist2D_t import twist2D_t
        from mbot_analog_t import mbot_analog_t


def send_velocity_cmd(lc, vx: float, wz: float):
    """Body frame vx and wz. vy is always 0."""
    cmd = twist2D_t()
    cmd.utime = now_utime()
    cmd.vx = float(vx)
    cmd.vy = 0.0
    cmd.wz = float(wz)
    lc.publish(VEL_CMD_CHANNEL, cmd.encode())

def send_motion_command(robot_controller, ax: float, az: float, duration: float, test_name: str) -> Tuple[bool, str]:
    # Send motion command (safety validation should be done by caller)
    cmd = motion_command_t()
    cmd.utime = now_utime()
    cmd.ax = float(ax)  # Linear acceleration [m/s²]
    cmd.az = float(az)  # Angular acceleration [rad/s²]
    cmd.duration = float(duration)  # Duration [s]
    cmd.test_name = test_name  # Name/identifier for this motion segment
    robot_controller.lc.publish(MOTION_CMD_CHANNEL, cmd.encode())

    return True, "Motion command sent successfully"

def send_stop_command(robot_controller, test_name: str = "STOP") -> Tuple[bool, str]:
    # Send immediate zero velocity command
    send_velocity_cmd(robot_controller.lc, 0.0, 0.0)
    
    # Send motion command with zero acceleration to maintain zero velocity
    print(f"🛑 Sending stop command")
    return send_motion_command(robot_controller, 0.0, 0.0, 1.0, test_name)

def is_robot_safe_now(robot_x, robot_y, safe_region_bounds):
    return safe_region_bounds['x_min'] < robot_x < safe_region_bounds['x_max'] and \
        safe_region_bounds['y_min'] < robot_y < safe_region_bounds['y_max']

def is_motion_command_safe(robot_controller: 'RobotController', 
                        vx0: float, wz0: float, ax: float, az: float, 
                        duration: float) -> tuple[bool, str, str]:
    """
    Centralized motion command validation used by all experiments.
    
    Args:
        robot_controller: RobotController instance with current pose
        vx0, wz0, ax, az, duration: Motion parameters
        
    Returns:
        (is_safe, reason, violation_type) tuple where violation_type is 'velocity' or 'spatial'
    """
    if not robot_controller.has_pose:
        return False, "No robot pose available", "spatial"
    
    if not is_robot_safe_now(robot_controller.x, robot_controller.y, robot_controller.safe_workspace_bounds):
        return False, "Robot not in safe starting position", "spatial"
    
    # Check velocity limits first (should not trigger reorientation)
    # Skip velocity checks for sysID experiment
    if robot_controller.exp_name != "sysID" and not is_velocity_safe(vx0, wz0, ax, az, duration):
        max_vx = max(vx0, vx0 + ax * duration)
        min_vx = min(vx0, vx0 + ax * duration)
        max_wz = max(wz0, wz0 + az * duration)
        min_wz = min(wz0, wz0 + az * duration)
        return False, f"Motion would exceed velocity limits: ({max_vx:.2f}, {min_vx:.2f}, {max_wz:.2f}, {min_wz:.2f})", "velocity"
    
    # Predict future position and check spatial safety (this should trigger reorientation)
    future_x = robot_controller.x + (vx0 * duration + 0.5 * ax * duration * duration) * math.cos(robot_controller.yaw)
    future_y = robot_controller.y + (vx0 * duration + 0.5 * ax * duration * duration) * math.sin(robot_controller.yaw)

    if not (robot_controller.safe_workspace_bounds['x_min'] < future_x < robot_controller.safe_workspace_bounds['x_max'] and
            robot_controller.safe_workspace_bounds['y_min'] < future_y < robot_controller.safe_workspace_bounds['y_max']):
        return False, f"Motion would exceed safe workspace bounds: ({future_x:.2f}, {future_y:.2f})", "spatial"
    
    return True, "Motion command is safe", "none"

def is_velocity_safe(vx0: float, wz0: float, ax: float, az: float, duration: float) -> bool:
    """
    Quick velocity safety check without full motion validation.
    
    Args:
        vx0, wz0: Current velocities
        ax, az: Accelerations
        duration: Segment duration
        
    Returns:
        True if velocities would remain within limits OR if motion improves the situation
    """
    final_vx = vx0 + ax * duration
    final_wz = wz0 + az * duration
    
    # Check if we're already outside limits - allow recovery motions
    vx_outside_limits = vx0 > MAX_LINEAR_VEL or vx0 < MIN_LINEAR_VEL
    wz_outside_limits = abs(wz0) > MAX_ANGULAR_VEL
    
    # Linear velocity check
    if vx_outside_limits:
        # Allow motion that moves toward valid range
        if vx0 > MAX_LINEAR_VEL:
            linear_vel_safe = final_vx <= vx0  # Must reduce velocity
        elif vx0 < MIN_LINEAR_VEL:
            linear_vel_safe = final_vx >= vx0  # Must increase velocity
        else:
            linear_vel_safe = True
    else:
        # Normal case: both initial and final must be within limits
        linear_vel_safe = (MIN_LINEAR_VEL <= final_vx <= MAX_LINEAR_VEL)
    
    # Angular velocity check
    if wz_outside_limits:
        # Allow motion that reduces angular velocity magnitude
        angular_vel_safe = abs(final_wz) <= abs(wz0)
    else:
        # Normal case: final velocity must be within limits
        angular_vel_safe = abs(final_wz) <= MAX_ANGULAR_VEL
    
    return linear_vel_safe and angular_vel_safe

def transform_vicon_to_centroid(vicon_x: float, vicon_y: float, yaw: float) -> tuple[float, float]:
    """
    Transform Vicon marker position to robot centroid using body-frame offset.
    
    Args:
        vicon_x, vicon_y: Vicon marker position in world frame [m]
        yaw: Robot orientation (yaw angle) [rad]
    
    Returns:
        (centroid_x, centroid_y): Robot centroid position in world frame [m]
    """
    # Transform body-frame offsets to world frame using robot's yaw
    dx_world = ROBOT_BODY_OFFSET_X * math.cos(yaw) - ROBOT_BODY_OFFSET_Y * math.sin(yaw)
    dy_world = ROBOT_BODY_OFFSET_X * math.sin(yaw) + ROBOT_BODY_OFFSET_Y * math.cos(yaw)
    
    # Apply transformation to get centroid position
    centroid_x = vicon_x + dx_world
    centroid_y = vicon_y + dy_world
    
    return centroid_x, centroid_y


class LcmLogger:
    def __init__(self, exp_name: str, log_prefix: str):
        self.exp_name = exp_name
        self.log_prefix = log_prefix
        self.proc: Optional[subprocess.Popen] = None

        self.start() # Start the lcm-logger process

    def start(self) -> str:
        # Use absolute path to ensure logs are saved in project root, not relative to working directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = os.path.dirname(current_dir)
        out_dir = os.path.join(workspace_root, "real_data", self.exp_name)
        os.makedirs(out_dir, exist_ok=True)
        
        # Generate unique filename with milliseconds to avoid collisions
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
        log_path = os.path.join(out_dir, f"{self.log_prefix}_{ts}.lcm")
        
        # Ensure file doesn't already exist
        counter = 0
        base_log_path = log_path
        while os.path.exists(log_path):
            counter += 1
            name, ext = os.path.splitext(base_log_path)
            log_path = f"{name}_{counter:02d}{ext}"
        env = os.environ.copy()
        env["LCM_DEFAULT_URL"] = DEFAULT_LCM_ADDRESS
        self.proc = subprocess.Popen(
            ["lcm-logger", "-f", log_path], env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
        )
        print(f"📹 LCM Logger started: {log_path}")
        print(f"   Logger PID: {self.proc.pid}")
        return log_path

    def stop(self): # Cleanly stop the lcm-logger process
        try:
            if self.proc is not None:
                self.proc.terminate()
        except Exception:
            pass

class RobotController:
    def __init__(self, exp_name: str):
        # Kill any existing lcm-logger processes to prevent data overwrites
        self._cleanup_existing_loggers()
        
        self.address = DEFAULT_LCM_ADDRESS
        self.rate_hz: float = CONTROL_RATE_HZ
        self.lc = lcm.LCM(self.address)
        self.stop_event = Event()
        self._thread: Optional[Thread] = None
        
        # SE(2) mocap state: x, y, yaw and velocities vx, vy, wz
        self.vicon_body = VICON_BODY_NAME
        self.has_pose = False
        
        # Vicon marker position (raw from mocap)
        self.vicon_x = 0.0
        self.vicon_y = 0.0
        
        # Robot centroid position (transformed using body offset)
        self.x = 0.0  # Robot centroid x (used for control and safety)
        self.y = 0.0  # Robot centroid y (used for control and safety)

        self.battery_state = {"voltage": None, "percentage": None, "last_update": 0.0}
        self.position_history = deque(maxlen=200)
        self.current_motion_status = {"type": "IDLE", "info": "Robot is idle", "last_update": 0.0}
        self.motion_segment_info = None
        self.last_motion_update = None

        self.exp_config = self.get_experiment_config(exp_name)
        self.get_new_segment = self.exp_config["get_new_segment"] # Callable that returns a new segment (ax, az, duration, test_name)
        
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._last_yaw: Optional[float] = None
        self._yaw_unwrapped = 0.0
        self._vicon_state_sub = None
        self._vicon_twist_sub = None

        # Don't start logger yet - wait until experiment starts
        self.logger = None
        self.exp_name = exp_name

        self.current_segment = None
        self.segment_counter = 0 # Counts how many segments have been executed successfully

        def _update_pose(self, ch, data):
            """Update SE(2) pose from vicon_state_t message"""
            try:
                msg = vicon_state_t.decode(data)
                if msg.num_bodies <= 0:
                    return
                
                # Find the robot body in Vicon data
                idx = -1
                if self.vicon_body:
                    for i in range(int(msg.num_bodies)):
                        name_i = msg.body_names[i]
                        if self.vicon_body in name_i.lower():
                            idx = i
                            break
                
                # If robot body not found, skip this update
                if idx == -1:
                    print(f"⚠️  Robot body '{self.vicon_body}' not found in Vicon data. Available bodies: {[msg.body_names[i] for i in range(int(msg.num_bodies))]}")
                    return
                
                # Validate that the position and orientation data exist
                if idx >= len(msg.positions) or idx >= len(msg.rpy):
                    print(f"⚠️  Index {idx} out of bounds for positions ({len(msg.positions)}) or rpy ({len(msg.rpy)})")
                    return
                
                # Store raw Vicon marker position
                self.vicon_x = float(msg.positions[idx][0])
                self.vicon_y = float(msg.positions[idx][1])
                yaw_raw = float(msg.rpy[idx][2])  # Extract yaw component
                
                # Simple yaw unwrapping for now (avoid calling self._unwrap_yaw)
                if self._last_yaw is None:
                    self._last_yaw = yaw_raw
                    self.yaw = yaw_raw
                else:
                    dy = ((yaw_raw - self._last_yaw + math.pi) % (2 * math.pi)) - math.pi
                    self.yaw = self.yaw + dy
                    self._last_yaw = yaw_raw
                
                # Transform Vicon position to robot centroid using body offset
                self.x, self.y = transform_vicon_to_centroid(self.vicon_x, self.vicon_y, self.yaw)
                
                self.has_pose = True
                
                # Look for wall markers
                for i in range(int(msg.num_bodies)):
                    name = msg.body_names[i]
                    if name == "top_wall":
                        self.top_wall_pos = {
                            'x': msg.positions[i][0],
                            'y': msg.positions[i][1],
                            'z': msg.positions[i][2],
                            'yaw': msg.rpy[i][2]
                        }
                    elif name == "left_wall":
                        self.left_wall_pos = {
                            'x': msg.positions[i][0],
                            'y': msg.positions[i][1],
                            'z': msg.positions[i][2],
                            'yaw': msg.rpy[i][2]
                        }
                
                # Calculate workspace when both walls are available
                if self.top_wall_pos and self.left_wall_pos:
                    self._calculate_workspace()
                    
            except Exception as e:
                print(f"❌ Error in _update_pose: {e}")
                import traceback
                traceback.print_exc()

        def _update_velocity(self, ch, data):
            msg = vicon_twist_t.decode(data)
            """Update SE(2) body velocities from vicon_twist_t message"""
            try:
                if msg.num_bodies <= 0:
                    return
                idx = 0
                if self.vicon_body:
                    for i in range(int(msg.num_bodies)):
                        name_i = msg.body_names[i]
                        if self.vicon_body in name_i.lower():
                            idx = i
                            break
                # SE(2) body velocities: vx, vy, wz
                self.vx = float(msg.vx[idx])
                self.vy = float(msg.vy[idx])
                self.wz = float(msg.wz[idx])
            except Exception as e:
                print(f"❌ Error in _update_velocity: {e}")

        def _update_battery(self, ch, data):
            """Update battery status from mbot_analog_t message"""
            try:
                msg = mbot_analog_t.decode(data)
                self.battery_state["voltage"] = msg.volts[3]
                self.battery_state["percentage"] = (self.battery_state["voltage"] - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN) * 100.0
                self.battery_state["last_update"] = time.time()
            except Exception as e:
                print(f"❌ Error in _update_battery: {e}")

        def _update_motion_status(self, ch, data):
            """Update motion status from motion_status_t message"""
            try:
                msg = motion_status_t.decode(data)
                self.current_motion_status["type"] = msg.motion_type
                self.current_motion_status["info"] = msg.motion_info
                self.current_motion_status["last_update"] = time.time()
                self.last_motion_update = time.time()
            except Exception as e:
                print(f"❌ Error in _update_motion_status: {e}")

        self._vicon_state_sub = self.lc.subscribe(DEFAULT_POSE_CHANNEL, lambda ch, data: _update_pose(self, ch, data))
        self._vicon_twist_sub = self.lc.subscribe(DEFAULT_VELOCITY_CHANNEL, lambda ch, data: _update_velocity(self, ch, data))
        self._battery_sub = self.lc.subscribe(BATTERY_CHANNEL, lambda ch, data: _update_battery(self, ch, data))
        self._motion_status_sub = self.lc.subscribe(MOTION_STATUS_CHANNEL, lambda ch, data: _update_motion_status(self, ch, data))
        
        # Workspace tracking
        self.workspace_bounds = None
        self.safe_workspace_bounds = None
        self.top_wall_pos = None
        self.left_wall_pos = None

        self.setup_plot()

        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # When start button is pressed, running = True and start experiment loop and start logging
        self.running = False
        self.start_button_pressed = False
        
        # Current segment tracking
        self.waiting_for_segment_completion = False
        self.retry_mode = False  # Track if we're executing a retry after safety interruption

    def _cleanup_existing_loggers(self):
        """Kill any existing lcm-logger processes to prevent data overwrites"""
        import subprocess
        try:
            result = subprocess.run(['pkill', '-f', 'lcm-logger'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print("🧹 Cleaned up existing lcm-logger processes")
            elif result.returncode == 1:
                # No processes found - this is normal
                pass
            else:
                print(f"⚠️  Warning: pkill returned code {result.returncode}")
        except subprocess.TimeoutExpired:
            print("⚠️  Warning: Timeout while cleaning up lcm-logger processes")
        except Exception as e:
            print(f"⚠️  Warning: Error cleaning up lcm-logger processes: {e}")

    def _signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully"""
        try:
            print(f"\n🛑 Received signal {sig}, shutting down gracefully...")
            self.stop()
        except:
            pass  # Ignore errors during shutdown
        finally:
            sys.exit(0)

    def reorient_to_safe_direction(self):
        """
        Reorient robot to face towards the center of the safe workspace.
        
        This function is called when the robot becomes unsafe and needs to be pointed
        back towards safety. It calculates the geometric center of the safe workspace
        bounds and turns the robot to face that direction.
        
        Uses velocity commands only (not motion commands) so the reorientation
        maneuver is not logged to the LCM data - only experimental motion segments
        should be logged.
        
        The reorientation logic:
        1. Calculate center point of safe workspace: (x_center, y_center)
        2. Calculate target yaw from current robot position to center
        3. Turn robot using velocity commands until facing target direction
        4. Stop with velocity command (no logging)
        """
        print("🔄 Starting reorientation to safe direction...")
        
        # Calculate the geometric center of the safe workspace
        safe_x_center = (self.safe_workspace_bounds['x_min'] + self.safe_workspace_bounds['x_max']) / 2
        safe_y_center = (self.safe_workspace_bounds['y_min'] + self.safe_workspace_bounds['y_max']) / 2
        
        # Calculate target yaw angle from robot's current position to workspace center
        target_yaw = math.atan2(safe_y_center - self.y, safe_x_center - self.x)
        
        print(f"   Robot position: ({self.x:.3f}, {self.y:.3f})")
        print(f"   Safe center: ({safe_x_center:.3f}, {safe_y_center:.3f})")
        print(f"   Target yaw: {target_yaw * 180 / math.pi:.1f}°")
        
        # Reorient using velocity commands (fixed loop, not logged)
        reorient_start_time = time.time()
        max_reorient_time = 30.0  # Increased timeout - may need more time if velocity commands are slow
        
        while time.time() - reorient_start_time < max_reorient_time:
            curr_yaw = self.yaw
            yaw_error = target_yaw - curr_yaw
            
            # Normalize yaw error to [-pi, pi]
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            while yaw_error < -math.pi:
                yaw_error += 2 * math.pi
            
            # Stop if close enough (within 5 degrees)
            if abs(yaw_error) < math.pi / 36:  # 5 degrees
                print(f"   ✅ Reorientation target reached (error: {yaw_error * 180 / math.pi:.1f}°)")
                break
                
            # Turn towards target (slow angular velocity)
            angular_vel = 0.5 * math.copysign(1.0, yaw_error)  # 0.5 rad/s max
            # print(f"   Sending angular velocity: {angular_vel:.2f} rad/s")
            send_velocity_cmd(self.lc, 0.0, angular_vel)
            time.sleep(0.1)  # Small control loop delay
            
            # Keep GUI responsive during reorientation
            self.update_plot()
            plt.pause(0.01)
        
        # Stop with velocity command (not motion command to avoid logging)
        send_velocity_cmd(self.lc, 0.0, 0.0)
        print("✅ Reorientation completed")
        time.sleep(1.0)  # Brief pause before next motion

    def move_to_safety(self):
        """
        Move robot forward slowly until it's back in the safe region.
        This function uses velocity commands (not logged) to recover from boundary situations.
        """
        print("🚶 Starting move to safety...")
        
        # Check if we're already safe
        if is_robot_safe_now(self.x, self.y, self.safe_workspace_bounds):
            print("✅ Robot already in safe region")
            return
        
        move_speed = 0.1  # Slow forward movement (10 cm/s)
        
        print(f"   Robot position: ({self.x:.3f}, {self.y:.3f})")
        print(f"   Moving forward at {move_speed} m/s until safe...")
        
        move_start_time = time.time()
        max_move_time = 10.0  # Maximum 10 seconds to reach safety
        
        while (not is_robot_safe_now(self.x, self.y, self.safe_workspace_bounds) and 
            time.time() - move_start_time < max_move_time):
            # Move forward slowly
            send_velocity_cmd(self.lc, move_speed, 0.0)
            time.sleep(0.1)  # 10Hz control rate
            
            # Debug position every 10 steps (1 second)
            if int(time.time() * 10) % 10 == 0:
                print(f"   Position: ({self.x:.3f}, {self.y:.3f})")
            
            # Keep GUI responsive during safety movement
            self.update_plot()
            plt.pause(0.01)
        
        # Stop movement
        send_velocity_cmd(self.lc, 0.0, 0.0)
        
        if is_robot_safe_now(self.x, self.y, self.safe_workspace_bounds):
            print(f"   ✅ Robot reached safe region at ({self.x:.3f}, {self.y:.3f})")
        else:
            print(f"   ⚠️ Could not reach safe region within {max_move_time}s, continuing anyway")
        
        time.sleep(1.0)  # Brief pause after movement

    def _send_next_segment(self, safe: bool):
        """Send next motion segment to robot with safety validation and retry logic"""
        print(f"🔍 _send_next_segment called with safe={safe}, current counter: {self.segment_counter}")
        
        # Safety check: never exceed target segment count
        # We want exactly n_segments (0 to n_segments-1), so stop when counter reaches n_segments
        if self.segment_counter > self.exp_config['n_segments'] - 1:
            print(f"📊 EXPERIMENT COMPLETED: {self.segment_counter} successful segments out of target {self.exp_config['n_segments']}")
            
            # Send final stop command to robot
            success, message = send_stop_command(self, "EXPERIMENT_COMPLETE")
            if success:
                print("🛑 Final stop command sent to robot")
            else:
                print(f"❌ Failed to send final stop: {message}")
            
            # Wait for stop command to complete (1s duration + buffer time)
            print("⏳ Waiting for EXPERIMENT_COMPLETE to finish...")
            time.sleep(1.5)
            print("✅ Robot should now be fully stopped")
            
            self.running = False
            return
        
        # Set retry mode when called with safe=False (after safety interruption)
        if not safe:
            self.retry_mode = True
            print("🔄 Entering retry mode after safety interruption")
        
        max_retries = 20  # Prevent infinite loops
        retry_count = 0
        
        while retry_count < max_retries:
            # Get next segment (only pass 'safe' flag on first attempt)
            next_segment = self.get_new_segment(safe and retry_count == 0)
            if next_segment is None:
                print(f"❌ No more segments available, stopping experiment (reached target: {self.exp_config['n_segments']})")
                print(f"📊 EXPERIMENT COMPLETED: {self.segment_counter} successful segments out of target {self.exp_config['n_segments']}")
                
                # Send final stop command to robot
                success, message = send_stop_command(self, "EXPERIMENT_COMPLETE")
                if success:
                    print("🛑 Final stop command sent to robot")
                else:
                    print(f"❌ Failed to send final stop: {message}")
                
                # Wait for stop command to complete (1s duration + buffer time)
                print("⏳ Waiting for EXPERIMENT_COMPLETE to finish...")
                time.sleep(1.5)
                print("✅ Robot should now be fully stopped")
                
                self.running = False
                return
            
            ax, az, duration, test_name = next_segment
            print(f"📋 Attempt {retry_count + 1}: Checking segment safety: ax={ax}, az={az}, duration={duration}, name='{test_name}'")
            
            # Check if segment is safe using current Vicon velocities
            current_vx = self.vx if hasattr(self, 'vx') else 0.0
            current_wz = self.wz if hasattr(self, 'wz') else 0.0
            is_safe, safety_reason, violation_type = is_motion_command_safe(self, current_vx, current_wz, ax, az, duration)
            if is_safe:
                # Safe segment found, send it
                print(f"✅ Segment is safe, sending motion command...")
                success, message = send_motion_command(self, ax, az, duration, test_name)
                if success:
                    print(f"✅ Motion command sent successfully: {message}")
                    self.waiting_for_segment_completion = True
                else:
                    print(f"❌ Failed to send motion command: {message}")
                return
            else:
                # Handle different types of safety violations
                print(f"⚠️  Segment {retry_count + 1} not safe: {safety_reason}")
                
                if violation_type == "velocity":
                    # Velocity violation: just try a different segment, no reorientation needed
                    print("🔄 Trying different segment (velocity limit issue)...")
                    retry_count += 1
                elif violation_type == "spatial":
                    # Spatial violation: reorient robot to safe direction
                    print("🔄 Reorienting robot to safe direction (spatial safety issue)...")
                    self.reorient_to_safe_direction()
                    
                    # Try to move into safe region if still outside after reorienting
                    if not is_robot_safe_now(self.x, self.y, self.safe_workspace_bounds):
                        print("🚶 Robot still outside safe region after reorientation, attempting to move to safety...")
                        self.move_to_safety()
                    
                    retry_count += 1
                else:
                    # Unknown violation type, default to incrementing retry count
                    retry_count += 1
        
        # Failed to find safe segment after max_retries
        print(f"❌ SAFETY TIMEOUT: Could not find safe segment after {max_retries} attempts, stopping experiment")
        print(f"📊 EXPERIMENT ENDED EARLY: {self.segment_counter}/{self.exp_config['n_segments']} successful segments completed")
        
        # Send final stop command to robot
        success, message = send_stop_command(self, "EXPERIMENT_COMPLETE")
        if success:
            print("🛑 Final stop command sent to robot")
        else:
            print(f"❌ Failed to send final stop: {message}")
        
        self.running = False

    def _on_start_button_click(self, event):
        """Handle start button click"""
        self.start_button_pressed = True

    def _on_emergency_stop_click(self, event):
        """Handle emergency stop button click"""
        print("🛑 EMERGENCY STOP button pressed!")
        print(f"📊 EXPERIMENT MANUALLY STOPPED: {self.segment_counter}/{self.exp_config['n_segments']} successful segments completed")
        
        # Stop the experiment immediately
        self.running = False
        self.start_button_pressed = False
        
        # Send emergency stop motion command to override current segment
        success, message = send_stop_command(self, "EMERGENCY_STOP")
        
        # Reset experiment state
        self.waiting_for_segment_completion = False
        
        print("✅ Emergency stop executed - robot stopped")

    def wait_for_click(self):
        # Wait for start button to be pressed
        while not self.start_button_pressed:
            plt.pause(0.01)
        
        print("🚀 Experiment started! Starting experiment loop...")
        
        # Start logger only when experiment begins
        if self.logger is None:
            self.logger = LcmLogger(self.exp_name, "robot_controller")
            print("⏱️  Waiting for logger to fully initialize...")
            time.sleep(2.0)  # Wait for logger to fully initialize before sending commands
            
        self.running = True
        self.update_loop() # Enter experiment loop
        
        # Clean up resources when experiment completes
        self.stop()

    def _unwrap_yaw(self, yaw: float) -> float:
        """Unwrap yaw angle to prevent 2π jumps"""
        if self._last_yaw is None:
            self._last_yaw = yaw
            self._yaw_unwrapped = yaw
            return self._yaw_unwrapped
        dy = ((yaw - self._last_yaw + math.pi) % (2 * math.pi)) - math.pi # Bounded to [-pi, pi]
        self._yaw_unwrapped += dy
        self._last_yaw = yaw
        return self._yaw_unwrapped

    def start(self):
        def lcm_loop():
            while not self.stop_event.is_set():
                try:
                    self.lc.handle_timeout(50)
                except Exception:
                    pass
        self._thread = Thread(target=lcm_loop, daemon=True)
        self._thread.start()

        def _signal_handler(sig, frame):
            self.stop()
            sys.exit(0)
        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)

    def stop(self):
        self.stop_event.set()
        
        self.running = False
        # Unsubscribe from Vicon channels
        self.lc.unsubscribe(self._vicon_state_sub)
        self.lc.unsubscribe(self._vicon_twist_sub)
        self.lc.unsubscribe(self._battery_sub)
        self.lc.unsubscribe(self._motion_status_sub)

        # Close LCM logger
        if self.logger is not None:
            self.logger.stop()

        # Close plot
        if hasattr(self, 'fig') and self.fig is not None:
            plt.close(self.fig)

    def get_experiment_config(self, exp_name: str):
        # There are two experiments: sysID and full_dataset collection
        # Segments have form (ax, az, duration, segment_name)
        # For data collection purposes, vx0 and wz0 (starting velocities) are obtained when reading logfile from the VICON_TWIST_BODY channel

        if exp_name == "sysID":
            _sampling = False # The segments are hardcoded and executed in order they appear
            _when_fail = "retry" # Retry the same segment
            _reset_between = True # Come to a full stop between segments (done by sending stop motion command and waiting 2 seconds)
        
            _segment_list = [
                # Forward motion tests (for mass estimation)
                (0.1, 0.0, 1.0, "forward_accel_low_from_rest"),
                (0.2, 0.0, 1.0, "forward_accel_med_from_rest"),
                (0.3, 0.0, 1.0, "forward_accel_high_from_rest"),
                
                # Angular motion tests (for inertia estimation)
                (0.0, 0.8, 1.0, "angular_accel_low_from_rest"),
                (0.0, 1.2, 1.0, "angular_accel_med_from_rest"),
                (0.0, 1.6, 1.0, "angular_accel_high_from_rest"),                
            ]
            n_segments = len(_segment_list)

            def get_new_segment(safe):
                if safe:
                    if self.segment_counter >= n_segments:
                        return None # No more segments (experiment is done)
                    current_segment = _segment_list[self.segment_counter] # Get current segment
                    return current_segment
                else:
                    return _segment_list[self.segment_counter] # Return the same segment again


        elif exp_name == "full_dataset":
            _sampling = True # The segments are sampled dynamically
            _when_fail = "skip" # Skip the segment and continue with another one
            _reset_between = False # Don't come to a full stop between segments (once a segment is done, a new starts right away; can start with nonzero velocities)
        
            _duration = 0.5 # All segments have duration 0.5s
            n_segments = 500 # Collect 500 segments in total (per .lcm file, repeated many times for full dataset)

            def generate_random_segment_with_velocity_limits(vx_current, wz_current):
                """
                Generate a random motion segment that respects velocity and acceleration limits.
                Takes into account current velocities to ensure we don't exceed limits.
                
                Args:
                    vx_current: Current linear velocity [m/s]
                    wz_current: Current angular velocity [rad/s]
                """
                # Calculate valid linear acceleration range based on current velocity
                # We need: MIN_LINEAR_VEL <= vx_current + ax * duration <= MAX_LINEAR_VEL
                ax_min_from_vel = (MIN_LINEAR_VEL - vx_current) / _duration
                ax_max_from_vel = (MAX_LINEAR_VEL - vx_current) / _duration
                
                # Apply acceleration limits
                ax_min = max(-MAX_LINEAR_ACCEL, ax_min_from_vel)
                ax_max = min(MAX_LINEAR_ACCEL, ax_max_from_vel)
                
                if vx_current < MIN_LINEAR_VEL * 0.8:  # Below 80% of min velocity
                    # Bias towards positive acceleration to speed up
                    ax_min = max(ax_min, 0.0)  # Only allow zero or positive acceleration
                    
                if vx_current < MIN_LINEAR_VEL:
                    # Force positive acceleration to speed up
                    ax_min = max(ax_min, 0.1)  # Force at least 0.1 m/s² acceleration
                    ax_max = min(ax_max, MAX_LINEAR_ACCEL)  # Allow maximum acceleration
                
                elif vx_current > MAX_LINEAR_VEL * 0.8:  # Above 80% of max velocity
                    # Bias towards negative acceleration to slow down
                    ax_max = min(ax_max, 0.0)  # Only allow zero or negative acceleration
                    
                if vx_current > MAX_LINEAR_VEL:
                    # Force strong negative acceleration to slow down
                    ax_max = min(ax_max, -0.1)  # Force at least -0.1 m/s² deceleration
                    ax_min = max(ax_min, -MAX_LINEAR_ACCEL)  # Allow maximum deceleration
                
                # Calculate valid angular acceleration range based on current velocity
                # We need: -MAX_ANGULAR_VEL <= wz_current + az * duration <= MAX_ANGULAR_VEL
                az_min_from_vel = (-MAX_ANGULAR_VEL - wz_current) / _duration
                az_max_from_vel = (MAX_ANGULAR_VEL - wz_current) / _duration
                
                # Apply acceleration limits
                az_min = max(-MAX_ANGULAR_ACCEL, az_min_from_vel)
                az_max = min(MAX_ANGULAR_ACCEL, az_max_from_vel)
                
                if abs(wz_current) > MAX_ANGULAR_VEL * 0.8:  # Above 80% of max angular velocity
                    # Bias towards opposite direction to slow down
                    if wz_current > 0:  # Rotating right, bias towards left (negative) acceleration
                        az_max = min(az_max, 0.0)
                    else:  # Rotating left, bias towards right (positive) acceleration
                        az_min = max(az_min, 0.0)
                
                if abs(wz_current) > MAX_ANGULAR_VEL:
                    # Force strong counter-acceleration to slow down
                    if wz_current > 0:  # Rotating right, force strong left acceleration
                        az_max = min(az_max, -0.2)  # Force at least -0.2 rad/s² counter-acceleration
                        az_min = max(az_min, -MAX_ANGULAR_ACCEL)  # Allow maximum counter-acceleration
                    else:  # Rotating left, force strong right acceleration
                        az_min = max(az_min, 0.2)  # Force at least +0.2 rad/s² counter-acceleration
                        az_max = min(az_max, MAX_ANGULAR_ACCEL)  # Allow maximum counter-acceleration
                
                # Sample accelerations within valid ranges
                if ax_min <= ax_max:
                    ax = np.random.uniform(ax_min, ax_max)
                else:
                    # No valid linear acceleration range - force recovery acceleration
                    if vx_current > MAX_LINEAR_VEL:
                        ax = -MAX_LINEAR_ACCEL * 0.5  # Strong deceleration
                    elif vx_current < MIN_LINEAR_VEL:
                        ax = MAX_LINEAR_ACCEL * 0.5   # Strong acceleration
                    else:
                        ax = 0.0
                
                if az_min <= az_max:
                    az = np.random.uniform(az_min, az_max)
                else:
                    # No valid angular acceleration range - force recovery acceleration
                    if abs(wz_current) > MAX_ANGULAR_VEL:
                        az = -MAX_ANGULAR_ACCEL * 0.5 * np.sign(wz_current)  # Counter-rotate
                    else:
                        az = 0.0
                
                if abs(ax) < 0.05 and abs(az) < 0.1:
                    name = "constant_velocity"
                elif abs(ax) > 0.3:
                    name = f"high_linear_{'accel' if ax > 0 else 'decel'}"
                elif abs(az) > 1.5:
                    name = f"high_angular_{'right' if az > 0 else 'left'}"
                elif ax > 0.1 and abs(az) > 0.3:
                    name = f"accel_turn_{'right' if az > 0 else 'left'}"
                elif ax < -0.1 and abs(az) > 0.3:
                    name = f"decel_turn_{'right' if az > 0 else 'left'}"
                elif ax > 0.1:
                    name = "forward_accel"
                elif ax < -0.1:
                    name = "backward_decel"
                elif abs(az) > 0.3:
                    name = f"turn_{'right' if az > 0 else 'left'}"
                else:
                    name = "mixed_motion"
                
                return (ax, az, _duration, name)

            def get_new_segment(safe):
                # Always check segment counter, even for retries
                if self.segment_counter >= n_segments:
                    return None # No more segments (experiment is done)
                
                # Get current velocities from robot controller
                vx_current = self.vx if hasattr(self, 'vx') else 0.0
                wz_current = self.wz if hasattr(self, 'wz') else 0.0
                
                # Generate a new random segment that respects all limits
                return generate_random_segment_with_velocity_limits(vx_current, wz_current)
        else:
            raise ValueError(f"Invalid experiment name: {exp_name}")

        return {"segments": None if exp_name == "full_dataset" else _segment_list, "sampling": _sampling, "when_fail": _when_fail, "n_segments": n_segments, "reset_between": _reset_between, "get_new_segment": get_new_segment}

    def _calculate_workspace(self):
        """Calculate workspace boundaries from wall positions considering yaw"""
        wall_width, wall_height = BLUE_WALL_SIZE
        
        # Get wall positions and orientations
        top_wall_x = self.top_wall_pos['x']
        top_wall_y = self.top_wall_pos['y']
        top_wall_yaw = self.top_wall_pos['yaw']
        
        left_wall_x = self.left_wall_pos['x']
        left_wall_y = self.left_wall_pos['y']
        left_wall_yaw = self.left_wall_pos['yaw']
        
        # Calculate the corners of the workspace based on actual wall orientations
        # Top wall: extends backwards from Vicon position
        top_wall_dx = wall_width * math.cos(top_wall_yaw + math.pi)
        top_wall_dy = wall_width * math.sin(top_wall_yaw + math.pi)
        top_wall_start_x = top_wall_x + top_wall_dx
        top_wall_start_y = top_wall_y + top_wall_dy
        
        # Left wall: extends backwards from Vicon position  
        left_wall_dx = wall_width * math.cos(left_wall_yaw + math.pi)
        left_wall_dy = wall_width * math.sin(left_wall_yaw + math.pi)
        left_wall_start_x = left_wall_x + left_wall_dx
        left_wall_start_y = left_wall_y + left_wall_dy
        
        # For rectangular workspace, find the bounding box of both walls
        all_x = [top_wall_x, top_wall_start_x, left_wall_x, left_wall_start_x]
        all_y = [top_wall_y, top_wall_start_y, left_wall_y, left_wall_start_y]
        
        workspace_left = min(all_x)
        workspace_right = max(all_x)
        workspace_bottom = min(all_y)
        workspace_top = max(all_y)
        
        # Calculate workspace dimensions
        actual_width = workspace_right - workspace_left
        actual_height = workspace_top - workspace_bottom
        
        self.workspace_bounds = { # Just used for visualization
            'x_min': workspace_left,
            'x_max': workspace_right, 
            'y_min': workspace_bottom,
            'y_max': workspace_top,
            'width': actual_width,
            'height': actual_height,
            # Store wall positions for visualization
            'top_wall_x': top_wall_x,
            'top_wall_y': top_wall_y,
            'left_wall_x': left_wall_x,
            'left_wall_y': left_wall_y
        }

        self.safe_workspace_bounds = { # Used for safety checks
            'x_min': workspace_left + ROBOT_SAFE_RADIUS,
            'x_max': workspace_right ,
            'y_min': workspace_bottom ,
            'y_max': workspace_top - ROBOT_SAFE_RADIUS,
            'width': actual_width - ROBOT_SAFE_RADIUS,
            'height': actual_height -  ROBOT_SAFE_RADIUS
        }

    def update_loop(self):
        print("🔄 Entering update_loop...")
        while self.running:
            # Used only for plotting trajectory
            if self.has_pose:
                self.position_history.append((self.x, self.y))

            # 1) Robot became unsafe during current segment
            if self.safe_workspace_bounds and not is_robot_safe_now(self.x, self.y, self.safe_workspace_bounds):
                print("⚠️  Robot became unsafe, sending safety stop motion command...")
                
                # Send safety stop motion command to override current segment
                success, message = send_stop_command(self, "SAFETY_STOP")
                
                self.waiting_for_segment_completion = False
                # Wait longer for robot to fully stop and exit motion command mode before reorienting
                print("⏱️  Waiting for robot to fully stop before reorientation...")
                time.sleep(2.0)  # Increased from 0.5s to ensure motion command mode is cleared
                self.reorient_to_safe_direction()
                
                # Try to move into safe region if still outside after reorienting
                if not is_robot_safe_now(self.x, self.y, self.safe_workspace_bounds):
                    print("🚶 Robot still outside safe region, attempting to move to safety...")
                    self.move_to_safety()
                
                self._send_next_segment(safe=False)

            # 2) Completed the past segment 
            elif (self.waiting_for_segment_completion and 
                self.current_motion_status["type"] in ["COMPLETED", "FAILED"]):
                
                if self.current_motion_status["type"] == "COMPLETED":
                    print(f"✅ Segment completed with status: COMPLETED")
                else:
                    print(f"❌ Segment failed with status: {self.current_motion_status['type']}")
                self.waiting_for_segment_completion = False
                
                if self.exp_config["reset_between"]:
                    send_velocity_cmd(self.lc, 0.0, 0.0)  # Reset stop via velocity (not logged)
                    time.sleep(2)
                
                success = self.current_motion_status["type"] == "COMPLETED"
                
                if self.retry_mode:
                    # Completing a retry after safety interruption
                    print("✅ Retry segment completed, exiting retry mode")
                    self.retry_mode = False
                    if success:
                        # After successful retry, increment counter and move to next segment
                        self.segment_counter += 1
                        print(f"📊 RETRY SUCCESS: Segment counter incremented to {self.segment_counter}/{self.exp_config['n_segments']}")
                        self._send_next_segment(safe=True)
                    else:
                        # Failed retry, try same segment again
                        print(f"❌ RETRY FAILED: Counter remains at {self.segment_counter}, trying same segment again")
                        self._send_next_segment(safe=False)
                else:
                    # Normal segment completion (not a retry)
                    if success:
                        self.segment_counter += 1  # Only increment on successful completion
                        print(f"📊 NORMAL SUCCESS: Segment counter incremented to {self.segment_counter}/{self.exp_config['n_segments']}")
                        self._send_next_segment(safe=True)
                    else:
                        # Handle failure based on experiment configuration
                        if self.exp_config["when_fail"] == "skip":
                            print(f"⏭️  SKIP FAILED: Counter remains at {self.segment_counter}, getting new segment")
                            self._send_next_segment(safe=True)  # Get new segment without incrementing counter
                        else:  # "retry" 
                            print(f"🔄 RETRY FAILED: Counter remains at {self.segment_counter}, retrying same segment")
                            self._send_next_segment(safe=False)  # Retry same segment for sysID

            # 3) Ready to start next segment
            elif not self.waiting_for_segment_completion and self.has_pose and self.segment_counter < self.exp_config['n_segments']:
                print("🚀 Ready to start next segment, has_pose=True")
                self._send_next_segment(safe=True)
            elif not self.waiting_for_segment_completion and not self.has_pose:
                print("⏳ Waiting for pose data before starting segment...")

            self.update_plot()
            time.sleep(0.04)  # 25Hz update rate for consistent safety checking

    def setup_plot(self):
        """Setup matplotlib figure and axes"""
        # Force interactive mode for real-time updates
        plt.ion()
        
        # Ensure we're using an interactive backend
        if not plt.isinteractive():
            print("⚠️  Warning: matplotlib not in interactive mode, forcing...")
            plt.ion()
        
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.fig.suptitle("MBot Monitor", fontsize=14, fontweight='bold')
        
        # Make the window non-blocking
        self.fig.canvas.manager.set_window_title("MBot Monitor - Click START to begin experiment")
        
        # Adjust layout to make room for status text on the right side
        plt.subplots_adjust(right=0.75)
        
        # Set up the plot
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True, alpha=0.3)
        # Force equal aspect ratio to prevent distortion
        self.ax.set_aspect('equal', adjustable='box')
        
        # Set initial view limits (will be auto-adjusted when workspace is detected)
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        
        # Create start button - make it more prominent and ensure it's clickable
        ax_start_button = plt.axes([0.02, 0.95, 0.15, 0.05])
        self.start_button = Button(ax_start_button, 'START EXPERIMENT', color='lightgreen', hovercolor='green')
        self.start_button.on_clicked(self._on_start_button_click)
        
        # Create emergency stop button
        ax_stop_button = plt.axes([0.18, 0.95, 0.15, 0.05])
        self.stop_button = Button(ax_stop_button, 'EMERGENCY STOP', color='red', hovercolor='darkred')
        self.stop_button.on_clicked(self._on_emergency_stop_click)
        
        # Initialize plot elements
        self.robot_point, = self.ax.plot([], [], 'ro', markersize=6, label='Robot Centroid', zorder=3)
        self.vicon_point, = self.ax.plot([], [], 'bo', markersize=3, label='Vicon Marker', zorder=2)
        self.robot_arrow = self.ax.arrow(0, 0, 0, 0, head_width=0.1, head_length=0.01, fc='red', ec='red', zorder=1)
        self.trajectory_line, = self.ax.plot([], [], 'g-', linewidth=2, alpha=0.7, label='Trajectory', zorder=1)
        
        # Create legend with zone information (positioned outside plot to avoid overlap with debug info)
        legend_elements = [
            self.robot_point,
            self.vicon_point, 
            self.trajectory_line,
            patches.Patch(color='lightgreen', alpha=0.3, label='Safe Zone'),
        ]
        # Position legend outside the plot area in bottom right (below battery info)
        legend = self.ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 0.45))
        # Ensure legend is drawn outside the plot area
        self.fig.tight_layout()
        
        # Workspace patches (will be added when available)
        self.workspace_patch = None
        self.safety_patch = None
        
        # Status text (robot info) - positioned to the right of plot area
        self.status_text = self.ax.text(1.02, 0.95, '', transform=self.ax.transAxes, 
                                       verticalalignment='top', fontsize=9, 
                                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        # Battery status text (below robot info, to the right)
        self.battery_text = self.ax.text(1.02, 0.65, '', transform=self.ax.transAxes, 
                                        fontsize=9, va='top',
                                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # Draw initial plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Ensure the plot is visible and interactive
        plt.show(block=False)
        print("🎯 Plot window opened! Look for the START EXPERIMENT button in the top-left corner.")

    def update_plot(self):
        """Update the matplotlib plot"""
        try:
            if self.has_pose:
                # Update robot centroid position (red circle)
                self.robot_point.set_data([self.x], [self.y])
                
                # Update Vicon marker position (blue circle)
                self.vicon_point.set_data([self.vicon_x], [self.vicon_y])
                
                # Debug: print both positions (first few times)
                if not hasattr(self, '_robot_debug_count'):
                    self._robot_debug_count = 0
                if self._robot_debug_count < 3:
                    print(f"Vicon marker: ({self.vicon_x:.3f}, {self.vicon_y:.3f})")
                    print(f"Robot centroid: ({self.x:.3f}, {self.y:.3f})")
                    print(f"Offset: ({self.x - self.vicon_x:.3f}, {self.y - self.vicon_y:.3f})")
                    print(f"Yaw: {math.degrees(self.yaw):.1f}°")
                    print("---")
                    self._robot_debug_count += 1
                
                # Update robot orientation arrow from centroid position
                arrow_length = 0.08
                
                # Remove old arrow and create new one
                if hasattr(self, 'robot_arrow'):
                    self.robot_arrow.remove()
                
                self.robot_arrow = self.ax.arrow(self.x, self.y, 
                                                arrow_length * math.cos(self.yaw),
                                                arrow_length * math.sin(self.yaw),
                                                head_width=0.05, head_length=0.05, 
                                                fc='red', ec='red')
                
                # Update trajectory
                if len(self.position_history) > 1:
                    positions = list(self.position_history)
                    x_coords = [pos[0] for pos in positions]
                    y_coords = [pos[1] for pos in positions]
                    self.trajectory_line.set_data(x_coords, y_coords)
                
                # Don't auto-scale if workspace view has been set
                if not hasattr(self, '_view_adjusted') and len(self.position_history) > 0:
                    x_coords = [pos[0] for pos in self.position_history] + [self.x]
                    y_coords = [pos[1] for pos in self.position_history] + [self.y]
                    
                    x_min, x_max = min(x_coords), max(x_coords)
                    y_min, y_max = min(y_coords), max(y_coords)
                    
                    # Add padding
                    x_pad = max(1.0, (x_max - x_min) * 0.2)
                    y_pad = max(1.0, (y_max - y_min) * 0.2)
                    
                    self.ax.set_xlim(x_min - x_pad, x_max + x_pad)
                    self.ax.set_ylim(y_min - y_pad, y_max + y_pad)
            
            # Update workspace visualization
            self.update_workspace_visualization()
            
            # Auto-adjust view when workspace is first detected
            bounds = self.workspace_bounds
            if bounds and not hasattr(self, '_view_adjusted'):
                # Set view to show workspace with minimal padding
                padding = 0.2
                
                x_min = bounds['x_min'] - padding
                x_max = bounds['x_max'] + padding
                y_min = bounds['y_min'] - padding  
                y_max = bounds['y_max'] + padding
                
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
                
                # Mark that view has been adjusted
                self._view_adjusted = True
            
            # Update status text
            self.update_status_text()
            
            # Update battery text with null checks
            voltage = self.battery_state.get('voltage')
            percentage = self.battery_state.get('percentage')
            
            if voltage is not None and percentage is not None:
                battery_text = f"Battery: Voltage: {voltage:.2f}V, Percentage: {percentage:.1f}%"
            else:
                battery_text = "Battery: No data available"
            
            self.battery_text.set_text(battery_text)
            
            # Redraw plot
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            print(f"Error updating plot: {e}")
    
    def update_workspace_visualization(self):
        """Update workspace boundary visualization"""
        bounds = self.workspace_bounds
        if not bounds:
            return
        
        # Only draw workspace elements once
        if hasattr(self, 'workspace_drawn') and self.workspace_drawn:
            return
        
        # Remove old patches if they exist
        if hasattr(self, 'workspace_patch') and self.workspace_patch:
            self.workspace_patch.remove()
        if hasattr(self, 'safety_patch') and self.safety_patch:
            self.safety_patch.remove()
        if hasattr(self, 'danger_zone_patch') and self.danger_zone_patch:
            self.danger_zone_patch.remove()
        if hasattr(self, 'warning_zone_patch') and self.warning_zone_patch:
            self.warning_zone_patch.remove()
        if hasattr(self, 'wall_lines') and self.wall_lines:
            for line in self.wall_lines:
                line.remove()
        
        # Create workspace boundary patch
        workspace_rect = patches.Rectangle(
            (bounds['x_min'], bounds['y_min']), 
            bounds['width'], bounds['height'],
            linewidth=2, edgecolor='blue', facecolor='none', linestyle='--', alpha=0.8
        )
        self.workspace_patch = self.ax.add_patch(workspace_rect)
        
        # Create safe workspace boundary patch (if available)
        if hasattr(self, 'safe_workspace_bounds') and self.safe_workspace_bounds:
            safe_bounds = self.safe_workspace_bounds
            safe_workspace_rect = patches.Rectangle(
                (safe_bounds['x_min'], safe_bounds['y_min']), 
                safe_bounds['width'], safe_bounds['height'],
                linewidth=3, edgecolor=None, facecolor='green', linestyle='-', alpha=0.3
            )
            self.safe_workspace_patch = self.ax.add_patch(safe_workspace_rect)
        
        # Draw wall lines as 2D lines
        self.wall_lines = []
        
        # Top wall - draw as parallelogram based on yaw
        if 'top_wall_x' in bounds and 'top_wall_y' in bounds:
            wall_width, wall_height = BLUE_WALL_SIZE
            top_wall_x = bounds['top_wall_x']
            top_wall_y = bounds['top_wall_y']
            top_wall_yaw = self.top_wall_pos['yaw']
            
            # Calculate wall corners to form a parallelogram
            # Wall extends from Vicon position backwards along its orientation
            dx = wall_width * math.cos(top_wall_yaw + math.pi)  # Backwards direction
            dy = wall_width * math.sin(top_wall_yaw + math.pi)
            
            # Wall thickness perpendicular to orientation
            thickness_dx = wall_height * math.cos(top_wall_yaw + math.pi/2)
            thickness_dy = wall_height * math.sin(top_wall_yaw + math.pi/2)
            
            # Four corners of the wall parallelogram
            corner1_x = top_wall_x + dx  # Back end, bottom
            corner1_y = top_wall_y + dy
            corner2_x = top_wall_x       # Front end, bottom (Vicon position)
            corner2_y = top_wall_y
            corner3_x = top_wall_x       # Front end, top
            corner3_y = top_wall_y + thickness_dy
            corner4_x = top_wall_x + dx  # Back end, top
            corner4_y = top_wall_y + thickness_dy
            
            # Draw wall as filled parallelogram
            wall_corners_x = [corner1_x, corner2_x, corner3_x, corner4_x, corner1_x]
            wall_corners_y = [corner1_y, corner2_y, corner3_y, corner4_y, corner1_y]
            
            top_wall_patch = patches.Polygon(
                list(zip(wall_corners_x, wall_corners_y)),
                facecolor='lightblue', edgecolor='blue', linewidth=2, alpha=0.6
            )
            self.ax.add_patch(top_wall_patch)
            self.wall_lines.append(top_wall_patch)
        
        # Left wall - draw as parallelogram based on yaw
        if 'left_wall_x' in bounds and 'left_wall_y' in bounds:
            wall_width, wall_height = BLUE_WALL_SIZE
            left_wall_x = bounds['left_wall_x']
            left_wall_y = bounds['left_wall_y']
            left_wall_yaw = self.left_wall_pos['yaw']
            
            # Calculate wall corners to form a parallelogram
            # Wall extends from Vicon position backwards along its orientation
            dx = wall_width * math.cos(left_wall_yaw + math.pi)  # Backwards direction
            dy = wall_width * math.sin(left_wall_yaw + math.pi)
            
            # Wall thickness perpendicular to orientation
            thickness_dx = wall_height * math.cos(left_wall_yaw + math.pi/2)
            thickness_dy = wall_height * math.sin(left_wall_yaw + math.pi/2)
            
            # Four corners of the wall parallelogram
            corner1_x = left_wall_x + dx  # Back end, bottom
            corner1_y = left_wall_y + dy
            corner2_x = left_wall_x       # Front end, bottom (Vicon position)
            corner2_y = left_wall_y
            corner3_x = left_wall_x       # Front end, top
            corner3_y = left_wall_y + thickness_dy
            corner4_x = left_wall_x + dx  # Back end, top
            corner4_y = left_wall_y + thickness_dy
            
            # Draw wall as filled parallelogram
            wall_corners_x = [corner1_x, corner2_x, corner3_x, corner4_x, corner1_x]
            wall_corners_y = [corner1_y, corner2_y, corner3_y, corner4_y, corner1_y]
            
            left_wall_patch = patches.Polygon(
                list(zip(wall_corners_x, wall_corners_y)),
                facecolor='lightblue', edgecolor='blue', linewidth=2, alpha=0.6
            )
            self.ax.add_patch(left_wall_patch)
            self.wall_lines.append(left_wall_patch)
        
        # Add wall labels near each wall
        if 'top_wall_x' in bounds and 'top_wall_y' in bounds:
            # Label for top wall (should be at the top)
            top_wall_center_x = (bounds['x_min'] + bounds['x_max']) / 2
            top_wall_y = bounds['top_wall_y']
            self.ax.text(top_wall_center_x, top_wall_y + 0.1, 
                        'TOP WALL', ha='center', va='bottom', fontsize=10, 
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        if 'left_wall_x' in bounds and 'left_wall_y' in bounds:
            # Label for left wall (should be on the left)
            left_wall_x = bounds['left_wall_x']
            left_wall_center_y = (bounds['y_min'] + bounds['y_max']) / 2
            self.ax.text(left_wall_x - 0.1, left_wall_center_y, 
                        'LEFT WALL', ha='right', va='center', fontsize=10, rotation=90,
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # Mark that workspace has been drawn
        self.workspace_drawn = True
    
    def update_status_text(self):
        """Update status text overlay"""
        status_lines = []
        
        # Robot status
        if self.has_pose:
            # Check if position variables are not None before formatting
            if all(v is not None for v in [self.vicon_x, self.vicon_y, self.x, self.y, self.yaw]):
                status_lines.append(f"Vicon: ({self.vicon_x:.3f}, {self.vicon_y:.3f})")
                status_lines.append(f"Centroid: ({self.x:.3f}, {self.y:.3f})")
                status_lines.append(f"   Yaw: {math.degrees(self.yaw):.1f}°")
                
                # Check velocity variables
                if all(v is not None for v in [self.vx, self.vy, self.wz]):
                    status_lines.append(f"   Velocity: ({self.vx:.2f}, {self.vy:.2f}, {self.wz:.2f})")
                else:
                    status_lines.append("   Velocity: No data")
                
                # Show body-frame offset for reference
                offset_world_x = self.x - self.vicon_x
                offset_world_y = self.y - self.vicon_y
                status_lines.append(f"   Offset: ({offset_world_x:.3f}, {offset_world_y:.3f})")
                
                # Safety status using direct check
                bounds = self.workspace_bounds
                if bounds and self.x is not None and self.y is not None:
                    is_safe = is_robot_safe_now(self.x, self.y, bounds)
                    safety_status = "SAFE" if is_safe else "UNSAFE!"
                    status_lines.append(f"   Safety: {safety_status}")
                    if not is_safe:
                        status_lines.append(f"   Reason: Robot outside safe workspace")
                else:
                    status_lines.append("   Safety: No workspace defined")
                
                # Motion segment status with null checks
                if self.current_motion_status:
                    status_lines.append(f"   Motion: {self.current_motion_status}")
                else:
                    status_lines.append("   Motion: No data")
                    
                if self.motion_segment_info:
                    status_lines.append(f"   {self.motion_segment_info}")
                

            else:
                status_lines.append("Robot: Position data incomplete")
        else:
            status_lines.append("Robot: No data")
        
        # Instructions
        status_lines.append("")
        status_lines.append("Press Ctrl+C to stop robot.")
        
        self.status_text.set_text('\n'.join(status_lines))

def read_lcm_experiment_log(log_file_path: str) -> Optional[Dict]:
    """
    Read LCM log file and extract motion commands, vicon data, and status messages.
    
    Returns:
        Dictionary with 'motion_commands', 'motion_status', 'vicon_twist' lists
    """
    if not os.path.exists(log_file_path):
        print(f"❌ Log file not found: {log_file_path}")
        return None
    
    print(f"📖 Reading LCM log: {log_file_path}")
    
    log_data = {
        'motion_commands': [],
        'motion_status': [], 
        'vicon_twist': [],
        'file_path': log_file_path
    }
    try:
        log = lcm.EventLog(log_file_path, 'r')
        
        for event in log:
            if event.channel == MOTION_CMD_CHANNEL:
                msg = motion_command_t.decode(event.data)
                log_data['motion_commands'].append((event.timestamp, msg))
                
            elif event.channel == MOTION_STATUS_CHANNEL:
                msg = motion_status_t.decode(event.data)
                log_data['motion_status'].append((event.timestamp, msg))
                
            elif event.channel == DEFAULT_VELOCITY_CHANNEL:
                msg = vicon_twist_t.decode(event.data)
                log_data['vicon_twist'].append((event.timestamp, msg))
        
        log.close()
        
        print(f"✅ Read {len(log_data['motion_commands'])} motion commands, "
            f"{len(log_data['motion_status'])} status messages, "
            f"{len(log_data['vicon_twist'])} vicon measurements")
        
        return log_data
        
    except Exception as e:
        print(f"❌ Error reading LCM log: {e}")
        return None

def find_latest_log_file(script_dir: str, exp_name: str = "sysID") -> Optional[str]:
    """Find the most recent LCM log file for the specified experiment."""
    import re
    from datetime import datetime
    import glob
    
    log_pattern = os.path.join(script_dir, "..", "real_data", exp_name, "*.lcm")
    log_files = glob.glob(log_pattern)
    
    if not log_files:
        print(f"❌ No log files found matching: {log_pattern}")
        return None
    
    # Parse timestamp from filename: robot_controller_YYYYMMDD_HHMMSS.lcm
    def extract_timestamp(filepath):
        filename = os.path.basename(filepath)
        # Match pattern: robot_controller_20250825_164425.lcm
        match = re.search(r'robot_controller_(\d{8})_(\d{6})\.lcm', filename)
        if match:
            date_str = match.group(1)  # YYYYMMDD
            time_str = match.group(2)  # HHMMSS
            try:
                # Parse as datetime for proper comparison
                dt = datetime.strptime(f"{date_str}_{time_str}", "%Y%m%d_%H%M%S")
                return dt
            except ValueError:
                pass
        # Fallback to modification time if filename parsing fails
        return datetime.fromtimestamp(os.path.getmtime(filepath))
    
    # Sort by timestamp from filename, most recent first
    log_files.sort(key=extract_timestamp, reverse=True)
    latest_file = log_files[0]
    
    print(f"📁 Latest log file: {latest_file}")
    return latest_file

def extract_motion_segments(log_data: Dict) -> List[Dict]:
    """
    Extract motion segments using robot status timestamps for accurate timing.
    Process by successful COMPLETED status messages to handle retries properly.
    """
    from config import VICON_BODY_NAME
    
    motion_commands = log_data['motion_commands']
    motion_status = log_data['motion_status'] 
    vicon_twist = log_data['vicon_twist']
    
    if not motion_commands:
        print("❌ No motion commands found")
        return []
    
    if not motion_status:
        print("❌ No motion status messages found")
        return []
    
    # Convert timestamps to relative seconds from first command
    t0 = motion_commands[0][0] if motion_commands else 0
    
    # Debug: Print summary of commands and statuses
    print(f"🔍 Debug: Found {len(motion_commands)} motion commands, {len(motion_status)} status messages")
    
    # Filter out control commands for matching
    experiment_commands = [(cmd_time, cmd) for cmd_time, cmd in motion_commands 
                          if cmd.test_name not in ['EXPERIMENT_COMPLETE', 'SAFETY_STOP', 'EMERGENCY_STOP']]
    
    print(f"🔍 Debug: {len(experiment_commands)} experiment commands (excluding control commands)")
    
    # Sort all data by timestamp for chronological processing
    motion_status_sorted = sorted(motion_status, key=lambda x: x[0])
    
    segments = []
    valid_segments = 0
    
    # Process by successful COMPLETED status messages
    executing_status = None
    
    for status_time, status_msg in motion_status_sorted:
        if status_msg.motion_type == "EXECUTING":
            # Start of a new execution - remember this
            executing_status = (status_time, status_msg)
            
        elif status_msg.motion_type == "COMPLETED" and executing_status is not None:
            # Found a successful completion - extract this segment
            actual_start_time = executing_status[0]
            actual_end_time = status_time
            actual_duration = (actual_end_time - actual_start_time) / 1e6  # Convert to seconds
            
            # Find the motion command that triggered this execution
            # Look for the command sent closest in time before the execution started
            matching_command = None
            min_time_diff = float('inf')
            
            for cmd_time, cmd in experiment_commands:
                if cmd_time <= actual_start_time:  # Command must be sent before execution
                    time_diff = actual_start_time - cmd_time
                    if time_diff < min_time_diff:
                        min_time_diff = time_diff
                        matching_command = (cmd_time, cmd)
            
            if matching_command is None:
                print(f"   ⚠️ No matching command found for COMPLETED execution at t={(actual_start_time-t0)/1e6:.1f}s")
                executing_status = None
                continue
                
            cmd_time, cmd = matching_command
            segment_id = status_msg.segment_id or cmd.test_name
            
            # Skip control commands even if they somehow got through
            if (cmd.test_name in ['EXPERIMENT_COMPLETE', 'SAFETY_STOP', 'EMERGENCY_STOP'] or 
                segment_id in ['EXPERIMENT_COMPLETE', 'SAFETY_STOP', 'EMERGENCY_STOP']):
                print(f"   ⚠️ Skipping control command execution: '{segment_id}' (cmd: '{cmd.test_name}')")
                executing_status = None
                continue
            
            # Get vicon data for this segment with proper body matching
            segment_vicon = []
            vx0, wz0 = 0.0, 0.0
            first_vicon = True
            
            for twist_time, twist_msg in vicon_twist:
                if actual_start_time <= twist_time <= actual_end_time:
                    if twist_msg.num_bodies > 0:
                        # Find robot body index by name
                        robot_idx = 0  # Default to first body
                        for body_i in range(int(twist_msg.num_bodies)):
                            body_name = twist_msg.body_names[body_i]
                            if VICON_BODY_NAME in body_name.lower():
                                robot_idx = body_i
                                break
                        
                        vx = twist_msg.vx[robot_idx]
                        vy = twist_msg.vy[robot_idx] 
                        wz = twist_msg.wz[robot_idx]
                        rel_time = (twist_time - actual_start_time) / 1e6  # Time relative to segment start
                        segment_vicon.append((rel_time, vx, vy, wz))
                        
                        # Set initial velocities from first measurement
                        if first_vicon:
                            vx0, wz0 = vx, wz
                            first_vicon = False
            
            if len(segment_vicon) < 10:
                print(f"   ⚠️ Execution {valid_segments+1}: '{segment_id}' - Insufficient vicon data ({len(segment_vicon)} points), skipping")
                executing_status = None
                continue
            
            segment = {
                'test_name': segment_id,
                'command': {
                    'ax': cmd.ax,
                    'az': cmd.az,
                    'duration': cmd.duration,
                    'vx0': vx0,
                    'wz0': wz0
                },
                'measured_twists': segment_vicon,
                'status': 'COMPLETED',
                'cmd_time': cmd_time,
                'start_time': actual_start_time,  # Robot execution start
                'end_time': actual_end_time,      # Robot execution end
                'actual_duration': actual_duration
            }
            
            segments.append(segment)
            valid_segments += 1
            
            # Debug output for successful extractions
            cmd_rel_time = (cmd_time - t0) / 1e6
            exec_rel_time = (actual_start_time - t0) / 1e6
            end_rel_time = (actual_end_time - t0) / 1e6
            comm_delay = (actual_start_time - cmd_time) / 1e6
            
            print(f"   ✅ Segment {valid_segments}: '{segment_id}'")
            print(f"     Command:   t={cmd_rel_time:.1f}s")
            print(f"     Execution: t={exec_rel_time:.1f}s - {end_rel_time:.1f}s (dur={actual_duration:.2f}s)")
            print(f"     Delay:     {comm_delay:.3f}s, Expected dur: {cmd.duration:.2f}s")
            print(f"     Vicon:     {len(segment_vicon)} measurements")
            
            # Reset for next execution
            executing_status = None
            
        elif status_msg.motion_type in ["FAILED_SAFETY", "FAILED_EMERGENCY"] and executing_status is not None:
            # Failed execution - reset and continue looking for successful ones
            segment_id = status_msg.segment_id or "unknown"
            actual_start_time = executing_status[0]
            exec_rel_time = (actual_start_time - t0) / 1e6
            print(f"   ❌ Failed execution: '{segment_id}' at t={exec_rel_time:.1f}s ({status_msg.motion_type})")
            executing_status = None
    
    print(f"\n📊 SEGMENT PROCESSING SUMMARY:")
    print(f"   Total motion commands: {len(motion_commands)}")
    print(f"   Experiment commands (non-control): {len(experiment_commands)}")
    print(f"   Valid segments extracted: {valid_segments}")
    
    # Count completed status messages for comparison
    completed_count = sum(1 for _, status_msg in motion_status if status_msg.motion_type == "COMPLETED")
    print(f"   COMPLETED status messages in log: {completed_count}")
    
    if valid_segments != completed_count:
        print(f"⚠️  WARNING: Extracted {valid_segments} segments but found {completed_count} COMPLETED messages")
        print(f"   This indicates some successful executions couldn't be processed (likely due to insufficient Vicon data)")
    else:
        print(f"✅ SUCCESS: Extracted all {valid_segments} successful segment executions")
    
    return segments
