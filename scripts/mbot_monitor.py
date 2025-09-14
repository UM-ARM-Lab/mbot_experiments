#!/usr/bin/env python3
"""
🤖 MBot Monitor
Real-time 2D visualization of robot position, orientation, and workspace boundaries.
Shows top-down view with Vicon tracking and wall boundaries using matplotlib.
"""

import argparse
import logging
import os
import sys
from datetime import datetime

from config import *
from exp_utils import RobotController, import_lcm_messages

import_lcm_messages()

class TeeLogger:
    """Class to capture all print statements and log them to both console and file."""

    def __init__(self, log_file_path):
        self.log_file_path = log_file_path
        self.original_stdout = sys.stdout

        # Set up logging to write to both file and console
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s - %(levelname)s - %(message)s",
            handlers=[
                logging.FileHandler(log_file_path, mode="w", encoding="utf-8"),
                logging.StreamHandler(sys.stdout),
            ],
            force=True,  # Override any existing logging configuration
        )

        # Create a custom stdout that writes to both console and file
        self.tee_stdout = self._create_tee_stdout()

    def _create_tee_stdout(self):
        """Create a custom stdout that writes to both console and file."""

        class TeeStdout:
            def __init__(self, original_stdout, log_file_path):
                self.original_stdout = original_stdout
                self.log_file_path = log_file_path
                self.log_file = open(log_file_path, "a", encoding="utf-8")

            def write(self, text):
                # Write to console
                self.original_stdout.write(text)
                # Write to file
                self.log_file.write(text)
                self.log_file.flush()  # Ensure immediate writing

            def flush(self):
                self.original_stdout.flush()
                self.log_file.flush()

            def __getattr__(self, attr):
                # Delegate other attributes to the console stream
                return getattr(self.original_stdout, attr)

            def close(self):
                self.log_file.close()

        return TeeStdout(sys.stdout, self.log_file_path)

    def start(self):
        """Start capturing all output."""
        sys.stdout = self.tee_stdout
        logging.info("=" * 80)
        logging.info("🤖 MBot Monitor Started")
        logging.info(f"Log file: {self.log_file_path}")
        logging.info(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logging.info("=" * 80)

    def stop(self):
        """Stop capturing and close the log file."""
        logging.info("=" * 80)
        logging.info(f"🤖 MBot Monitor Stopped")
        logging.info(f"End time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logging.info("=" * 80)

        sys.stdout = self.original_stdout
        self.tee_stdout.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


def setup_logging(exp_name):
    """Set up logging with a timestamped log file."""
    # Create logs directory if it doesn't exist
    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "log")
    os.makedirs(logs_dir, exist_ok=True)

    # Create log filename with timestamp and experiment name
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"mbot_monitor_{exp_name}_{timestamp}.txt"
    log_file_path = os.path.join(logs_dir, log_filename)

    return log_file_path


def main():
    parser = argparse.ArgumentParser(
        description="🤖 MBot Monitor - Real-time 2D Visualization"
    )
    parser.add_argument("--exp_name", type=str, default="sysID", help="Experiment name")

    args = parser.parse_args()

    # Set up logging
    log_file_path = setup_logging(args.exp_name)

    # Use the TeeLogger context manager to capture all output
    with TeeLogger(log_file_path):
        print(f"🤖 Starting MBot Monitor for experiment: {args.exp_name}")
        print(f"📝 All output will be logged to: {log_file_path}")

        rc = RobotController(exp_name=args.exp_name)

        # Start the LCM message processing thread
        rc.start()

        rc.wait_for_click()  # Wait for start experiment button to be pressed


if __name__ == "__main__":
    sys.exit(main())
