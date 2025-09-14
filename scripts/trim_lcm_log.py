#!/usr/bin/env python3
"""
Trim LCM log files to remove data after EXPERIMENT_COMPLETE message.
"""

import argparse
import os
import sys
import time
from datetime import datetime

# Add current directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from exp_utils import import_lcm_messages
from config import MOTION_CMD_CHANNEL, MOTION_STATUS_CHANNEL
import lcm

# Import LCM message types
import_lcm_messages()
from exp_utils import motion_command_t


def analyze_log(log_path: str) -> dict:
    """Analyze an LCM log file to find key information."""
    print(f"📊 Analyzing log file: {os.path.basename(log_path)}")
    print(f"📁 File size: {os.path.getsize(log_path) / (1024**3):.2f} GB")
    
    log = lcm.EventLog(log_path, 'r')
    
    first_timestamp = None
    last_timestamp = None
    experiment_complete_time = None
    experiment_complete_event_idx = None
    total_events = 0
    events_after_complete = 0
    motion_commands = []
    
    for event_idx, event in enumerate(log):
        if first_timestamp is None:
            first_timestamp = event.timestamp
        last_timestamp = event.timestamp
        total_events += 1
        
        # Track motion commands
        if event.channel == MOTION_CMD_CHANNEL:
            try:
                msg = motion_command_t.decode(event.data)
                motion_commands.append({
                    'timestamp': event.timestamp,
                    'test_name': msg.test_name,
                    'event_idx': event_idx
                })
                
                if msg.test_name == 'EXPERIMENT_COMPLETE':
                    experiment_complete_time = event.timestamp
                    experiment_complete_event_idx = event_idx
                    print(f"✅ Found EXPERIMENT_COMPLETE at event {event_idx}")
                    relative_time = (event.timestamp - first_timestamp) / 1e6
                    print(f"   Time: {relative_time:.1f} seconds from start")
            except Exception as e:
                print(f"⚠️  Warning: Could not decode motion command at event {event_idx}: {e}")
        
        # Count events after experiment complete
        if experiment_complete_time and event.timestamp > experiment_complete_time:
            events_after_complete += 1
    
    log.close()
    
    # Calculate durations
    total_duration = (last_timestamp - first_timestamp) / 1e6 if first_timestamp and last_timestamp else 0
    
    analysis = {
        'file_path': log_path,
        'file_size_gb': os.path.getsize(log_path) / (1024**3),
        'total_events': total_events,
        'total_duration_hours': total_duration / 3600,
        'first_timestamp': first_timestamp,
        'last_timestamp': last_timestamp,
        'experiment_complete_time': experiment_complete_time,
        'experiment_complete_event_idx': experiment_complete_event_idx,
        'events_after_complete': events_after_complete,
        'motion_commands': motion_commands
    }
    
    # Print summary
    print(f"📊 Analysis Summary:")
    print(f"   Total duration: {total_duration:.1f} seconds ({total_duration/3600:.2f} hours)")
    print(f"   Total events: {total_events:,}")
    print(f"   Motion commands found: {len(motion_commands)}")
    
    if experiment_complete_time:
        complete_duration = (experiment_complete_time - first_timestamp) / 1e6
        extra_duration = (last_timestamp - experiment_complete_time) / 1e6
        print(f"   Duration until EXPERIMENT_COMPLETE: {complete_duration:.1f} seconds")
        print(f"   🚨 Extra recording after complete: {extra_duration:.1f} seconds ({extra_duration/3600:.2f} hours)")
        print(f"   🚨 Events after EXPERIMENT_COMPLETE: {events_after_complete:,}")
        
        # Calculate potential savings
        if events_after_complete > 0:
            estimated_trimmed_size = analysis['file_size_gb'] * (1 - events_after_complete / total_events)
            potential_savings = analysis['file_size_gb'] - estimated_trimmed_size
            print(f"   💾 Potential size after trimming: ~{estimated_trimmed_size:.2f} GB")
            print(f"   💾 Potential savings: ~{potential_savings:.2f} GB ({potential_savings/analysis['file_size_gb']*100:.1f}%)")
    else:
        print(f"   ❌ No EXPERIMENT_COMPLETE message found")
    
    return analysis


def trim_log(log_path: str, output_path: str = None, buffer_seconds: float = 2.0) -> bool:
    """Trim an LCM log file to remove data after EXPERIMENT_COMPLETE + buffer."""
    
    # Analyze the log first
    analysis = analyze_log(log_path)
    
    if analysis['experiment_complete_time'] is None:
        print(f"❌ Cannot trim: No EXPERIMENT_COMPLETE message found")
        return False
    
    if analysis['events_after_complete'] == 0:
        print(f"✅ No trimming needed: No events found after EXPERIMENT_COMPLETE")
        return True
    
    # Generate output path if not provided
    if output_path is None:
        base_path = os.path.splitext(log_path)[0]
        output_path = f"{base_path}_trimmed.lcm"
    
    print(f"\n🔧 Trimming log file...")
    print(f"📂 Input:  {log_path}")
    print(f"📂 Output: {output_path}")
    print(f"⏱️  Buffer: {buffer_seconds} seconds after EXPERIMENT_COMPLETE")
    
    # Calculate cutoff time (EXPERIMENT_COMPLETE + buffer)
    cutoff_time = analysis['experiment_complete_time'] + int(buffer_seconds * 1e6)
    
    # Open input and output logs
    input_log = lcm.EventLog(log_path, 'r')
    output_log = lcm.EventLog(output_path, 'w')
    
    events_written = 0
    events_skipped = 0
    start_time = time.time()
    
    try:
        for event in input_log:
            if event.timestamp <= cutoff_time:
                output_log.write_event(event.timestamp, event.channel, event.data)
                events_written += 1
            else:
                events_skipped += 1
                
            # Progress update every 100k events
            if (events_written + events_skipped) % 100000 == 0:
                progress = (events_written + events_skipped) / analysis['total_events'] * 100
                elapsed = time.time() - start_time
                print(f"   Progress: {progress:.1f}% ({events_written:,} written, {events_skipped:,} skipped) - {elapsed:.1f}s")
    
    except Exception as e:
        print(f"❌ Error during trimming: {e}")
        return False
    finally:
        input_log.close()
        output_log.close()
    
    # Final summary
    elapsed = time.time() - start_time
    output_size = os.path.getsize(output_path) / (1024**3)
    savings = analysis['file_size_gb'] - output_size
    
    print(f"\n✅ Trimming completed in {elapsed:.1f} seconds")
    print(f"📊 Results:")
    print(f"   Events written: {events_written:,}")
    print(f"   Events skipped: {events_skipped:,}")
    print(f"   Original size: {analysis['file_size_gb']:.2f} GB")
    print(f"   Trimmed size:  {output_size:.2f} GB")
    print(f"   Space saved:   {savings:.2f} GB ({savings/analysis['file_size_gb']*100:.1f}%)")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Trim LCM log files after EXPERIMENT_COMPLETE")
    parser.add_argument("log_file", help="Path to the LCM log file to analyze/trim")
    parser.add_argument("--analyze-only", action="store_true", 
                       help="Only analyze the log, don't create trimmed version")
    parser.add_argument("--output", "-o", help="Output path for trimmed log (default: <input>_trimmed.lcm)")
    parser.add_argument("--buffer", "-b", type=float, default=2.0,
                       help="Buffer time in seconds after EXPERIMENT_COMPLETE to keep (default: 2.0)")
    
    args = parser.parse_args()
    
    # Validate input file
    if not os.path.exists(args.log_file):
        print(f"❌ Error: Log file not found: {args.log_file}")
        return 1
    
    try:
        if args.analyze_only:
            # Just analyze
            analyze_log(args.log_file)
        else:
            # Analyze and trim
            success = trim_log(args.log_file, args.output, args.buffer)
            if not success:
                print(f"❌ Trimming failed")
                return 1
        
        print(f"\n✅ Operation completed successfully")
        return 0
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())