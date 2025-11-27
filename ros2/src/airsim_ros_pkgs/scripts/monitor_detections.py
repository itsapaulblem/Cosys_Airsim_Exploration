#!/usr/bin/env python3
"""
YOLOv10 + DeepSORT Detection Monitor
Pretty-print detections in terminal with statistics

Usage:
  docker exec ros2-x11-node bash -c "source /airsim_ros2_ws/install/setup.bash && \\
    python3 /airsim_ros2_ws/src/airsim_ros_pkgs/scripts/monitor_detections.py"
"""

import rclpy
from rclpy.node import Node
from airsim_interfaces.msg import ObjectDetectionArray
from collections import defaultdict
import time
from datetime import datetime

# Terminal colors
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Class color mapping for better visualization
CLASS_COLORS = {
    'person': Colors.OKGREEN,
    'car': Colors.OKBLUE,
    'truck': Colors.OKCYAN,
    'bus': Colors.WARNING,
    'bicycle': Colors.HEADER,
    'motorcycle': Colors.HEADER,
}

class DetectionMonitor(Node):
    def __init__(self):
        super().__init__('detection_monitor')

        # Parameters
        self.declare_parameter('detection_topic', '/detections/front_center')
        self.declare_parameter('show_bbox', True)
        self.declare_parameter('min_confidence', 0.0)  # Filter low confidence
        self.declare_parameter('save_csv', False)

        self.detection_topic = self.get_parameter('detection_topic').value
        self.show_bbox = self.get_parameter('show_bbox').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.save_csv = self.get_parameter('save_csv').value

        # Statistics
        self.total_detections = 0
        self.frame_count = 0
        self.class_counts = defaultdict(int)
        self.track_history = {}  # track_id -> last_seen_time
        self.start_time = time.time()
        self.last_msg_time = None

        # CSV logging
        self.csv_file = None
        if self.save_csv:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_filename = f'/tmp/detections_{timestamp}.csv'
            self.csv_file = open(self.csv_filename, 'w')
            self.csv_file.write('timestamp,camera_id,track_id,class_name,class_id,confidence,x,y,width,height\n')
            self.get_logger().info(f'Saving detections to: {self.csv_filename}')

        # Subscribe to detections
        self.subscription = self.create_subscription(
            ObjectDetectionArray,
            self.detection_topic,
            self.detection_callback,
            10
        )

        self.print_header()
        self.get_logger().info(f'Monitoring detections on: {self.detection_topic}')
        self.get_logger().info(f'Minimum confidence filter: {self.min_confidence}')

    def print_header(self):
        """Print colorful header"""
        print('\n' + '='*80)
        print(f'{Colors.BOLD}{Colors.OKGREEN}YOLOv10 + DeepSORT Detection Monitor{Colors.ENDC}')
        print('='*80 + '\n')

    def detection_callback(self, msg):
        """Process incoming detection messages"""
        current_time = time.time()
        self.last_msg_time = current_time
        self.frame_count += 1

        # Filter by confidence
        filtered_detections = [
            d for d in msg.detections
            if d.confidence >= self.min_confidence
        ]

        if len(filtered_detections) == 0 and len(msg.detections) > 0:
            print(f'{Colors.WARNING}[Frame {self.frame_count}] {len(msg.detections)} detections filtered (below {self.min_confidence} confidence){Colors.ENDC}')
            return

        # Update statistics
        self.total_detections += len(filtered_detections)

        # Print frame header
        elapsed = current_time - self.start_time
        fps = self.frame_count / elapsed if elapsed > 0 else 0.0

        print(f'\n{Colors.BOLD}[Frame {self.frame_count}]{Colors.ENDC} '
              f'{Colors.OKCYAN}Camera: {msg.camera_id}{Colors.ENDC} | '
              f'{Colors.OKGREEN}{len(filtered_detections)} detections{Colors.ENDC} | '
              f'{Colors.HEADER}FPS: {fps:.1f}{Colors.ENDC}')
        print('-' * 80)

        if len(filtered_detections) == 0:
            print(f'{Colors.WARNING}No detections in frame{Colors.ENDC}')
            return

        # Group by class for better readability
        by_class = defaultdict(list)
        for det in filtered_detections:
            by_class[det.class_name].append(det)
            self.class_counts[det.class_name] += 1

            # Update track history
            if det.track_id != -1:
                self.track_history[det.track_id] = current_time

            # Save to CSV
            if self.save_csv:
                self.csv_file.write(
                    f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec},'
                    f'{msg.camera_id},'
                    f'{det.track_id},'
                    f'{det.class_name},'
                    f'{det.class_id},'
                    f'{det.confidence:.3f},'
                    f'{det.x:.1f},{det.y:.1f},{det.width:.1f},{det.height:.1f}\n'
                )

        # Print detections grouped by class
        for class_name in sorted(by_class.keys()):
            detections = by_class[class_name]
            color = CLASS_COLORS.get(class_name, Colors.ENDC)

            print(f'\n  {color}{Colors.BOLD}{class_name.upper()}{Colors.ENDC} ({len(detections)}):')

            for det in detections:
                # Format track ID
                track_str = f'Track {det.track_id:3d}' if det.track_id != -1 else 'No track '

                # Format confidence with color
                conf_color = Colors.OKGREEN if det.confidence > 0.7 else Colors.WARNING if det.confidence > 0.4 else Colors.FAIL
                conf_str = f'{conf_color}{det.confidence:.2f}{Colors.ENDC}'

                # Format bounding box
                bbox_str = ''
                if self.show_bbox:
                    bbox_str = f' @ [{det.x:.0f}, {det.y:.0f}, {det.width:.0f}x{det.height:.0f}]'

                print(f'    [{track_str}] Conf: {conf_str}{bbox_str}')

        # Print running statistics every 10 frames
        if self.frame_count % 10 == 0:
            self.print_statistics()

    def print_statistics(self):
        """Print running statistics"""
        elapsed = time.time() - self.start_time
        active_tracks = len([t for t, last_seen in self.track_history.items()
                            if time.time() - last_seen < 2.0])  # Active in last 2 seconds

        print(f'\n{Colors.BOLD}{Colors.OKCYAN}=== Statistics ==={Colors.ENDC}')
        print(f'  Runtime: {elapsed:.1f}s | '
              f'Frames: {self.frame_count} | '
              f'Total Detections: {self.total_detections} | '
              f'Active Tracks: {active_tracks}')

        if self.class_counts:
            print(f'\n  {Colors.BOLD}Detection Counts by Class:{Colors.ENDC}')
            for class_name, count in sorted(self.class_counts.items(), key=lambda x: x[1], reverse=True):
                color = CLASS_COLORS.get(class_name, Colors.ENDC)
                print(f'    {color}{class_name}{Colors.ENDC}: {count}')

    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'Detections saved to: {self.csv_filename}')

        # Print final statistics
        print(f'\n{Colors.BOLD}{Colors.OKGREEN}=== Final Statistics ==={Colors.ENDC}')
        elapsed = time.time() - self.start_time
        avg_fps = self.frame_count / elapsed if elapsed > 0 else 0.0
        avg_detections = self.total_detections / self.frame_count if self.frame_count > 0 else 0.0

        print(f'  Total Runtime: {elapsed:.1f}s')
        print(f'  Frames Processed: {self.frame_count}')
        print(f'  Average FPS: {avg_fps:.2f}')
        print(f'  Total Detections: {self.total_detections}')
        print(f'  Average Detections/Frame: {avg_detections:.2f}')
        print(f'  Unique Tracks: {len(self.track_history)}')

        if self.class_counts:
            print(f'\n  {Colors.BOLD}Final Class Counts:{Colors.ENDC}')
            for class_name, count in sorted(self.class_counts.items(), key=lambda x: x[1], reverse=True):
                color = CLASS_COLORS.get(class_name, Colors.ENDC)
                pct = (count / self.total_detections) * 100 if self.total_detections > 0 else 0
                print(f'    {color}{class_name}{Colors.ENDC}: {count} ({pct:.1f}%)')

        print('\n' + '='*80 + '\n')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = DetectionMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n{Colors.WARNING}Interrupted by user{Colors.ENDC}')
    except Exception as e:
        print(f'{Colors.FAIL}Error: {e}{Colors.ENDC}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
