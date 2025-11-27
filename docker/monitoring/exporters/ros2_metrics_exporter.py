#!/usr/bin/env python3
"""
ROS2 Metrics Exporter for Prometheus
Monitors ROS2 nodes, topics, and message rates in the AirSim ecosystem
Exposes metrics on port 9200
"""

import rclpy
from rclpy.node import Node
from prometheus_client import start_http_server, Gauge, Counter, Histogram
import time
import psutil
import os
from collections import defaultdict

# Prometheus metrics
ros2_node_count = Gauge('ros2_node_count', 'Total number of active ROS2 nodes')
ros2_topic_count = Gauge('ros2_topic_count', 'Total number of active ROS2 topics')

ros2_topic_rate = Gauge('ros2_topic_rate_hz', 'Topic publish rate in Hz', ['topic_name'])
ros2_topic_message_count = Counter('ros2_topic_messages_total', 'Total messages published to topic', ['topic_name'])

ros2_node_cpu_percent = Gauge('ros2_node_cpu_percent', 'CPU usage percentage for ROS2 node', ['node_name'])
ros2_node_memory_mb = Gauge('ros2_node_memory_mb', 'Memory usage in MB for ROS2 node', ['node_name'])

ros2_tf_tree_health = Gauge('ros2_tf_tree_health', 'TF tree health status (1=healthy, 0=broken)')
ros2_connection_health = Gauge('ros2_connection_health', 'ROS2 DDS connection health (1=healthy, 0=degraded)')

# Message latency tracking
ros2_message_latency = Histogram('ros2_message_latency_seconds', 'Message processing latency', ['topic_name'])


class ROS2MetricsExporter(Node):
    """
    ROS2 node that collects and exposes Prometheus metrics
    """

    def __init__(self):
        super().__init__('ros2_metrics_exporter')
        self.get_logger().info('ROS2 Metrics Exporter starting...')

        # Topic rate tracking
        self.topic_message_counts = defaultdict(int)
        self.topic_last_counts = defaultdict(int)
        self.last_rate_calc_time = time.time()

        # Create timer for periodic metrics collection (every 2 seconds)
        self.create_timer(2.0, self.collect_metrics)

        self.get_logger().info('ROS2 Metrics Exporter ready on port 9200')

    def collect_metrics(self):
        """Collect all ROS2 metrics and update Prometheus gauges"""
        try:
            # Get list of all nodes
            node_names = self.get_node_names()
            ros2_node_count.set(len(node_names))

            # Get list of all topics
            topic_names_and_types = self.get_topic_names_and_types()
            ros2_topic_count.set(len(topic_names_and_types))

            # Calculate topic rates
            self._calculate_topic_rates()

            # Monitor specific drone nodes
            self._monitor_drone_nodes(node_names)

            # Check TF tree health (simplified check)
            self._check_tf_health(topic_names_and_types)

            # Check overall connection health
            self._check_connection_health()

        except Exception as e:
            self.get_logger().error(f'Error collecting metrics: {e}')

    def _calculate_topic_rates(self):
        """Calculate and publish topic rates"""
        current_time = time.time()
        time_delta = current_time - self.last_rate_calc_time

        if time_delta >= 1.0:  # Calculate rate every second
            for topic, current_count in self.topic_message_counts.items():
                last_count = self.topic_last_counts.get(topic, 0)
                rate = (current_count - last_count) / time_delta
                ros2_topic_rate.labels(topic_name=topic).set(rate)
                self.topic_last_counts[topic] = current_count

            self.last_rate_calc_time = current_time

    def _monitor_drone_nodes(self, node_names):
        """Monitor drone-specific nodes"""
        drone_nodes = [n for n in node_names if 'Drone' in n or 'drone' in n or 'localization' in n]

        for node_name in drone_nodes:
            # Try to get process info (simplified - in real deployment would need better process tracking)
            try:
                # This is a simplified placeholder - actual implementation would need process ID mapping
                # For now, we'll set placeholder values
                ros2_node_cpu_percent.labels(node_name=node_name).set(0.0)
                ros2_node_memory_mb.labels(node_name=node_name).set(0.0)
            except Exception as e:
                self.get_logger().debug(f'Could not get metrics for {node_name}: {e}')

    def _check_tf_health(self, topic_names_and_types):
        """Check if TF tree is being published"""
        tf_topics = [t[0] for t in topic_names_and_types if '/tf' in t[0]]
        ros2_tf_tree_health.set(1 if len(tf_topics) > 0 else 0)

    def _check_connection_health(self):
        """Check overall ROS2 DDS connection health"""
        # Simplified health check - checks if we can get node list
        try:
            nodes = self.get_node_names()
            ros2_connection_health.set(1 if len(nodes) > 0 else 0)
        except:
            ros2_connection_health.set(0)


def main():
    """Main entry point"""
    print("Starting ROS2 Metrics Exporter for Prometheus...")

    # Start Prometheus HTTP server on port 9200
    start_http_server(9200)
    print("Prometheus metrics server started on port 9200")

    # Initialize ROS2
    rclpy.init()

    try:
        # Create and spin the metrics exporter node
        exporter = ROS2MetricsExporter()
        rclpy.spin(exporter)
    except KeyboardInterrupt:
        print("Shutting down ROS2 Metrics Exporter...")
    finally:
        exporter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
