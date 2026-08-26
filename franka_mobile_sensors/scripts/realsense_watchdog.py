#!/usr/bin/env python3

import argparse
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class RealSenseWatchdog(Node):
    def __init__(self, camera_name, camera_namespace, timeout, startup_grace):
        super().__init__(f'{camera_name}_stream_watchdog')
        namespace = camera_namespace.strip('/')
        self._topic = f'/{namespace}/{camera_name}/color/image_raw'
        self._camera_name = camera_name
        self._camera_namespace = f'/{namespace}'
        self._timeout = timeout
        self._started_at = time.monotonic()
        self._last_frame = self._started_at
        self._last_restart = 0.0
        self._launch_pid = os.getppid()
        self.create_subscription(
            Image, self._topic, self._on_frame, qos_profile_sensor_data)
        self.create_timer(1.0, self._check_stream)
        self._startup_grace = startup_grace
        self.get_logger().info(f'Watching {self._topic}')

    def _on_frame(self, _message):
        self._last_frame = time.monotonic()

    def _check_stream(self):
        now = time.monotonic()
        if now - self._started_at < self._startup_grace:
            return
        if now - self._last_frame < self._timeout:
            return
        if now - self._last_restart < self._timeout:
            return

        pid = self._find_camera_process()
        if pid is None:
            self.get_logger().warning(
                f'No frames on {self._topic}; camera process is not running')
        else:
            self.get_logger().error(
                f'No frames on {self._topic} for {self._timeout:.0f}s; '
                f'restarting RealSense process {pid}')
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                self.get_logger().warning(
                    f'RealSense process {pid} exited before it could be restarted')
            except PermissionError as exception:
                self.get_logger().error(
                    f'Cannot restart RealSense process {pid}: {exception}')
        self._last_restart = now

    def _find_camera_process(self):
        expected_node = f'__node:={self._camera_name}'
        expected_namespace = f'__ns:={self._camera_namespace}'
        for entry in os.scandir('/proc'):
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                with open(f'{entry.path}/stat', encoding='ascii') as stat_file:
                    stat = stat_file.read()
                # /proc/<pid>/stat field 2 is a parenthesized command name and
                # may contain spaces or parentheses. Parse after its final ')'.
                command_end = stat.rfind(')')
                if command_end < 0:
                    continue
                fields_after_command = stat[command_end + 2:].split()
                if len(fields_after_command) < 2:
                    continue
                if int(fields_after_command[1]) != self._launch_pid:
                    continue
                with open(f'{entry.path}/cmdline', 'rb') as cmdline_file:
                    command = cmdline_file.read().replace(b'\0', b' ').decode(errors='replace')
            except (OSError, ProcessLookupError, ValueError):
                continue
            if ('realsense2_camera_node' in command
                    and expected_node in command
                    and expected_namespace in command):
                return int(entry.name)
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-name', required=True)
    parser.add_argument('--camera-namespace', required=True)
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--startup-grace', type=float, default=30.0)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = RealSenseWatchdog(
        args.camera_name,
        args.camera_namespace,
        args.timeout,
        args.startup_grace,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
