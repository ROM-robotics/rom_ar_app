#!/usr/bin/env python3
"""
Gesture Controller Node for AR Robot Control

Receives hand gesture data from the web AR app via rosbridge
and converts it to /cmd_vel Twist messages.

Gesture Types:
  - FIST: Stop
  - INDEX_POINT: Move forward
  - TWO_FINGERS: Move backward
  - OPEN_HAND: Emergency stop
  - THUMB_LEFT: Turn left
  - THUMB_RIGHT: Turn right
  - PINCH: Variable speed control
  - JOYSTICK: Virtual joystick mode (x, y normalized -1 to 1)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import json
import math


class GestureControllerNode(Node):
    def __init__(self):
        super().__init__('gesture_controller_node')

        # Declare parameters
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('dead_zone', 0.15)
        self.declare_parameter('smoothing', 0.3)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('emergency_stop_enabled', True)

        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.dead_zone = self.get_parameter('dead_zone').value
        self.smoothing = self.get_parameter('smoothing').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.emergency_stop = self.get_parameter('emergency_stop_enabled').value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/ar_controller/status', 10)

        # Subscribers - receive from web app via rosbridge
        self.gesture_sub = self.create_subscription(
            String,
            '/ar_controller/gesture',
            self.gesture_callback,
            10
        )
        self.joystick_sub = self.create_subscription(
            Float32MultiArray,
            '/ar_controller/joystick',
            self.joystick_callback,
            10
        )

        # State
        self.current_twist = Twist()
        self.target_twist = Twist()
        self.is_emergency_stopped = False
        self.last_gesture = "NONE"
        self.connection_alive = False

        # Timer for publishing cmd_vel at fixed rate
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)

        # Watchdog timer - stop if no data received for 1 second
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)
        self.last_data_time = self.get_clock().now()

        self.get_logger().info('🤖 Gesture Controller Node started!')
        self.get_logger().info(f'   Max linear vel: {self.max_linear_vel} m/s')
        self.get_logger().info(f'   Max angular vel: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'   Publish rate: {self.publish_rate} Hz')
        self.publish_status("READY")

    def gesture_callback(self, msg: String):
        """Process gesture commands from the AR app."""
        self.last_data_time = self.get_clock().now()
        self.connection_alive = True

        try:
            data = json.loads(msg.data)
            gesture = data.get('gesture', 'NONE')
            confidence = data.get('confidence', 0.0)
            speed_factor = data.get('speed_factor', 0.5)

            self.last_gesture = gesture

            if confidence < 0.7:
                # Low confidence - gradually stop
                self.target_twist = Twist()
                return

            if gesture == 'EMERGENCY_STOP' or (gesture == 'OPEN_HAND' and self.emergency_stop):
                self.is_emergency_stopped = True
                self.target_twist = Twist()
                self.current_twist = Twist()
                self.cmd_vel_pub.publish(Twist())
                self.publish_status("EMERGENCY_STOP")
                self.get_logger().warn('🛑 Emergency stop activated!')
                return

            if gesture == 'RESUME':
                self.is_emergency_stopped = False
                self.publish_status("RESUMED")
                self.get_logger().info('▶️ Resumed from emergency stop')
                return

            if self.is_emergency_stopped:
                return

            speed = self.max_linear_vel * speed_factor

            if gesture == 'FIST':
                self.target_twist = Twist()
                self.publish_status("STOPPED")

            elif gesture == 'INDEX_POINT':
                self.target_twist = Twist()
                self.target_twist.linear.x = speed
                self.publish_status("FORWARD")

            elif gesture == 'TWO_FINGERS':
                self.target_twist = Twist()
                self.target_twist.linear.x = -speed
                self.publish_status("BACKWARD")

            elif gesture == 'THUMB_LEFT':
                self.target_twist = Twist()
                self.target_twist.angular.z = self.max_angular_vel * speed_factor
                self.publish_status("TURN_LEFT")

            elif gesture == 'THUMB_RIGHT':
                self.target_twist = Twist()
                self.target_twist.angular.z = -self.max_angular_vel * speed_factor
                self.publish_status("TURN_RIGHT")

            elif gesture == 'PINCH':
                # Pinch distance controls speed
                pinch_value = data.get('pinch_distance', 0.0)
                self.target_twist = Twist()
                self.target_twist.linear.x = self.max_linear_vel * pinch_value
                self.publish_status(f"PINCH_SPEED:{pinch_value:.2f}")

        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON received: {msg.data}')

    def joystick_callback(self, msg: Float32MultiArray):
        """Process virtual joystick input from the AR app.
        
        Expected data: [x, y] where x and y are normalized -1.0 to 1.0
        x = left/right (angular), y = forward/backward (linear)
        """
        self.last_data_time = self.get_clock().now()
        self.connection_alive = True

        if self.is_emergency_stopped:
            return

        if len(msg.data) >= 2:
            x = max(-1.0, min(1.0, msg.data[0]))
            y = max(-1.0, min(1.0, msg.data[1]))

            # Apply dead zone
            if abs(x) < self.dead_zone:
                x = 0.0
            if abs(y) < self.dead_zone:
                y = 0.0

            self.target_twist = Twist()
            self.target_twist.linear.x = y * self.max_linear_vel
            self.target_twist.angular.z = -x * self.max_angular_vel

            self.publish_status(f"JOYSTICK:{x:.2f},{y:.2f}")

    def publish_cmd_vel(self):
        """Smoothly interpolate and publish cmd_vel."""
        alpha = 1.0 - self.smoothing

        self.current_twist.linear.x = self._lerp(
            self.current_twist.linear.x, self.target_twist.linear.x, alpha)
        self.current_twist.linear.y = self._lerp(
            self.current_twist.linear.y, self.target_twist.linear.y, alpha)
        self.current_twist.angular.z = self._lerp(
            self.current_twist.angular.z, self.target_twist.angular.z, alpha)

        # Clamp small values to zero
        if abs(self.current_twist.linear.x) < 0.01:
            self.current_twist.linear.x = 0.0
        if abs(self.current_twist.angular.z) < 0.01:
            self.current_twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self.current_twist)

    def watchdog_callback(self):
        """Stop robot if no data received for too long."""
        elapsed = (self.get_clock().now() - self.last_data_time).nanoseconds / 1e9
        if elapsed > 2.0 and self.connection_alive:
            self.get_logger().warn('⚠️ No data from AR app for 2s - stopping robot')
            self.target_twist = Twist()
            self.current_twist = Twist()
            self.cmd_vel_pub.publish(Twist())
            self.connection_alive = False
            self.publish_status("DISCONNECTED")

    def publish_status(self, status: str):
        """Publish controller status for the AR app to display."""
        msg = String()
        msg.data = json.dumps({
            'status': status,
            'gesture': self.last_gesture,
            'emergency_stop': self.is_emergency_stopped,
            'linear_vel': round(self.current_twist.linear.x, 3),
            'angular_vel': round(self.current_twist.angular.z, 3),
        })
        self.status_pub.publish(msg)

    @staticmethod
    def _lerp(current, target, alpha):
        return current + alpha * (target - current)


def main(args=None):
    rclpy.init(args=args)
    node = GestureControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutting down
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
