#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.br = CvBridge()

        # === 속도 설정 ===
        self.turbo_speed = 2.5        # 직선 최고 속도
        self.normal_speed = 1.2       # 일반 주행 속도
        self.curve_speed = 0.6        # 커브 주행 속도 (정지 X, 전진하며 회전)

        # === 회전 설정 ===
        self.base_angular_gain = 0.008  # 기본 회전 게인
        self.curve_angular_gain = 0.012 # 커브에서 더 강한 회전
        self.pivot_speed = 0.8          # 탐색 회전 속도

        # === 임계값 (픽셀) ===
        self.turbo_threshold = 25     # 이 안쪽 = 터보
        self.normal_threshold = 60    # 이 안쪽 = 일반
        self.curve_threshold = 120    # 이 안쪽 = 커브 (전진+회전)
        # 이 바깥 = 라인 이탈 (탐색 모드)

        # === 상태 변수 ===
        self.last_err = 0.0
        self.last_direction = 1       # 1: 오른쪽, -1: 왼쪽
        self.current_speed = 0.0
        self.current_angular = 0.0

        # === Rate Limiter ===
        self.max_accel = 0.2
        self.max_decel = 0.4
        self.max_angular_rate = 0.15

    def find_line_center(self, thresh_img):
        """가장 큰 컨투어의 중심 찾기"""
        contours, _ = cv2.findContours(thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < 80:
            return None

        M = cv2.moments(largest)
        if M['m00'] > 0:
            return int(M['m10'] / M['m00'])
        return None

    def rate_limit(self, current, target, accel, decel):
        """급발진/급정지 방지"""
        diff = target - current
        if diff > accel:
            return current + accel
        elif diff < -decel:
            return current - decel
        return target

    def listener_callback(self, data):
        try:
            frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Image error: {e}')
            return

        frame = cv2.flip(frame, -1)
        h, w, _ = frame.shape
        center = w / 2

        # === ROI 설정 (하단 40%) ===
        roi_top = int(h * 0.6)
        roi = frame[roi_top:h, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

        cx = self.find_line_center(thresh)

        # === 목표 속도/각속도 계산 ===
        if cx is not None:
            err = cx - center
            abs_err = abs(err)

            # 방향 기억
            if err > 10:
                self.last_direction = 1
            elif err < -10:
                self.last_direction = -1

            self.last_err = err

            if abs_err < self.turbo_threshold:
                # [터보] 직선 - 최고 속도, 미세 조향
                target_speed = self.turbo_speed
                target_angular = -err * 0.002

            elif abs_err < self.normal_threshold:
                # [일반] 약간 틀어짐 - 빠른 속도, 적당한 조향
                target_speed = self.normal_speed
                target_angular = -err * self.base_angular_gain

            elif abs_err < self.curve_threshold:
                # [커브] 많이 틀어짐 - 느리지만 전진하며 회전
                target_speed = self.curve_speed
                target_angular = -err * self.curve_angular_gain

            else:
                # [급커브] 심하게 틀어짐 - 최소 속도로 전진 + 강한 회전
                target_speed = 0.3
                target_angular = -self.last_direction * self.pivot_speed
        else:
            # [탐색] 라인 이탈 - 천천히 전진하며 마지막 방향으로 회전
            target_speed = 0.2
            target_angular = -self.last_direction * self.pivot_speed

        # === Rate Limiter 적용 ===
        self.current_speed = self.rate_limit(
            self.current_speed, target_speed, self.max_accel, self.max_decel)

        angular_diff = target_angular - self.current_angular
        if abs(angular_diff) > self.max_angular_rate:
            self.current_angular += self.max_angular_rate * np.sign(angular_diff)
        else:
            self.current_angular = target_angular

        # === 명령 발행 ===
        twist = Twist()
        twist.linear.x = self.current_speed
        twist.angular.z = self.current_angular
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
