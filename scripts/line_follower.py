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

        # === PID 게인 ===
        self.kp = 0.006
        self.ki = 0.0001
        self.kd = 0.012

        # === 상태 변수 ===
        self.prev_error = 0
        self.integral = 0
        self.last_known_direction = 0  # 라인 이탈 시 복구용 (-1: 왼쪽, 1: 오른쪽)
        self.lost_line_count = 0       # 연속으로 라인을 못 찾은 횟수

        # === 속도 설정 ===
        self.max_speed = 1.8
        self.min_speed = 0.4
        self.turn_sensitivity = 0.25

        # === 다중 ROI 가중치 ===
        self.near_weight = 0.7   # 가까운 영역 (즉각 반응)
        self.far_weight = 0.3   # 먼 영역 (커브 예측)

    def find_line_center(self, thresh_img):
        """컨투어 기반 라인 중심 찾기"""
        contours, _ = cv2.findContours(thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # 가장 큰 컨투어 선택
        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < 50:
            return None

        M = cv2.moments(largest)
        if M['m00'] > 0:
            return int(M['m10'] / M['m00'])
        return None

    def listener_callback(self, data):
        try:
            frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        frame = cv2.flip(frame, -1)
        h, w, _ = frame.shape

        # === 다중 ROI 설정 ===
        # Far ROI: 화면 중간 (커브 예측용)
        far_top = int(h * 0.4)
        far_bot = int(h * 0.6)

        # Near ROI: 화면 하단 (즉각 반응용)
        near_top = int(h * 0.7)
        near_bot = h

        # ROI 추출 및 전처리
        far_roi = frame[far_top:far_bot, :]
        near_roi = frame[near_top:near_bot, :]

        far_gray = cv2.cvtColor(far_roi, cv2.COLOR_BGR2GRAY)
        near_gray = cv2.cvtColor(near_roi, cv2.COLOR_BGR2GRAY)

        _, far_thresh = cv2.threshold(far_gray, 80, 255, cv2.THRESH_BINARY_INV)
        _, near_thresh = cv2.threshold(near_gray, 80, 255, cv2.THRESH_BINARY_INV)

        # 라인 중심 찾기
        far_cx = self.find_line_center(far_thresh)
        near_cx = self.find_line_center(near_thresh)

        twist = Twist()
        center = w / 2

        # === 에러 계산 ===
        if near_cx is not None and far_cx is not None:
            # 두 ROI 모두 라인 감지 -> 가중 평균
            combined_cx = near_cx * self.near_weight + far_cx * self.far_weight
            err = combined_cx - center
            self.lost_line_count = 0

        elif near_cx is not None:
            # Near만 감지 -> Near 기준
            err = near_cx - center
            self.lost_line_count = 0

        elif far_cx is not None:
            # Far만 감지 -> 커브 진입 중
            err = far_cx - center
            self.lost_line_count = 0

        else:
            # 라인 완전 이탈
            self.lost_line_count += 1

            if self.lost_line_count < 15:
                # 잠시 이탈: 마지막 방향으로 복구 시도
                err = self.last_known_direction * (w / 4)
            else:
                # 오래 이탈: 더 강하게 회전
                err = self.last_known_direction * (w / 3)

        # === 방향 기억 (복구용) ===
        if near_cx is not None or far_cx is not None:
            if err > 20:
                self.last_known_direction = 1   # 라인이 오른쪽
            elif err < -20:
                self.last_known_direction = -1  # 라인이 왼쪽

        # === PID 제어 ===
        self.integral += err
        self.integral = np.clip(self.integral, -1000, 1000)  # Anti-windup

        derivative = err - self.prev_error
        angular_z = (self.kp * err) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = err

        # === 적응형 속도 제어 ===
        # 커브 정도에 따라 감속
        curve_factor = abs(err) / center  # 0~1 범위
        speed_reduction = curve_factor * self.turn_sensitivity * self.max_speed
        linear_x = max(self.min_speed, self.max_speed - speed_reduction)

        # 라인 이탈 시 감속
        if self.lost_line_count > 0:
            linear_x = min(linear_x, 0.4)

        twist.linear.x = linear_x
        twist.angular.z = -angular_z

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
