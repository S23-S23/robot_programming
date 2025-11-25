#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

##### 완주는 함 

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

        # --- [Pivot Turn 전용 변수] ---
        # 제자리 회전 시 사용할 회전 속도
        self.pivot_speed = 0.6 #default : 0.6
        # 직진 시 기본 속도
        self.forward_speed = 1.25 #default : 1.2

        # 이 범위(픽셀)를 벗어나면 즉시 멈추고 제자리 회전
        # 값을 줄일수록 로봇이 더 자주 멈춰서 정교하게 돕니다.
        self.pivot_threshold = 40 #default : 40

        self.last_angular_z = 0.0

        # --- [터보 모드 변수] ---
        self.turbo_speed = 4.0              # 터보 모드 최고 속도
        self.current_speed = 0.0            # 현재 속도 (부드러운 가감속용)
        self.max_accel = 0.15               # 최대 가속도 (급발진 방지)
        self.max_decel = 0.4                # 최대 감속도 (빠른 감속)

        # 안정성 카운터 (직진 시간 측정)
        self.stable_count = 0               # 안정적 직진 카운트
        self.stable_threshold = 20          # 2초 (30Hz 기준, 60프레임) #30 = 1초
        self.turbo_err_threshold = 20       # 터보 모드 진입 조건 (에러 20px 이내)

        # 곡선 예측 변수
        self.curve_detected = False         # 곡선 감지 플래그

    def listener_callback(self, data):
        try:
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().info(f'Error converting image: {e}')
            return

        current_frame = cv2.flip(current_frame, -1)
        h, w, _ = current_frame.shape

        # --- [시야 설정] ---
        # 하단 ROI: 현재 라인 추종 (하단 30%)
        search_top = int(h * 0.7)
        search_bot = h
        mask_image = current_frame[search_top:search_bot, 0:w]

        gray = cv2.cvtColor(mask_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # --- [상단 ROI: 곡선 예측용] ---
        # 상단 영역을 보고 곡선이 다가오는지 미리 감지
        lookahead_top = int(h * 0.4)
        lookahead_bot = int(h * 0.6)
        lookahead_roi = current_frame[lookahead_top:lookahead_bot, 0:w]

        gray_look = cv2.cvtColor(lookahead_roi, cv2.COLOR_BGR2GRAY)
        _, thresh_look = cv2.threshold(gray_look, 80, 255, cv2.THRESH_BINARY_INV)
        contours_look, _ = cv2.findContours(thresh_look, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 곡선 예측: 상단 ROI에서 라인이 중앙에서 많이 벗어나 있으면 곡선 진입 중
        self.curve_detected = False
        if contours_look:
            largest_look = max(contours_look, key=cv2.contourArea)
            if cv2.contourArea(largest_look) > 100:
                M_look = cv2.moments(largest_look)
                if M_look['m00'] > 0:
                    cx_look = int(M_look['m10'] / M_look['m00'])
                    err_look = abs(cx_look - w / 2)
                    # 상단에서 에러가 40px 이상이면 곡선 감지
                    if err_look > 40:
                        self.curve_detected = True
        
        twist = Twist()
        
        if contours:
            # 가장 큰 라인 덩어리 선택
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 100:
                M = cv2.moments(largest_contour)
                
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    err = cx - w / 2

                    # --- [핵심: Pivot Turn + 터보 모드 로직] ---
                    if abs(err) > self.pivot_threshold:
                        # [모드 1] 위험: 라인이 중앙에서 벗어남 -> 제자리 회전
                        target_speed = 0.0  # 전진 금지 (완전 정지)

                        # 에러 방향에 맞춰 제자리 돌기
                        if err > 0:
                            twist.angular.z = -self.pivot_speed # 오른쪽 회전
                        else:
                            twist.angular.z = self.pivot_speed  # 왼쪽 회전

                        # 다음에 라인을 잃었을 때를 대비해 회전 방향 기억
                        self.last_angular_z = twist.angular.z

                        # 안정성 카운터 리셋
                        self.stable_count = 0

                    else:
                        # [모드 2] 안전: 라인이 중앙에 있음 -> 직진

                        # 안정성 체크: 에러가 매우 작으면 카운터 증가
                        if abs(err) < self.turbo_err_threshold and not self.curve_detected:
                            self.stable_count += 1
                        else:
                            # 곡선 감지되거나 에러 크면 카운터 리셋
                            self.stable_count = 0

                        # 목표 속도 결정
                        if self.curve_detected:
                            # 곡선 예측 시 즉시 일반 속도로
                            target_speed = self.forward_speed
                        elif self.stable_count >= self.stable_threshold:
                            # 2초 이상 안정적 직진 -> 터보 모드
                            target_speed = self.turbo_speed
                        else:
                            # 일반 직진
                            target_speed = self.forward_speed

                        # 직진 중에도 미세한 방향 보정 (P제어)
                        twist.angular.z = -float(err) * 0.005

                    # --- [부드러운 가감속 적용] ---
                    # 급발진/급정지 방지
                    speed_diff = target_speed - self.current_speed
                    if speed_diff > self.max_accel:
                        # 가속 제한
                        self.current_speed += self.max_accel
                    elif speed_diff < -self.max_decel:
                        # 감속 제한
                        self.current_speed -= self.max_decel
                    else:
                        # 목표 속도에 도달
                        self.current_speed = target_speed

                    twist.linear.x = self.current_speed

                else:
                    # 무게중심 계산 불가 -> 제자리에서 찾기
                    self.current_speed = 0.0
                    twist.linear.x = 0.0
                    twist.angular.z = self.last_angular_z
                    self.stable_count = 0
            else:
                # 노이즈 -> 제자리에서 찾기
                self.current_speed = 0.0
                twist.linear.x = 0.0
                twist.angular.z = self.last_angular_z
                self.stable_count = 0
        else:
            # --- [라인 놓침] ---
            # 직진하지 않고 제자리에서 회전하며 라인 탐색
            self.current_speed = 0.0
            twist.linear.x = 0.0
            twist.angular.z = self.last_angular_z
            self.stable_count = 0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()