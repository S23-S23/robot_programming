#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# 완주는 함 

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
        self.pivot_speed = 0.5  
        # 직진 시 속도
        self.forward_speed = 0.9 #edtied 
        
        # 이 범위(픽셀)를 벗어나면 즉시 멈추고 제자리 회전
        # 값을 줄일수록 로봇이 더 자주 멈춰서 정교하게 돕니다.
        self.pivot_threshold = 50 #edited
        
        self.last_angular_z = 0.0

    def listener_callback(self, data):
        try:
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().info(f'Error converting image: {e}')
            return

        current_frame = cv2.flip(current_frame, -1)
        h, w, _ = current_frame.shape
        
        # --- [시야 설정] ---
        # 직각 코너에서 엉뚱한 곳을 보지 않도록 하단 30%만 집중
        search_top = int(h * 0.8)
        search_bot = h
        mask_image = current_frame[search_top:search_bot, 0:w]
        
        gray = cv2.cvtColor(mask_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        twist = Twist()
        
        if contours:
            # 가장 큰 라인 덩어리 선택
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 100:
                M = cv2.moments(largest_contour)
                
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    err = cx - w / 2
                    
                    # --- [핵심: Pivot Turn 로직] ---
                    if abs(err) > self.pivot_threshold:
                        # [모드 1] 위험: 라인이 중앙에서 벗어남 -> 제자리 회전
                        twist.linear.x = 0.0  # 전진 금지 (완전 정지)
                        
                        # 에러 방향에 맞춰 제자리 돌기
                        if err > 0: 
                            twist.angular.z = -self.pivot_speed # 오른쪽 회전
                        else:       
                            twist.angular.z = self.pivot_speed  # 왼쪽 회전
                            
                        # 다음에 라인을 잃었을 때를 대비해 회전 방향 기억
                        self.last_angular_z = twist.angular.z
                        
                    else:
                        # [모드 2] 안전: 라인이 중앙에 있음 -> 직진
                        twist.linear.x = self.forward_speed
                        
                        # 직진 중에도 미세한 방향 보정 (P제어)
                        twist.angular.z = -float(err) * 0.005
                        
                else:
                    # 무게중심 계산 불가 -> 제자리에서 찾기
                    twist.linear.x = 0.0
                    twist.angular.z = self.last_angular_z
            else:
                # 노이즈 -> 제자리에서 찾기
                twist.linear.x = 0.0
                twist.angular.z = self.last_angular_z
        else:
            # --- [라인 놓침] ---
            # 직진하지 않고 제자리에서 회전하며 라인 탐색
            twist.linear.x = 0.0
            twist.angular.z = self.last_angular_z

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()