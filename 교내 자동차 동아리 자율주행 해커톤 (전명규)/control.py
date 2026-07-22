#!/usr/bin/env python3
import rospy
import numpy as np
import math
import tf
# FSDS 메시지 타입 임포트
from fs_msgs.msg import Track, Cone, ControlCommand
# ROS 표준 메시지 타입 임포트
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, TwistWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
# --- 유틸리티 함수 ---
def normalize_angle(angle_rad):
    """
    라디안 각도를 [-pi, pi] 구간으로 정규화합니다.
    """
    while angle_rad > np.pi:
        angle_rad -= 2.0 * np.pi
    while angle_rad < -np.pi:
        angle_rad += 2.0 * np.pi
    return angle_rad
def cone_color(cone_type):
    """
    콘 색상 타입(Cone.BLUE, Cone.YELLOW 등)에 따라 ColorRGBA를 반환합니다.
    RViz 시각화에 사용됩니다.
    """
    if cone_type == Cone.BLUE:
        return ColorRGBA(0.0, 0.0, 1.0, 1.0) # 파란색
    elif cone_type == Cone.YELLOW:
        return ColorRGBA(1.0, 1.0, 0.0, 1.0) # 노란색
    elif cone_type == Cone.ORANGE_BIG or cone_type == Cone.ORANGE_SMALL:
        return ColorRGBA(1.0, 0.5, 0.0, 1.0) # 주황색
    else:
        return ColorRGBA(1.0, 1.0, 1.0, 1.0) # 기본 (흰색)
class ControlNode:
    """
    메인 제어 노드:
      - 트랙, 주행 거리계(Odometry), 속도 정보를 구독합니다.
      - 파란색/노란색 콘을 기반으로 트랙의 중간선(midline)을 생성합니다.
      - Stanley 횡방향(lateral) 제어기를 실행하여 조향 각을 계산합니다.
      - ControlCommand 및 시각화 토픽을 발행합니다.
    """
    def __init__(self):
        # 콘 위치 저장용 컨테이너
        self.blue_cones = []    # 파란색 콘 리스트
        self.yellow_cones = []  # 노란색 콘 리스트
        self.mid_points = []    # 중간선 포인트 리스트 (파란색/노란색 콘 쌍의 평균 위치)
        # 시각화를 위한 마커 배열
        self.cone_markers = MarkerArray()
        # 차량 상태: [x, y, yaw(deg), speed(m/s)]
        self.state = [0.0, 0.0, 0.0, 0.0]
        # TF (Transform) 브로드캐스터: 차량의 포즈를 ROS 시스템에 알림
        self.tf_broadcaster = tf.TransformBroadcaster()
        # --- 구독자 (Subscribers) 설정 ---
        rospy.Subscriber("/fsds/testing_only/track", Track, self.track_callback)
        rospy.Subscriber("/fsds/testing_only/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/fsds/gss", TwistWithCovarianceStamped, self.speed_callback)
        # --- 발행자 (Publishers) 설정 ---
        self.cmd_pub = rospy.Publisher("/fsds/control_command", ControlCommand, queue_size=10)
        self.midline_path_pub = rospy.Publisher("/midpoint_path", Path, queue_size=10)
        self.cones_pub = rospy.Publisher("/cones_markers", MarkerArray, queue_size=10)
    def track_callback(self, msg):
        """
        Track 메시지를 수신하고, 콘을 색상별로 분류하며, 중간선을 업데이트합니다.
        """
        # RViz 시각화를 위해 마커 업데이트
        self.cone_markers = self.track_to_markers(msg)
        # 콘을 파란색/노란색으로 분류
        self.blue_cones = []
        self.yellow_cones = []
        for cone in msg.track:
            if cone.color == Cone.BLUE:
                self.blue_cones.append(cone)
            elif cone.color == Cone.YELLOW:
                self.yellow_cones.append(cone)
        # 트랙의 중간선 재계산
        self.calculate_midpoints()
    def track_to_markers(self, track_msg, frame_id="fsds/map"):
        """
        트랙 정보(Track message)를 RViz 시각화를 위한 MarkerArray로 변환합니다.
        """
        markers = MarkerArray()
        for idx, cone in enumerate(track_msg.track):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = rospy.Time(0)
            m.ns = "cones"
            m.id = idx
            m.type = Marker.SPHERE # 구형 마커 사용
            m.action = Marker.ADD
            m.pose.position = cone.location
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3 # 마커 크기 설정
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color = cone_color(cone.color) # 콘 색상 설정
            m.lifetime = rospy.Duration(0.5) # 마커 지속 시간
            markers.markers.append(m)
        return markers
    def calculate_midpoints(self):
        """
        페어링된 파란색 콘과 노란색 콘의 위치를 평균하여 중간선(mid_points) 리스트를 생성합니다.
        두 콘 리스트의 최소 길이만큼만 페어링하여 사용합니다.
        """
        self.mid_points = []
        # zip을 사용하여 길이가 짧은 쪽을 기준으로 콘을 페어링
        for blue_cone, yellow_cone in zip(self.blue_cones, self.yellow_cones):
            mid = Point()
            # x, y, z 위치의 평균을 계산
            mid.x = (blue_cone.location.x + yellow_cone.location.x) / 2.0
            mid.y = (blue_cone.location.y + yellow_cone.location.y) / 2.0
            mid.z = (blue_cone.location.z + yellow_cone.location.z) / 2.0
            self.mid_points.append(mid)
    def publish_midpoints(self):
        """
        계산된 중간선(midline)을 Path 메시지로 발행하여 시각화합니다.
        """
        path_msg = Path()
        path_msg.header.frame_id = "fsds/map"
        path_msg.header.stamp = rospy.Time.now()
        # 각 중간점(Point)을 PoseStamped로 변환하여 Path 메시지에 추가
        for pt in self.mid_points:
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position = pt
            pose.pose.orientation.w = 1.0 # 회전은 고려하지 않으므로 w=1.0 (단위 쿼터니언)
            path_msg.poses.append(pose)
        self.midline_path_pub.publish(path_msg)
    def odom_callback(self, msg):
        """
        주행 거리계(Odometry)로부터 차량의 위치(x, y)와 방향(yaw)을 업데이트하고 TF를 브로드캐스트합니다.
        """
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w
        # 쿼터니언을 오일러 각으로 변환하고 yaw (z축 회전)를 추출
        _, _, yaw = tf.transformations.euler_from_quaternion([ori_x, ori_y, ori_z, ori_w])
        self.state[0] = pos_x
        self.state[1] = pos_y
        self.state[2] = np.degrees(yaw) # yaw를 도(degree)로 변환하여 저장
        # 차량 포즈를 TF로 브로드캐스트 ("fsds/FSCar" 프레임)
        self.tf_broadcaster.sendTransform(
            (pos_x, pos_y, pos_z),
            (ori_x, ori_y, ori_z, ori_w),
            rospy.Time.now(),
            "fsds/FSCar", # 자율 주행 차량의 프레임 이름
            "fsds/map"   # 기준 프레임
        )
    def speed_callback(self, msg):
        """
        TwistWithCovarianceStamped 메시지로부터 스칼라 속도(선속도의 크기)를 업데이트합니다.
        """
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        # 속도의 크기(magnitude) 계산 (v = sqrt(vx^2 + vy^2))
        self.state[3] = np.hypot(vel_x, vel_y)
    def run(self):
        """
        주기적 제어 루프:
          - Stanley 제어기를 사용하여 제어 명령을 계산합니다.
          - ControlCommand를 발행합니다.
          - 중간선 및 콘 마커 시각화 토픽을 발행합니다.
        """
        # Stanley 제어기 실행
        throttle, steering, brake = self.stanley_control(
            k=0.85,           # cross-track error gain (횡단 오차 이득)
            wheelbase=1.55,  # 차량 축간 거리 [m]
            target_speed=10.0 # 목표 속도 [m/s]
        )
        # 제어 명령 메시지 생성 및 발행
        cmd = ControlCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.throttle = throttle
        cmd.steering = steering
        cmd.brake = brake
        self.cmd_pub.publish(cmd)
        # 시각화 토픽 발행
        self.publish_midpoints()
        self.cones_pub.publish(self.cone_markers)
    def stanley_control(self, k=0.5, wheelbase=1.55, target_speed=5.0):
        """
        Stanley 조향 제어 법칙 구현 (횡방향 제어).
        Args:
            k (float): cross-track error gain (횡단 오차 이득).
            wheelbase (float): 차량 축간 거리 [m].
            target_speed (float): 원하는 공칭(nominal) 속도 [m/s].
        Returns:
            (throttle, steering, brake): [0, 1] 범위의 명령 값.
        """
        # 경로(midline)가 최소 2개 이상의 포인트로 준비되지 않았다면, 브레이크 명령
        if len(self.mid_points) < 2:
            rospy.logwarn("Midline not ready, applying full brake.")
            return 0.0, 0.0, 1.0
        # --- 1. 차량 상태 읽기 ---
        cog_x, cog_y, yaw_deg, v = self.state
        yaw_rad = np.radians(yaw_deg) # yaw 각을 라디안으로 변환
        max_steer_rad = np.radians(30.0) # 최대 조향각 (30도)
        # --- 2. 무게 중심(CoG) 위치를 앞 차축(Front Axle) 위치로 변환 ---
        front_offset = wheelbase / 2.0 # CoG에서 앞 차축까지의 거리 (단순 모델 가정)
        front_x = cog_x + front_offset * np.cos(yaw_rad)
        front_y = cog_y + front_offset * np.sin(yaw_rad)
        # --- 3. 앞 차축 위치에 가장 가까운 경로 점 찾기 (최근접점 탐색) ---
        closest_dist = float("inf")
        target_idx = 0
        for i, pt in enumerate(self.mid_points):
            dx = pt.x - front_x
            dy = pt.y - front_y
            dist = np.hypot(dx, dy) # 유클리드 거리
            if dist < closest_dist:
                closest_dist = dist
                target_idx = i
        target_pt = self.mid_points[target_idx]
        # --- 4. 경로 헤딩(Path Heading) 계산 ---
        # 최근접점과 그 이웃점을 사용하여 경로의 접선 방향(yaw)을 계산
        if target_idx >= len(self.mid_points) - 1:
            # 경로의 끝에 도달하면, 이전 점을 사용 (후진 방향으로 계산)
            next_idx = target_idx - 1
            path_yaw = math.atan2(
                target_pt.y - self.mid_points[next_idx].y,
                target_pt.x - self.mid_points[next_idx].x
            )
        else:
            # 일반적인 경우, 다음 점을 사용 (전진 방향으로 계산)
            next_idx = target_idx + 1
            path_yaw = math.atan2(
                self.mid_points[next_idx].y - target_pt.y,
                self.mid_points[next_idx].x - target_pt.x
            )
        # 경로 헤딩 오차 (Heading Error, psi_e) 계산: 경로 yaw - 차량 yaw
        heading_error = normalize_angle(path_yaw - yaw_rad)
        # --- 5. 부호 있는 횡단 오차 (Signed Cross-Track Error, e) 계산 (Stanley의 CTE) ---
        # 차량의 진행 방향에 대한 수직 벡터(왼쪽 방향)와 (타겟 - 앞 차축) 벡터의 내적을 사용
        err_vec_x = target_pt.x - front_x
        err_vec_y = target_pt.y - front_y
        # 차량의 왼쪽 방향 단위 벡터: (-sin(yaw), cos(yaw))
        left_x = -np.sin(yaw_rad)
        left_y = np.cos(yaw_rad)
        # CTE 계산 (내적): e = Err · Left (왼쪽이 양수)
        cte = err_vec_x * left_x + err_vec_y * left_y

        # --- 6. Stanley 제어 법칙 (delta) 적용 ---
        # delta = psi_e + arctan(k * e / (v + epsilon))
        # k: 횡단 오차 이득, e: 횡단 오차, v: 속도, epsilon=1.0 (분모 0 방지)
        steer_term = np.arctan2(k * cte, v + 1.0)
        delta = heading_error + steer_term
        # --- 7. 조향 명령을 [-1, 1]로 정규화 ---
        steering_cmd = delta / max_steer_rad
        steering_cmd = np.clip(steering_cmd, -1.0, 1.0) # [-1.0, 1.0] 범위로 클리핑
        # 시뮬레이터 조향 방향과 맞추기 위해 부호를 반전 (필요에 따라)
        steering_cmd = -steering_cmd
        # --- 8. 단순 종방향 제어 (Simple Longitudinal Control) ---
        throttle_cmd = 0.0
        brake_cmd = 0.0
        # 큰 조향각이 필요할 때 목표 속도를 줄여 커브 통과 속도를 제한
        if abs(steering_cmd) > 0.305:
            current_target_speed = target_speed * 0.7
        else:
            current_target_speed = target_speed
        # 현재 속도가 목표 속도보다 낮으면 스로틀 적용
        if v < current_target_speed:
            # 비례 제어 (P-control)와 유사하게 속도 오차에 따라 스로틀을 계산
            throttle_cmd = 0.5 * (current_target_speed - v)
            throttle_cmd = np.clip(throttle_cmd, 0.0, 1.0)
        # 현재 속도가 목표 속도보다 일정 이상 높으면 브레이크 적용
        elif v > current_target_speed + 2.0:
            throttle_cmd = 0.0
            brake_cmd = 0.2
        # 디버깅 정보를 ROS 로그에 출력
        rospy.loginfo(
            f"Stanley -> CTE:{cte:.2f} | HeadErr:{np.degrees(heading_error):.2f} | Steer:{steering_cmd:.2f}"
        )
        return throttle_cmd, steering_cmd, brake_cmd
def main():
    """
    메인 함수: ROS 노드를 초기화하고 제어 루프를 실행합니다.
    """
    rospy.init_node("control_node") # ROS 노드 이름 초기화
    node = ControlNode()
    rate = rospy.Rate(30) # 제어 루프 주기 30 Hz
    # ROS 종료 시까지 반복
    while not rospy.is_shutdown():
        node.run() # 제어 로직 실행
        rate.sleep() # 지정된 속도로 루프 실행을 지연
if __name__ == "__main__":
    main()
