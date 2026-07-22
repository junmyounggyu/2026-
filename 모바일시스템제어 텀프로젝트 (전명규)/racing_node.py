import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
import numpy as np
import pandas as pd
import math
# ==========================================
# 1. Global Path Manager (경로 및 가속도 로더)
# ==========================================
class GlobalPath:
    def __init__(self, filename):
        self.cx = []    # x 좌표
        self.cy = []    # y 좌표
        self.cyaw = []  # 헤딩 (Yaw)
        self.cv = []    # 목표 속도
        self.ca = []    # 목표 가속도 (Feedforward용)

        self.load_path(filename)

    def load_path(self, filename):
        try:
            df = pd.read_csv(filename)

            # 1. 필수 데이터 (위치, 속도) - 원본 좌표 사용 (보정 없음)
            self.cx = df['x_m'].values
            self.cy = df['y_m'].values
            self.cv = df['vx_mps'].values

            # 2. 헤딩(Yaw) 직접 계산 (안전성 확보)
            dx = np.gradient(self.cx)
            dy = np.gradient(self.cy)
            self.cyaw = np.arctan2(dy, dx)

            # 3. 가속도 데이터 로드
            if 'ax_mps2' in df.columns:
                self.ca = df['ax_mps2'].values
            else:
                self.ca = np.zeros_like(self.cx)

            print(f"✅ Path Loaded Successfully: {len(self.cx)} points.")

        except Exception as e:
            print(f"❌ Error loading path: {e}")
            self.cx, self.cy, self.cv, self.cyaw, self.ca = [0], [0], [0], [0], [0]
    def get_target_state(self, front_x, front_y):
        # 현재 위치에서 가장 가까운 경로점 인덱스 찾기
        dx = self.cx - front_x
        dy = self.cy - front_y
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)

        return (self.cx[target_idx],
                self.cy[target_idx],
                self.cyaw[target_idx],
                self.cv[target_idx],
                self.ca[target_idx])
# ==========================================
# 2. Stanley Controller (조향 제어)
# ==========================================
class StanleyController:
    def __init__(self):
        self.k = 0.6     # [튜닝] 조향 민감도
        self.Lf = 0.50  # [튜닝] 축거
    def compute(self, ego_x, ego_y, ego_yaw, ego_v, tx, ty, tyaw):
        # 1. Front Axle 좌표 변환
        front_x = ego_x + self.Lf * np.cos(ego_yaw)
        front_y = ego_y + self.Lf * np.sin(ego_yaw)

        # 2. 헤딩 오차
        heading_error = tyaw - ego_yaw
        while heading_error > np.pi: heading_error -= 2.0 * np.pi
        while heading_error < -np.pi: heading_error += 2.0 * np.pi
        # 3. 횡방향 오차 (CTE) - 벡터 내적 방식
        err_vec_x = tx - front_x
        err_vec_y = ty - front_y
        left_x = -np.sin(ego_yaw)
        left_y = np.cos(ego_yaw)
        cte = err_vec_x * left_x + err_vec_y * left_y

        # 4. Stanley 제어 법칙
        steer_term = np.arctan2(self.k * cte, max(ego_v, 1.0))
        delta = heading_error + steer_term

        # 5. 최종 조향 (부호 반전 없음: delta 그대로 사용)
        return delta
# ==========================================
# 3. PID + Feedforward Controller (속도 제어)
# ==========================================
class PIDController:
    def __init__(self):
        self.kp = 1.0
        self.ki = 0.05
        self.kd = 0.05
        self.kf = 0.1   # 피드포워드 게인
        self.prev_error = 0
        self.integral = 0
    def compute(self, target_v, current_v, dt, target_a):
        error = target_v - current_v

        self.integral += error * dt
        self.integral = max(min(self.integral, 10), -10)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        pid_out = self.kp * error + self.ki * self.integral + self.kd * derivative
        ff_out = target_a * self.kf

        return pid_out + ff_out
# ==========================================
# 4. Main ROS2 Node
# ==========================================
class RacingNode(Node):
    def __init__(self):
        super().__init__('racing_node')

        self.team_name = "MY_TEAM_NAME"  # 👈 본인 팀 이름 수정!
        csv_filename = 'optimized_trajectory.csv'

        self.path_manager = GlobalPath(csv_filename)
        self.stanley = StanleyController()
        self.pid = PIDController()
        self.prev_time = self.get_clock().now()
        # Subscriber
        self.sub_state = self.create_subscription(
            Float32MultiArray,
            '/mobile_system_control/ego_vehicle',
            self.state_callback,
            10)
        # Publisher
        self.pub_cmd = self.create_publisher(
            Vector3Stamped,
            '/mobile_system_control/control_msg',
            10)

        self.get_logger().info(f"🏎️ Racing Node Started (Clean Version). Team: {self.team_name}")
    def state_callback(self, msg):
        # 1. 데이터 파싱
        curr_x = msg.data[0]
        curr_y = msg.data[1]
        curr_yaw = msg.data[2]
        curr_v = msg.data[3]

        # 2. DT 계산
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 1e-3: dt = 1e-3
        self.prev_time = current_time
        # 3. 목표값 조회
        front_x = curr_x + self.stanley.Lf * np.cos(curr_yaw)
        front_y = curr_y + self.stanley.Lf * np.sin(curr_yaw)

        tx, ty, tyaw, target_v, target_a = self.path_manager.get_target_state(front_x, front_y)
        # 4. 제어값 계산
        # 4-1. Stanley
        raw_steer = self.stanley.compute(curr_x, curr_y, curr_yaw, curr_v, tx, ty, tyaw)
        max_steer_rad = np.deg2rad(20)
        norm_steer = np.clip(raw_steer / max_steer_rad, -1.0, 1.0)
        # 4-2. PID + Feedforward
        acc_cmd = self.pid.compute(target_v, curr_v, dt, target_a)

        throttle = np.clip(acc_cmd, 0.0, 1.0)

        # [안전장치] 조향각이 크면(약 10도 이상) 악셀을 20% 줄임 (횡방향 간섭 방지)
        if abs(norm_steer) > 0.5:
            throttle *= 0.85
        brake = 0.0
        if acc_cmd < 0:
            brake = np.clip(-acc_cmd, 0.0, 1.0)
        # 5. 명령 전송
        out_msg = Vector3Stamped()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = self.team_name
        out_msg.vector.x = float(throttle)
        out_msg.vector.y = float(-norm_steer) #조향이 반대로라면 -norm_steer
        out_msg.vector.z = float(brake)

        self.pub_cmd.publish(out_msg)
def main(args=None):
    rclpy.init(args=args)
    node = RacingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
