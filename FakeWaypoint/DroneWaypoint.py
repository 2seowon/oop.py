import numpy as np
import time


class MiniFlightTest:
    def __init__(self):
        # (x,y) 좌표만 사용한 WayPoint 생성
        self.waypoints = [
            np.array([0.0, 0.0]),
            np.array([10.0, 0.0]),
            np.array([10.0, 10.0]),
        ]
        self.wp_idx = 0  # 현재 가야 할 WayPoint 차례
        self.curr_pos = np.array([0.0, 0.0])

    def MoveDrone(self):
        if self.wp_idx >= len(self.waypoints):
            print("모든 웨이포인트 도착, 비행 종료! 🏁")
            return False  # 종료

        # 목표 웨이 포인트
        target_pos = self.waypoints[self.wp_idx]

        # 방향 벡터 (행렬) to_wp 를 구하기
        to_wp = target_pos - self.curr_pos

        # 드론과 목표 지점 사이의 거리 (dist_to_wp) 구하기
        dist_to_wp = np.linalg.norm(to_wp)

        # 거리가 1.0 미만이면 도착을 출력하고, wp_idx를 1씩 증가 시키기
        if dist_to_wp < 1.0:
            print(f">>> {self.wp_idx}번 웨이포인트 도착! <<<")
            self.wp_idx += 1
        else:
            # 방향 벡터를 남은 거리로 나누어 '길이가 1인 방향(단위 벡터)' 만들기
            direction = to_wp / dist_to_wp
            # 현재 위치에서 그 방향으로 0.5m 이동
            self.curr_pos = self.curr_pos + (direction * 0.5)

            # 소수점 2자리까지만 출력 :.2f 사용
        print(f"현재 좌표: [{self.curr_pos[0]:.2f}, {self.curr_pos[1]:.2f}]  향하는 목표: {target_pos} 남은 거리: {dist_to_wp:.2f}")
        return True


if __name__ == "__main__":
    test_node = MiniFlightTest()
    is_flying = True

    while is_flying:
        is_flying = test_node.MoveDrone()
        time.sleep(0.5) #메소드 실행 주기 0.5초
