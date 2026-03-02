import numpy as np
import time

class VTOLStateMachine:
    def __init__(self):
        self.waypoints= [
            np.array([10.0, 0.0, 5.0]),
            np.array([10.0, 10.0, 5.0]),
            np.array([0.0, 10.0, 5.0]),
        ]
        self.wp_idx = 0

        # 드론의 현재 상태 (state)
        self.curr_pos = np.array([0.0, 0.0, 0.0]) #(0,0,0)에서 현재 위치 시작
        self.battery = 100.0
        self.mode = "TAKEOFF"  #비행 모드 3개 (TAKEOFF,MISSION,RTL)
        self.takeoff_alt = 5.0 #목표 이륙 고도

    def update_drone(self): #0.5초마다 호출되어 드론의 상태를 업데이트 하는 함수
        #배터리가 30.0 미만이고 현재 모드가 "RTL"이 아닌 경우 알림 메시지 + 모드 변경
        if self.battery < 30. and self.mode != "RTL":
            self.mode = "RTL"

        if self.mode == "TAKEOFF":
            # 이륙 로직 : 고도가 이륙 고도보다 작은 경우 , 1.0씩 고도 상승
            if self.curr_pos[2] < self.takeoff_alt:
                self.curr_pos[2] += 1.0
            if self.curr_pos[2] >= self.takeoff_alt:
                self.mode = "Mission"
                print("이륙 완료, 임무 시작!")

        elif self.mode == "Mission": #미션 모드일 때 목표 좌표 비행
            if self.wp_idx >= len(self.waypoints):
                print("모든 웨이 포인트 도착! 미션 비행 종료. 귀환함")
                self.mode = "RTL"
                return True

            #waypoint logic
            target_pos = self.waypoints[self.wp_idx] #way point 가져오기
            wp_vector = target_pos - self.curr_pos
            wp_distance = np.linalg.norm(wp_vector)
            unit_vector = wp_vector / wp_distance

            if wp_distance < 1.0:
                self.wp_idx += 1
                print(f"{self.wp_idx} 번째 웨이 포인트 이동 중")
            else:
                self.curr_pos = self.curr_pos + (unit_vector * 0.5 ) #목표 지점까지 0.5m 씩 전진
                self.battery -= 1.0 #전진 할 때마다 1.0씩 배터리 감소


        elif self.mode == "RTL": # 비행 귀환 로직
            home = np.array([0.0,0.0,self.curr_pos[2]]) #고도는 통일
            dp_vector = home - self.curr_pos
            dp_distance = np.linalg.norm(dp_vector)
            dp_unit_vector = dp_vector / dp_distance

            if dp_distance < 1.0: # 상공에서 거리가 적을 때
                if self.curr_pos[2] > 1.0:
                    self.curr_pos[2] -= 0.5 #0.5씩 제자리 하강
                    self.battery -= 1.0
                else: # 고도가 1.0보다 작아지면 비행 종료 알림 , 메소드 종료
                    print("홈 도착! 비행 종료")
                    return False #메소드 종료
            else:
                self.curr_pos = self.curr_pos + (dp_unit_vector * 0.5) #0.5m씩 전진
                self.battery -= 1.0 # 전진할 때마다 1.0씩 배터리 감소


        print(f"[{self.mode}] mode 위치:[{self.curr_pos[0]:.2f},{self.curr_pos[1]:.2f},{self.curr_pos[2]:.2f}] 배터리 : {self.battery:.2f}%")
        return True #Method 전체 True로 실행

if __name__ == "__main__": #코드 실행점
    drone = VTOLStateMachine()
    is_flying = True
    print("🚀🚀Flight Start🚀🚀")

    while is_flying:
        is_flying = drone.update_drone() #메소드 실행
        time.sleep(0.5) #0.5초 주기로 실행


