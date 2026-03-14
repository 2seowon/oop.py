import cv2 
import time

#1. 마커 Dictionary 준비 
aruco_dict = cv2. aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)  #50개의 id로 아루코 마커 생성
parameters = cv2. aruco.DetectorParameters() # 이미지 분석에 필요한 매개변수들 

#2. 노트북 웹 캠 열기 ( 0: 기본 카메라 켜짐)
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
time.sleep(1)

print("웹 캠이 켜졌습니다 , 종료하려면 창을 클릭하고 'q'를 누르세요.")  #아래 코드에서 로직 설정

while True:
    #3. 카메라에서 실시가능로 한 프레임씩 읽어 오기 (한 프레임 = 사진)
    ret , frame = cap.read()  #(T/F, 디코딩된 배열 데이터)
    if not ret: # 디코딩 실패의 경우 
        print("카메라를 읽을 수 없습니다.")
        break 
    
    #4. 사진을 흑백으로 변환 ( RGB 사용을 안하고 밝기만 표시) 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    #5. 핵심 로직 : 마커 찾기 
    #corenrs : 마커의 테두리 네 꼭 짓점 x,y 픽셀 좌표 
    #ids : 몇 번 패턴과 일치하는지 알려줌 
    #rejected : 가짜 마커의 모서리 좌표 
    corners, ids, rejected, = cv2.aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
    
    #6. 화면에 마커가 보인다면? 테두리와 ID 번호 그리기 
    if ids is not None: # 몇 번 패턴과 일치하는 지 값이 나옴 
        cv2.aruco.drawDetectedMarkers(frame,corners,ids) 
        
    #7. 창에 frame을 띄우기 
    cv2.imshow("Day 1: Marker Recognition", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'): # q 키를 누르면 무한 루프 탈출 
        break
    
#8. 끝날 떄는 카메라와 창을 닫아주기 (무한루프 종료 후 )
cap.release()
cv2.destroyAllWindows()