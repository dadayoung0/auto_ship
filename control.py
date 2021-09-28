from motor import *
from socket import *
from graphic import *
from lidar import *
from camera import *
from imu import *
import calculate as cal
import asyncio


# 실제 부표 크기(cm)
REAL_BUOY_SIZE = 100

# 실제 경기장 크기(M)
REAL_TRACK_WIDTH = 4.5
REAL_TRACK_HEIGHT = 14.5

# 지도 배율
MAGNIFICATION = 60

# 안전 거리
SAFE_DISTANCE = 0.5


# mode1 : 사용자가 직접 조종
class ControlMode1:
    # 초기화
    def __init__(self, joy_stick):
        print("ControlMode1 init")

        # 모터
        self.motor = Motor()

        # 모터 기본 셋팅값
        self.speed = 20
        self.degree = 40
        self.direction = 0

        # 소켓 통신 셋팅
        self.sock = socket(AF_INET, SOCK_STREAM)
        self.sock.bind(('', 1972))
        self.sock.listen(1)

        # 조이스틱 사용 여부
        self.joy_stick = joy_stick

    # 직접 주행 시작
    def drive_myself(self):
        # 소켓 통신 시작
        conn, _ = self.sock.accept()

        # 종료 버튼 입력까지 반복
        while True:
            try:
                # 소켓 통신으로 데이터 받아오기
                data = conn.recv(1024).decode('utf-8')

                # 조이스틱을 사용할 때
                if self.joy_stick:
                    if data != '':
                        conn.send("ok".encode('utf-8'))
                        self.motor.motor_move(int(data[0:3]), int(data[3:6]))

                # 조이스틱을 사용하지 않을 때
                else:
                    if data == "exit":
                        break
                    if data == "up":
                        self.direction = 1
                        self.motor.motor_move_only_speed(self.direction * self.speed)
                    elif data == "down":
                        self.direction = -1
                        self.motor.motor_move_only_speed(self.direction * self.speed)
                    elif data == "left":
                        self.direction = -1
                        self.motor.motor_move_only_degree(self.direction * self.degree)
                    elif data == "right":
                        self.direction = 1
                        self.motor.motor_move_only_degree(self.direction * self.degree)
                    elif data == "keyupbldc":
                        self.motor.motor_move_only_speed(0)
                    elif data == "keyupservo":
                        self.motor.motor_move_only_degree(0)
                    if data == "one":
                        self.speed = 15
                        self.motor.motor_move_only_speed(self.direction * self.speed)
                    elif data == "two":
                        self.speed = 25
                        self.motor.motor_move_only_speed(self.direction * self.speed)
                    elif data == "thr":
                        self.speed = 35
                        self.motor.motor_move_only_speed(self.direction * self.speed)
                    elif data == "for":
                        self.speed = 45
                        self.motor.motor_move_only_speed(self.direction * self.speed)
                    elif data == "fiv":
                        self.speed = 55
                        self.motor.motor_move_only_speed(self.direction * self.speed)
            # 오류 발생
            except error as err:
                print("오류 발생 : " + err)
                break

    # 종료
    def __del__(self):
        print("ControlMode1 del")
        del self.motor
        del self.sock


# mode2 : camera + lidar 사용해서 자율 주행
class ControlMode2:
    # 초기화
    def __init__(self):
        print("ControlMode2 init")

        # 모터 생성 및 기본값 설정
        self.motor = Motor()
        self.speed = 15
        self.direction = 0

        # 카메라 생성 및 출력 화면 설정
        self.camera = Camera(True, False)
        self.camera_graphic = Graphic("Camera Detect Result", FRAME_W, FRAME_H)

        # IMU 생성 및 초기값(처음 각도) 저장
        self.imu = Imu()
        self.forward_direction = self.imu.imu_read()

        # 지도 크기 설정 후 지도 생성(세로 : 14.5m * 60 = 870, 가로 : 4.5m * 60 = 270)
        self.map_w = REAL_TRACK_WIDTH * MAGNIFICATION
        self.map_h = REAL_TRACK_HEIGHT * MAGNIFICATION
        self.map_graphic = Graphic("Map", self.map_w, self.map_h)
        self.map_graphic.set_image(np.full((self.map_h, self.map_w, 3), (150, 200, 250), dtype=np.uint8))

        # 라이다 생성
        self.lidar = Lidar()

        # 회전 방향(1: 오른쪽으로 회전, -1: 왼쪽으로 회전)
        self.turn_direction = 1

        # 목적지 좌표 및 각도(x좌표, y좌표, 각도)
        self.destination = [0, 0, 0]

        # 현재 선박의 좌표 및 각도(x좌표, y좌표, 각도)
        self.ship_position = [0, 0, 0]

        # 카메라 상태 정보(객체 탐지 성공 여부)
        self.camera_state = True

        # 후방 벽과의 거리
        wall_distance_back = 0

        # 후방 벽과의 거리를 측정할 때까지 반복
        while wall_distance_back == 0:
            # 라이다로 가장 짧은 장애물 측정
            shortest_degree, shortest_distance, blocks = self.lidar.shortest_block()

            # 가장 짧은 거리가 180도 부근인지(후방 벽인지) 확인
            if 160 < shortest_degree < 200:
                # 가장 짧은 거리를 후방 벽과의 거리로 저장
                wall_distance_back = shortest_distance

        # 좌우 벽과의 거리 저장
        wall_distance_left = blocks[int((shortest_degree + 90) * 2)]
        wall_distance_right = blocks[int((shortest_degree - 90) * 2)]

        # 좌우 벽과의 거리의 합이 경기장의 가로 길이와 비슷한지 확인
        if REAL_TRACK_WIDTH - 0.5 < wall_distance_left + wall_distance_right < REAL_TRACK_WIDTH + 0.5:
            # 현재 선박의 좌표 저장(라이다로 측정한 좌표)
            self.ship_position = [wall_distance_left, REAL_TRACK_HEIGHT - wall_distance_back, self.forward_direction]

        else:
            # 현재 선박의 좌표 저장(경기장의 중앙이라고 가정)
            self.ship_position = [REAL_TRACK_WIDTH / 2.0, REAL_TRACK_HEIGHT - wall_distance_back, self.forward_direction]

        # 선박 지도에 그리기
        self.map_graphic.draw_ship_on_map([self.ship_position[0] * MAGNIFICATION, self.ship_position[1] * MAGNIFICATION])

    # 자율 주행 시작
    async def drive_auto(self):
        # 목적지 계산 및 장애물 정보 확인 함수 비동기 실행
        await asyncio.wait([
            self.set_destination(),
            self.drive_to_destination()
        ])

    # 목적지 계산 및 모터 동작
    # 카메라 + 모터
    async def set_destination(self):
        # 카메라 객체 탐지 실패 횟수
        detection_fail_count = 0

        # 무한 반복
        while True:
            # 카메라로 객체 탐지가 가능한 상황일 때
            if self.camera_state:
                # 객체 탐지 결과 저장
                img, results = self.camera.object_detection()

                # 객체 탐지에 실패했을 때
                if len(results) == 0:
                    # 객체 탐지 실패 횟수 1 더하기
                    detection_fail_count += 1

                    # 3번 이상 객체 탐지 실패 시 카메라 상태 변경
                    if detection_fail_count >= 3:
                        self.camera_state = False

                # 객체 탐지에 성공했을 때
                else:
                    # 객체 탐지 실패 횟수 초기화
                    detection_fail_count = 0

                    # 화면에 출력할 이미지 설정
                    self.camera_graphic.set_image(img)

                    # 가장 큰 부표 정보 저장할 리스트
                    biggest_buoy_xy = []
                    biggest_buoy_size = 0

                    # 탐지된 객체 수만큼 반복
                    for result in results:
                        # 객체 정보 이미지에 추가
                        self.camera_graphic.draw_object_on_img(result)

                        # 객체 크기 탐지
                        object_size = cal.get_average_size(result[1:3], result[3:])

                        # 가장 큰 부표 정보 저장
                        if object_size > biggest_buoy_size:
                            biggest_buoy_size = object_size
                            biggest_buoy_xy = result[1:]

                    # FPS 이미지에 추가
                    self.camera_graphic.add_text_on_img("FPS : " + str(self.camera.get_fps()))

                    # 사진 출력
                    self.camera_graphic.show_image()

                    # 부표 중심 좌표, 부표와의 거리, 부표의 크기 저장
                    biggest_buoy_center = cal.get_center_point(biggest_buoy_xy[:2], biggest_buoy_xy[2:])
                    target_buoy_distance = cal.get_real_distance(REAL_BUOY_SIZE, biggest_buoy_size)
                    target_buoy_away = cal.get_real_size(target_buoy_distance, biggest_buoy_size)

                    # 좌표가 우측에 있을 때
                    if biggest_buoy_center[0] > REAL_TRACK_WIDTH * MAGNIFICATION / 2.0:
                        # 회전 방향 오른쪽으로 설정
                        self.turn_direction = 1
                        
                    # 좌표가 좌측에 있을 때
                    else:
                        # 회전 방향 왼쪽으로 설정
                        self.turn_direction = -1

                    # 선박과 부표의 각도(좌측: 음수, 우측: 양수), 부표 좌표 저장
                    target_buoy_degree = self.turn_direction * cal.get_real_degree(target_buoy_distance, target_buoy_away)

                    # 목적지 좌표 저장
                    self.destination = cal.get_destination(target_buoy_distance, target_buoy_degree, self.ship_position[2])

                    ##################################모터 동작하기

    # 장애물 정보 확인하며 주행 보조
    # 라이다 + IMU + 모터
    async def drive_to_destination(self):
        # 무한 반복
        while True:
            # 카메라 주행 상태일 때
            if self.camera_state:
                # 라이다로 가장 짧은 장애물 측정
                shortest_degree, shortest_distance, _ = self.lidar.shortest_block()

                # 가장 짧은 장애물과의 거리가 특정 거리 이하일 때
                if shortest_distance < SAFE_DISTANCE:


    

    # 주행 상태 판별
    def set_state(self):      
        # 객체 탐지 성공 횟수가 7이상이고 마지막 객체 탐지를 성공했을 때
        if count > 7 and len(results) != 0:

            # 전방의 가장 가까운 장애물 찾기
            block_distance = 0
            for idx, block in enumerate(block_distances):
                # 후방의 장애물 정보 제외
                if 90 * 2 < idx < 270 * 2 or block == 0:
                    continue

                # 전방의 장애물 정보 중 가장 가까운 장애물 탐색
                elif block_distance == 0 or block < block_distance:
                    block_distance = block
                    block_degree = idx / 2.0

            # 라이다의 장애물 정보과 카메라의 장애물 정보가 비슷할 때
            if round(target_buoy_distance / block_distance) == 1 and round(target_buoy_degree / block_degree) == 1:
                
                # drive 반환
                return 'drive'

        # 객체 탐지 성공 횟수가 7이하거나 마지막 객체 탐지를 성공하지 못 했을 때
        else:
            # 현재 방향 저장
            current_direction = self.imu.imu_read()

            # 라이다로 가장 가까운 장애물 측정
            shortest_degree, _, _ = self.lidar.shortest_block()

            # 현재 방향이 시작 방향과 반대이면서 가장 가까운 장애물이 좌측이나 우측에 있을 때
            if 160 < current_direction < 200 and (80 < shortest_degree < 100 or 260 < shortest_degree < 280):
                # end 반환
                return 'end'
            
            # 현재 방향이 시작 방향이면서 가장 가까운 장애물이 좌측이나 우측에 있을 때
            elif (340 < current_direction < 360 or current_direction < 20) and (80 < shortest_degree < 100 or 260 < shortest_degree < 280):
                # turn 반환
                return 'turn'

            # 예외 발생시
            else:
                # retry 반환
                return 'retry'


    # 지도 갱신하기
    def update_map(self):
        # 라이다로 가장 가까운 장애물과의 거리 구하기(출발 지점 구하기)
        block_distance = 1

        # 지도에 현재 배 좌표 그리기
        self.map_graphic.draw_ship_on_map([self.map_w / 2, self.map_h - (block_distance * 6)])

    
        #
        # # 목적지 좌표 계산
        # destination_x =
        # destination_y = target_buoy_distance
        #
        # # 여유 공간 고려해서 목적지 조정
        # # 목적지 x 좌표가 양수일 때(우회전 할 때)
        # if destination_x > 0:
        #     # 부표 크기만큼 여유 공간 부여
        #     destination_x += REAL_BUOY_SIZE
        #
        # # 목적지 x 좌표가 0 또는 음수일 때(좌회전 할 때)
        # else:
        #     # 부표 크기만큼 여유 공간 부여
        #     destination_x -= REAL_BUOY_SIZE
        #
        # # 목적지 좌표 반환
        # return [destination_x, destination_y]

    # 종료
    def __del__(self):
        pass
