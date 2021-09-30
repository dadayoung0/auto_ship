from motor import *
from socket import *
from graphic import *
from lidar import *
from camera import *
from imu import *
import calculate as cal
import asyncio


# 실제 부표 크기(M)
REAL_BUOY_SIZE = 0.4

# 실제 경기장 크기(M)
REAL_TRACK_WIDTH = 4
REAL_TRACK_HEIGHT = 10

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
                        self.motor.motor_move_only_speed(
                            self.direction * self.speed)
                    elif data == "down":
                        self.direction = -1
                        self.motor.motor_move_only_speed(
                            self.direction * self.speed)
                    elif data == "left":
                        self.direction = -1
                        self.motor.motor_move_only_degree(
                            self.direction * self.degree)
                    elif data == "right":
                        self.direction = 1
                        self.motor.motor_move_only_degree(
                            self.direction * self.degree)
                    elif data == "keyupbldc":
                        self.motor.motor_move_only_speed(0)
                    elif data == "keyupservo":
                        self.motor.motor_move_only_degree(0)
                    if data == "one":
                        self.speed = 15
                        self.motor.motor_move_only_speed(
                            self.direction * self.speed)
                    elif data == "two":
                        self.speed = 25
                        self.motor.motor_move_only_speed(
                            self.direction * self.speed)
                    elif data == "thr":
                        self.speed = 35
                        self.motor.motor_move_only_speed(
                            self.direction * self.speed)
                    elif data == "for":
                        self.speed = 45
                        self.motor.motor_move_only_speed(
                            self.direction * self.speed)
                    elif data == "fiv":
                        self.speed = 55
                        self.motor.motor_move_only_speed(
                            self.direction * self.speed)
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
        self.speed = 12
        self.direction = 0

        # 카메라 생성 및 출력 화면 설정
        self.camera = Camera(True, False)
        self.camera_graphic = Graphic("Camera Detect Result", FRAME_W, FRAME_H)

        # IMU 생성 및 초기값(처음 각도) 저장
        self.imu = Imu()
        self.forward_direction = self.imu.imu_read()

        # 지도 크기 설정 후 지도 생성(세로 : 14.5m * 60 = 870, 가로 : 4.5m * 60 = 270)
        self.map_w = int(REAL_TRACK_WIDTH * MAGNIFICATION)
        self.map_h = int(REAL_TRACK_HEIGHT * MAGNIFICATION)
        self.map_graphic = Graphic("Map", self.map_w, self.map_h)
        self.map_graphic.set_image(
            np.full((self.map_h, self.map_w, 3), (150, 200, 250), dtype=np.uint8))

        # 라이다 생성
        self.lidar = Lidar()

        # 회전 방향(1: 오른쪽으로 회전, -1: 왼쪽으로 회전)
        self.turn_direction = -1

        # 목적지 좌표 및 각도(x좌표, y좌표)
        self.destination = [0, 0]

        # 현재 선박의 좌표 및 각도(x좌표, y좌표, 각도)
        self.ship_position = [0, 0, 0]

        # 작업 상태
        self.process = True

        # 카메라 상태 정보(객체 탐지 가능 여부, 객체 탐지 실패 횟수)
        self.camera_state = True
        self.detection_fail_count = 0

        # 모터 상태 정보
        self.drive_mode = 'start'

        # 후방 벽과의 거리
        wall_distance_back = 0

        # 부표 좌표 추가 트리거 (True일때만 넣을 수 있게)
        self.add_buoy_point_trigger = True

        # 후방 벽과의 거리를 측정할 때까지 반복
        while wall_distance_back == 0:
            if input() == 'start':
                wall_distance_back = 1
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
            self.ship_position = [
                wall_distance_left, REAL_TRACK_HEIGHT - wall_distance_back, 0]

        else:
            # 현재 선박의 좌표 저장(경기장의 중앙이라고 가정)
            self.ship_position = [
                REAL_TRACK_WIDTH / 2.0, REAL_TRACK_HEIGHT - wall_distance_back, 0]

    # 자율 주행 시작
    async def drive_auto(self):
        # 작업 상태일 때
        while self.process:
            # 카메라로 객체 탐지가 가능한 상황일 때
            # if self.camera_state:
            #     # 카메라로 목적지 계산
            #     await asyncio.ensure_future(self.set_destination())
            self.set_destination()
            asyncio.sleep(0.1)
            # 배 위치 설정
            self.set_ship_position()
            # 모터 동작
            asyncio.ensure_future(self.motor_controller())
            # 지도 갱신
            self.map_graphic.draw_map([int(self.ship_position[0] * MAGNIFICATION), int(self.ship_position[1] * MAGNIFICATION)],
                                      int(self.ship_position[2]), self.destination)

    # 목적지 계산(카메라)
    def set_destination(self):
        self.camera_state = False
        print('start')
        # 객체 탐지 결과 저장
        img, results = self.camera.object_detection()
        print('객체 탐지 완료')

        # 화면에 출력할 이미지 설정
        self.camera_graphic.set_image(img)

        # 객체 탐지에 실패했을 때
        if len(results) == 0:
            # 객체 탐지 실패 횟수 1 더하기
            self.detection_fail_count += 1

            # 3번 이상 객체 탐지 실패 시 카메라 상태 변경
            if self.detection_fail_count >= 3:
                self.camera_state = False
            print('객체 탐지 실패')

        # 객체 탐지에 성공했을 때
        else:
            # 객체 탐지 실패 횟수 초기화
            self.detection_fail_count = 0

            # 가장 큰 부표 정보 저장할 리스트
            biggest_buoy_xy = []
            biggest_buoy_size = 0

            # 탐지된 객체 수만큼 반복
            for result in results:
                # 객체 정보 이미지에 추가
                self.camera_graphic.draw_object_on_img(result)

                # 객체 크기 탐지
                object_size = cal.get_average_size(
                    result[1:3], result[3:])

                # 가장 큰 부표 정보 저장
                if object_size > biggest_buoy_size:
                    biggest_buoy_size = object_size
                    biggest_buoy_xy = result[1:]

            # 부표 중심 좌표, 부표와의 거리, 부표의 크기 저장
            biggest_buoy_center = cal.get_center_point(
                biggest_buoy_xy[:2], biggest_buoy_xy[2:])
            target_buoy_distance = cal.get_real_distance(
                REAL_BUOY_SIZE, biggest_buoy_size)
            target_buoy_away = cal.get_real_size(
                target_buoy_distance, biggest_buoy_size)

            # 좌표가 우측에 있을 때
            if biggest_buoy_center[0] > REAL_TRACK_WIDTH * MAGNIFICATION / 2.0:
                # 회전 방향 오른쪽으로 설정
                self.turn_direction = 1

            # 좌표가 좌측에 있을 때
            else:
                # 회전 방향 왼쪽으로 설정
                self.turn_direction = -1

            # 선박과 부표의 각도(좌측: 음수, 우측: 양수), 부표 좌표 저장
            target_buoy_degree = self.turn_direction * \
                cal.get_real_degree(target_buoy_distance, target_buoy_away)

            # 목적지 좌표 저장
            buoy_point = cal.get_destination(
                target_buoy_distance, target_buoy_degree, self.ship_position[2])
            self.destination = [
                buoy_point[0] + (self.turn_direction * REAL_BUOY_SIZE), buoy_point[1]]

            # 모터 모드 변경
            self.drive_mode = 'drive'

            # 처음 한번만 부표 그리기
            if self.add_buoy_point_trigger:
                self.map_graphic.add_buoy_point(buoy_point)
                self.add_buoy_point_trigger = False

            print('객체 탐지 성공')

        # FPS 이미지에 추가
        self.camera_graphic.add_text_on_img(
            "FPS : " + str(self.camera.get_fps()))

        # 사진 출력
        self.camera_graphic.show_image()

    # 장애물 정보 확인하며 주행 보조(라이다 + IMU)
    def set_ship_position(self):
        # 라이다로 가장 짧은 장애물 측정
        _, shortest_distance, blocks = self.lidar.shortest_block()

        # 가장 짧은 장애물과의 거리가 안전 거리 이하일 때
        if shortest_distance < SAFE_DISTANCE:
            # 모터 모드 변경
            self.drive_mode = 'avoid'

        # IMU로 현재 선박 각도 측정(선박 초기 각도 기준)
        current_degree = (self.imu.imu_read() -
                          self.forward_direction) % 360

        # 선박의 회전 방향이 왼쪽일 때
        if self.turn_direction == -1:
            # 현재 선박 위치 업데이트
            self.ship_position = [blocks[round(((self.forward_direction - 90 + current_degree) % 360) * 2)],
                                  blocks[round(((self.forward_direction - 180 + current_degree) % 360) * 2)], current_degree]

        # 선박의 회전 방향이 오른쪽일 때
        else:
            # 현재 선박 위치 업데이트
            self.ship_position = [REAL_TRACK_WIDTH - blocks[round(((self.forward_direction + 90 + current_degree) % 360) * 2)],
                                  blocks[round(((self.forward_direction - 180 + current_degree) % 360) * 2)], current_degree]

            print('배 좌표 : ', self.ship_position)

        # 목적지 부근에 도착했을 때
        if cal.get_distance(self.ship_position[:2], self.destination) < 0.5 and self.drive_mode != 'start':
            self.drive_mode = 'ready'

    # 모터 동작하기
    async def motor_controller(self):
        # 목적지로 주행할 때
        if self.drive_mode == 'drive':
            # 목적지 각도 계산
            destination_degree = cal.get_destination_degree(
                self.destination, self.ship_position[:2])

            # 모터 지정 각도 저장
            motor_degree = destination_degree - self.ship_position[2]

            # 각도 보정
            if motor_degree > 180:
                motor_degree -= 360
            elif motor_degree < -180:
                motor_degree += 360
            if motor_degree > 45:
                motor_degree = 45
            elif motor_degree < -45:
                motor_degree = -45

            # 각도 지정
            self.direction = motor_degree

        # 근처의 장애물을 피할 때
        elif self.drive_mode == 'avoid':
            # 장애물 위치 확인
            shortest_degree, _, _ = self.lidar.shortest_block()

            # 장애물이 전방에 위치할 때만 방향 변경
            if 0 <= shortest_degree < 90:
                self.motor.motor_move(-45, self.speed)
            elif 270 < shortest_degree <= 360:
                self.motor.motor_move(45, self.speed)
            else:
                self.motor.motor_move(self.direction, self.speed)

            # 장애물 피할 때까지 모터 동작
            await asyncio.sleep(0.5)

            # 모터 모드 변경
            self.drive_mode = 'drive'

        # 다음 목적지를 인지하기 위해 준비할 때
        elif self.drive_mode == 'ready':
            # 장애물 위치 확인
            _, _, blocks = self.lidar.shortest_block()

            # 운행을 종료할 때(후방 각도 and 전방의 벽 인지)
            if 90 < self.ship_position[2] < 270 and blocks[int((180 - self.ship_position[2]) % 360) * 2] < 5:
                self.direction = 180 - self.ship_position[2]

                # 모터 동작
                self.motor.motor_move(self.direction, self.speed)

                # 잠시 동작
                await asyncio.sleep(0.5)

                # 모터 정지
                self.motor.motor_move(0, 0)

                # 프로세스 종료하기
                self.process = False

            # 각도를 조절할 때
            else:
                # 현재 회전 방향의 반대로 각도 지정
                motor_degree = self.turn_direction * (-45)

                # 각도 보정
                if motor_degree > 180:
                    motor_degree -= 360
                elif motor_degree < -180:
                    motor_degree += 360
                if motor_degree > 45:
                    motor_degree = 45
                elif motor_degree < -45:
                    motor_degree = -45

                # 각도 지정
                self.direction = motor_degree

                # 카메라 상태 변경
                self.camera_state = True
                self.add_buoy_point_trigger = True

        # 모터 동작
        self.motor.motor_move(self.direction, self.speed)

    # 종료
    def __del__(self):
        print("del")
