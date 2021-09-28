from adafruit_servokit import ServoKit                          #서보모터 드라이버를 사용하기 위해 패키지를 불러옵니다
import board                                                    #서보모터 드라이버 패키지에 종속된 패키지입니다
import busio                                                    #위와같이 서보모터 드라이버 패키지에 종속된 패키지입니다
import time                                                     #모터 제어시 delay를 주기 위해 time패키지를 불러옵니다


class Motor:
    # 초기화
    def __init__(self):
        print("Motor init")

        # i2c 통신을 젯슨 나노의 27,28번 핀으로 시작합니다
        i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))

        # servo_kit 에 서보모터 드라이버를 연결합니다
        self.servo_kit = ServoKit(channels=16, i2c=i2c_bus0)

        # 0번째 모터(서보모터)에 90도 각도를 주어, 서보모터가 다른 위치를 향하고 있을 때, 정면으로 향하게 합니다
        # 서보 모터의 경우 0~180의 범위를 가지며 0으로 갈수록 좌측, 180으로 갈수록 우측
        self.servo_kit.servo[0].angle = 90
        # 1번째 모터(ESC에 연결된 BLDC 모터)에 90 신호를 주어 ESC 신호를 보정합니다(2초정도 필요함)
        # BLDC 모터의 경우 0~180의 범위를 가지며 0으로 갈수록 빠르게 전진, 180으로 갈수록 빠르게 후진
        # 하지만 40~140정도의 범위에서 움직이는 것을 추천한다
        self.servo_kit.servo[1].angle = 90

        # 2초정도 기다립니다
        time.sleep(2)

    # 모터 동작 함수
    # degree : -90(좌측) ~ +90(우측)
    # speed : -100(후진) ~ +100(전진)
    def motor_move(self, degree: int, speed: int):
        # 0번째 모터(서보모터)를 주어진 각도로 움직입니다
        if -90 <= degree <= 90:
            self.servo_kit.servo[0].angle = degree + 90

        # 1번째 모터(BLDC 모터)를 90을 기준으로 주어진 속도로 움직입니다
        if -100 <= speed <= 100:
            self.servo_kit.servo[1].angle = 90 - (speed * (90/100))

        # 에러를 방지하기 위해 0.02초 지연합니다
        time.sleep(0.02)

    # 각도만을 변경합니다
    def motor_move_only_degree(self, degree: int):
        # 현재 속도를 받아옵니다
        speed = (90 - self.servo_kit.servo[1].angle) * (100/90)
        self.motor_move(degree, speed)

    # 속도만을 변경합니다
    def motor_move_only_speed(self, speed: int):
        # 현재 각도를 받아옵니다
        degree = self.servo_kit.servo[0].angle - 90
        self.motor_move(degree, speed)

    # 종료
    def __del__(self):
        print("Motor del")
        self.servo_kit.servo[0].angle = 90
        self.servo_kit.servo[1].angle = 90
        del self.servo_kit
