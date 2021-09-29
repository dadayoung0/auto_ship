from control import *
import asyncio


# ################### MODE LIST ####################
# mode1 : 사용자가 직접 조종
# mode2 : camera + lidar 사용해서 자율 주행
# ##################################################


# 메인 함수(프로그램 실행)
if __name__ == "__main__":
    # 모드 입력
    # mode = int(input("어떤 모드를 실행할까요?(1: 직접 조종, 2: 카메라+라이다)\n"))
    mode = 2

    # mode1(직접 주행일 때)
    if mode == 1:
        # 클래스 불러오기
        control = ControlMode1(True)

        # 주행 시작
        control.drive_myself()

    # mode2(camera + lidar)
    elif mode == 2:
        # 클래스 불러오기
        control = ControlMode2()

        # 주행 시작
        loop = asyncio.get_event_loop()
        loop.run_until_complete(control.drive_auto())
        loop.close()
        # asyncio.run(control.drive_auto())

    # 주행 종료
    del control
