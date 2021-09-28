from control import *


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

        # 자율 주행 상태 판별
        state = control.set_state()

        # 자율 주행 시도 횟수
        try_count = 0

        # end 출력 전까지 실행
        while state != 'end':
            # drive 상태일 때
            if state == 'drive':
                #drive
                
                # 자율 주행 시도 횟수 초기화
                 try_count = 0

            # turn 상태일 때
            elif state == 'turn':
                # U turn
                
                # 자율 주행 시도 횟수 초기화
                 try_count = 0

            # retry 상태일 때
            elif state == 'retry':
                # 시도 횟수 10번 미만일 때
                if try_count < 10:
                    # 자율 주행 상태 다시 판별
                    state = control.set_state()

                    # 자율 주행 시도 횟수 증가
                    try_count += 1
                
                # 시도 횟수 10번 이상일 때
                else:
                    # 주행 종료
                    print("시도 횟수가 10번을 넘었습니다. 자율 주행을 종료합니다.")
                    break

        # end 출력 후
        # 잠시 전진

    # 주행 종료
    del control
