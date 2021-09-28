from control import *


# ################### TEST LIST ####################
# test1 : mode1 - keyboard
# test2 : mode1 - joystick
# test3 : mode2 -
# ##################################################


def test1():
    # 클래스 불러오기
    control = ControlMode1(False)

    # 주행 시작
    control.drive_myself()


def test2():
    # 클래스 불러오기
    control = ControlMode1(True)

    # 주행 시작
    control.drive_myself()


if __name__ == "__main__":
    test1()

