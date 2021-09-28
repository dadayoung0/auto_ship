import rospy                            # ros 관련 명령어들을 사용하기 위해 rospy 패키지를 불러옵니다
from sensor_msgs.msg import LaserScan   # ros에서 사용하는 통신규칙 중 Float32라는 규칙을 라이다에서 사용하기 때문에 패키지를 불러옵니다
from tf2_msgs.msg import TFMessage      # ros에서 사용하는 통신규칙 중 tf2라는 규칙을 라이다에서 사용하기 때문에 패키지를 불러옵니다
import os                               # 라이다를 작동시키키 위한 ROS 프로그램들을 실행시키고 종료하기 위해, 터미널에 명령을 보낼 필요가 있습니다. 따라서 os 패키지를 불러옵니다
import time                             # 코드를 일시정지 시켜, 일정시간 delay를 주기 위해 time 패키지를 불러옵니다


class Lidar:
    # 초기화
    def __init__(self):
        print("lidar init")

        # roscore 프로그램을 백그라운드에서 실행합니다
        os.system("screen -dmS core roscore")
        # roscore가 실행되어야 다음 작업이 실행되므로 잠깐 기다려줍니다
        time.sleep(7)
        # 라이다 작동 프로그램을 백그라운드에서 실행합니다
        os.system("screen -dmS lidar roslaunch ydlidar_ros X4.launch")
        # 라이다가 움직이기까지 잠시 기다려줍니다
        time.sleep(7)
        # 라이다를 이용해 매핑을 하고 현재 위치를 파악하는 프로그램을 백그라운드에서 실행합니다
        os.system("screen -dmS mapping roslaunch hector_mapping mapping_default.launch")

        # 라이다에서 파악한 자신의 위치가 담길 리스트를 만듭니다. 그리고 초기화합니다
        self.position = []
        self.block = []

        # ROS 시스템과 통신을 시작합니다(노드를 만든다고 합니다)
        self.lidar = rospy.init_node('listener', anonymous=True)        # ########################################################### 확인할 것!!

    def position_callback(self, data):
        # data 변수에, 라이다에서 받아온 수 많은 정보들을 저장합니다
        data = data.transforms[0]

        # 이번에 받아온 값이 scanmatcher_frame(우리 배의 위치와 회전값) 주제가 맞다면
        if data.child_frame_id == "scanmatcher_frame":
            # position 리스트의 0번째에, 이 배의 x 위치를 넣습니다(단위 m)
            self.position[0] = data.transform.translation.x
            # position 리스트의 1번째에, 이 배의 y 위치를 넣습니다(단위 m)
            self.position[1] = data.transform.translation.y
            # position 리스트의 2번째에, 이 배의 회전값을 넣습니다
            self.position[2] = data.transform.rotation.z

    def position_listen(self):
        # position 리스트에 담긴 쓸모없는 값을 지워줍니다
        self.position = []

        # position_callback()함수를 통해 유의미한 값이 리스트에 담겨진게 아니라면
        while len(self.position) == 0:
            # 계속 position_callback()함수를 이용하여 데이터를 받아옵니다
            self.lidar.Subscriber('/tf', TFMessage, Lidar.position_callback)

        # position 값을 반환합니다.
        return self.position

    def block_callback(self, data):
        self.block = data.ranges

    def block_listen(self):
        # block 리스트에 담긴 쓸모없는 값을 지워줍니다
        self.block = []

        # block_callback()함수를 통해 유의미한 값이 리스트에 담겨진게 아니라면
        while len(self.block) == 0:
            # 계속 block_callback()함수를 이용하여 데이터를 받아옵니다
            self.lidar.Subscriber('/scan', LaserScan, Lidar.block_callback)

        # block 값을 반환합니다.
        return self.block

    # 가장 짧은 장애물과의 거리를 구하는 함수(0 제외)
    def shortest_block(self):
        blocks = self.block_listen()

        shortest_distance = 0

        for idx, block in enumerate(blocks):
            if (shortest_distance == 0 or block < shortest_distance) and block != 0:
                    shortest_distance = block
                    shortest_degree = idx / 2.0

        return shortest_degree, shortest_distance, blocks

    # 종료
    def __del__(self):
        print("lidar del")

        # 프로그램을 역순으로 끄기 시작합니다. 위치 파악 프로그램부터 종료합니다.
        os.system("screen -S mapping -X quit")
        # 끄는데는 딱히 기다림이 필요없으므로 조금만 delay를 줍니다
        time.sleep(1)
        # 라이다 작동 프로그램을 끕니다
        os.system("screen -S lidar -X quit")
        # 또 조금 기다립니다
        time.sleep(1)
        # ROS 핵심 프로그램을 끕니다
        os.system("screen -S core -X quit")
