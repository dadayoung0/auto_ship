import rospy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import os
import time


class Lidar:
    # 초기화
    def __init__(self):
        print("lidar init")

        # roscore 백그라운드에 실행
        os.system("screen -dmS core roscore")
        time.sleep(5)
        
        # 라이다 작동 프로그램 실행
        os.system("screen -dmS lidar roslaunch ydlidar_ros X4.launch")
        time.sleep(5)
        
        # 라이다 매핑 프로그램 실행
        os.system("screen -dmS mapping roslaunch hector_mapping mapping_default.launch")

        # 라이다의 위치, 장애물 정보 저장할 리스트
        self.position = []
        self.block = []

        # ROS 통신 시작(노드 생성)
        rospy.init_node('listener', anonymous=True)
        
    # 라이다의 위치 인식
    def position_callback(self, data):
        # data 변수에, 라이다에서 받아온 수 많은 정보들을 저장합니다
        data = data.transforms[0]

        # 이번에 받아온 값이 scanmatcher_frame(우리 배의 위치와 회전값) 주제가 맞다면
        if data.child_frame_id == "scanmatcher_frame":
            # 라이다의 x좌표(단위 m)
            self.position[0] = data.transform.translation.x
            # 라이다의 y좌표(단위 m)
            self.position[1] = data.transform.translation.y
            # 라이다의 회전값
            self.position[2] = data.transform.rotation.z
            
    # 라이다의 위치 반환
    def position_listen(self):
        # position 리스트 초기화
        self.position = []

        # position_callback()함수를 통해 유의미한 값이 리스트에 담겨진게 아니라면
        while len(self.position) == 0:
            # 계속 position_callback()함수를 이용하여 데이터를 받아옵니다
            rospy.Subscriber('/tf', TFMessage, self.position_callback)

        # position 반환
        return self.position

    def block_callback(self, data):
        self.block = data.ranges

    def block_listen(self):
        # block 리스트에 담긴 쓸모없는 값을 지워줍니다
        self.block = []

        # block_callback()함수를 통해 유의미한 값이 리스트에 담겨진게 아니라면
        while len(self.block) == 0:
            # 계속 block_callback()함수를 이용하여 데이터를 받아옵니다
            rospy.Subscriber('/scan', LaserScan, self.block_callback)

        # block 값을 반환합니다.
        return self.block

    # 가장 짧은 장애물과의 거리를 구하는 함수(0 제외)
    def shortest_block(self):
        blocks = self.block_listen()

        shortest_distance = 0
        shortest_degree = 0

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
