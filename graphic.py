import cv2
import numpy as np

import math
import time

# 화면 제어(이미지 표시)


class Graphic:
    # 초기화
    def __init__(self, title, width, height):
        print("Graphic init")

        # 보여줄 화면 크기
        self.show_w = width
        self.show_h = height

        # 표시할 이미지
        self.img = ''

        # 화면 제목
        self.title = title

        # 텍스트 갯수
        self.text_num = 0

        # 색 지정
        self.text_color = (200, 0, 200)
        self.ship_color = (0, 0, 0)
        self.buoy_color = (0, 0, 255)
        self.smallball_color = (0, 255, 0)

        # 부표 좌표 리스트
        self.buoy_point_list = [[200, 100], [200, 300], [200, 500]]

    # 출력할 이미지 설정하기
    def set_image(self, img):
        self.img = img

    # 이미지 화면에 출력
    def show_image(self):
        # 이미지 크기 조정 후 화면에 출력
        cv2.imshow(self.title, cv2.resize(
            self.img, (self.show_w, self.show_h)))
        cv2.waitKey(1)

    # 이미지에 입력된 글자 개수만큼 표시(화면 좌상단 위치부터 표시)
    def add_text_on_img(self, *texts):
        # text 매개변수의 index, 내용 조회
        for idx, text in enumerate(texts):
            # text_num, text 개수에 따라 y축 위치 조정하여 글자 표시
            cv2.putText(self.img, text, (5, 30*(idx+self.text_num+1)),
                        cv2.FONT_HERSHEY_COMPLEX, 1, self.text_color, 2)
            self.text_num += 1

    # 객체 정보 이미지에 표시(객체 테두리 좌표에 표시)
    def draw_object_on_img(self, obj_data):
        # 객체가 buoy 일 때
        if obj_data[0] == 'buoy':
            # 색 지정
            color = self.buoy_color

        # 객체가 smallball 일 때
        else:
            # 색 지정
            color = self.smallball_color

        # 테두리 그리기
        cv2.rectangle(
            self.img, (obj_data[1], obj_data[2]), (obj_data[3], obj_data[4]), color, 2)

        # 객체 이름 표시
        cv2.putText(self.img, obj_data[0], (obj_data[1],
                    obj_data[2]-5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    # 선박 좌표 지도에 표시
    def draw_ship_on_map(self, ship_point, th):

        # th = 0

        def rotate(point, th):
            th = math.radians(th)

            cx = ship_point[0]
            cy = ship_point[1]  # centre of square coords

            point[0] -= cx
            point[1] -= cy

            xx = point[0]
            point[0] = point[0]*math.cos(th) - point[1]*math.sin(th)
            point[1] = xx*math.sin(th) + point[1]*math.cos(th)

            point[0] += cx
            point[1] += cy

            return [point[0], point[1]]

        points = np.array([rotate([ship_point[0], ship_point[1]-20], th),
                           rotate([ship_point[0]+10, ship_point[1]+10], th),
                           rotate([ship_point[0]-10, ship_point[1]+10], th)], dtype=np.int32)
        # 삼각형 그리기
        cv2.polylines(self.img, [points], True, self.ship_color, 1)
        # 사각형 그리기
        # cv2.rectangle(self.img, (ship_point[0] - 20, ship_point[1] - 20),
        #               (ship_point[0] + 20, ship_point[1] + 20), self.ship_color, 10)

    # 부표 좌표 지도에 표시
    def draw_buoy_on_map(self, buoy_points: list):
        # 원 그리기
        for buoy_point in buoy_points:
            cv2.circle(
                self.img, (buoy_point[0], buoy_point[1]), 10, self.buoy_color, 5)

    # 부표 좌표 추가

    def add_buoy_point(self, x, y):
        self.buoy_point_list.append([x, y])

    # 종료
    def __del__(self):
        print("Graphic del")
        cv2.destroyAllWindows()


w = int(4.5*60)
h = int(14.5*60)

g = Graphic('busan', w, h)
th = 0
while True:
    th += 1
    g.set_image(np.full((h, w, 3), (150, 200, 250), dtype=np.uint8))
    g.draw_ship_on_map([200, 600], th)
    g.draw_buoy_on_map(g.buoy_point_list)
    g.show_image()
