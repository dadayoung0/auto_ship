import cv2


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
        self.buoy_color = (255, 0, 0)
        self.smallball_color = (0, 255, 0)

    # 출력할 이미지 설정하기
    def set_image(self, img):
        self.img = img

    # 이미지 화면에 출력
    def show_image(self):
        # 이미지 크기 조정 후 화면에 출력
        cv2.imshow(self.title, cv2.resize(self.img, (self.show_w, self.show_h)))
        cv2.waitKey(1)

    # 이미지에 입력된 글자 개수만큼 표시(화면 좌상단 위치부터 표시)
    def add_text_on_img(self, *texts):
        # text 매개변수의 index, 내용 조회
        for idx, text in enumerate(texts):
            # text_num, text 개수에 따라 y축 위치 조정하여 글자 표시
            cv2.putText(self.img, text, (5, 30*(idx+self.text_num+1)), cv2.FONT_HERSHEY_COMPLEX, 1, self.text_color, 2)
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
        cv2.rectangle(self.img, (obj_data[1], obj_data[2]), (obj_data[3], obj_data[4]), color, 2)

        # 객체 이름 표시
        cv2.putText(self.img, obj_data[0], (obj_data[1], obj_data[2]-5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    # 선박 좌표 지도에 표시
    def draw_ship_on_map(self, ship_point):
        # 사각형 그리기
        cv2.rectangle(self.img, (ship_point[0] - 10, ship_point[1] - 10),
                      (ship_point[0] - 10, ship_point[1] - 10), self.ship_color, -1)

    # 종료
    def __del__(self):
        print("Graphic del")
        cv2.destroyAllWindows()
