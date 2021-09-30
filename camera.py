import cv2
import time
import numpy as np
import tensorflow.lite as tflite


# 동영상 설정
FRAME_W = 400
FRAME_H = 300
FPS = 5

# tensorflow 파일 위치
LABEL_DIR = './tensor_model/labels.txt'
MODEL_DIR = './tensor_model/model.tflite'

# tensorflow 정확도 최소값
CONFIDENCE_THRESHOLD = 0.5


# 젯슨 나노 카메라 동작 코드
def gstreamer_pipeline(capture_width=FRAME_W, capture_height=FRAME_H, frame_rate=FPS,
                       flip_method=0, display_width=FRAME_W, display_height=FRAME_H):
    return ("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
            % (capture_width, capture_height, frame_rate,
               flip_method, display_width, display_height))


# 카메라 제어
class Camera:
    # 초기화
    def __init__(self, tensor, tpu):
        print("Camera init")

        # 카메라 연결
        self.cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

        # FPS 측정용 시간 저장 변수
        self.prev_time = 0
        self.curr_time = 0

        # tensorflow 사용 여부(True 일때는 tensorflow, False 일때는 OpenCV 사용)
        self.tensor = tensor

        # tensorflow 사용할 때
        if self.tensor:
            # 라벨 파일 조회해서 리스트 생성
            self.labels = open(LABEL_DIR).read().strip().split("\n")

            # tpu 사용할 때
            if tpu:
                # 모델 불러오기
                self.model = tflite.Interpreter(model_path=MODEL_DIR,
                                                 experimental_delegates=[tflite.experimental.load_delegate('libedgetpu.so.1.0')])

            # tpu 사용하지 않을 때
            else:
                # 모델 불러오기
                self.model = tflite.Interpreter(model_path=MODEL_DIR)

            # 모델 초기화(텐서 할당하기)
            self.model.allocate_tensors()

            # 모델 설정값 조회
            self.input_details = self.model.get_input_details()     # 입력값 형식
            self.output_details = self.model.get_output_details()   # 출력값 형식
            self.input_h = self.input_details[0]['shape'][1]        # 입력 이미지 높이
            self.input_w = self.input_details[0]['shape'][2]        # 입력 이미지 너비

            # 모델 타입 판단(입력값이 실수형인지 판단)
            self.floating_model = (
                self.input_details[0]['dtype'] == np.float32)

        # tensorflow 사용하지 않을 때
        else:
            # 색체 탐지 영역 최소크기 지정
            self.area_min = 100

        # 딜레이 제어용 빈화면 출력
        for i in range(10):
            _, test_img = self.cap.read()
            cv2.imshow("test", test_img)
            cv2.waitKey(1)

    # fps 계산 > fps 반환
    def get_fps(self):
        # 현재 시간 저장
        self.curr_time = time.time()

        # 사진 찍는데 걸린 시간
        take_time = self.curr_time - self.prev_time

        # fps 계산(1장의 사진을 찍는데 걸린 시간으로 나누기)
        fps = 1 / take_time

        # 현재 시간을 이전 시간으로 저장
        self.prev_time = self.curr_time

        # 반올림 후 fps 반환
        return round(fps, 1)

    # 객체 탐지 > 객체 표시된 이미지, 탐지 결과 반환
    async def object_detection(self):
        # 카메라에서 프레임 받아오기
        _, img = self.cap.read()

        # 탐지 결과 저장할 리스트
        result = []

        # tensorflow 사용할 때(부표와 작은공 구별하여 인식 가능)
        if self.tensor:
            # BGR 이미지를 RGB 이미지로 변경
            img_tensor = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # 원본 이미지 크기 저장
            origin_h, origin_w, _ = img.shape

            # 원본 이미지를 tensorflow 이미지 크기로 변환
            img_tensor = cv2.resize(img_tensor, (self.input_w, self.input_h))

            # 객체 탐지에 사용할 이미지 데이터 생성(matrix 데이터를 tensor 데이터로 변환)
            input_data = np.expand_dims(img_tensor, axis=0)

            # 실수형 입력값을 갖는 모델일 때
            if self.floating_model:
                # 입력값을 실수형으로 변경(0~255 픽셀값을 -1~1 픽셀값으로 변경)
                input_data = (np.float32(input_data) - 127.5) / 127.5

            # 불러온 모델에 가공된 이미지 데이터 주입
            self.model.set_tensor(self.input_details[0]['index'], input_data)

            # 객체 탐지
            start = time.perf_counter()         # 객체 탐지 시작 시간
            self.model.invoke()                 # 객체 탐지 수행
            end = time.perf_counter() - start   # 객체 탐지 걸린 시간
            print('%.2f ms' % (end * 1000))

            # 객체 탐지 결과 받아오기
            scores = self.model.get_tensor(
                self.output_details[0]['index'])[0]      # 정확도
            boxes = self.model.get_tensor(self.output_details[1]['index'])[
                0]       # 테두리 좌표
            count = self.model.get_tensor(self.output_details[2]['index'])[
                0]       # 객체 갯수
            classes = self.model.get_tensor(self.output_details[3]['index'])[
                0]     # 객체 클래스 번호

            # 객체 개수만큼 반복
            for i in range(int(count)):
                # 정확도가 지정한 범위 안에 있을 때
                if (scores[i] > CONFIDENCE_THRESHOLD) and (scores[i] <= 1.0):
                    # 객체 테두리 좌표 저장(텐서플로우 이미지용 좌표를 원본 이미지용 좌표로 변환)
                    y_min = int(max(1, (boxes[i][0] * origin_h)))
                    x_min = int(max(1, (boxes[i][1] * origin_w)))
                    y_max = int(min(origin_h, (boxes[i][2] * origin_h)))
                    x_max = int(min(origin_w, (boxes[i][3] * origin_w)))

                    # 탐지 결과 리스트에 추가
                    result.append([self.labels[int(classes[i])],
                                  x_min, y_min, x_max, y_max])

        # tensorflow 사용하지 않을 때(부표만 인식)
        else:
            # opencv 사용해서 img 보정(BGR 이미지를 HSV 이미지로 변경)
            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # 빨간색 영역 추출
            img_mask1 = cv2.inRange(img_hsv, (0, 150, 0), (20, 255, 220))
            img_mask2 = cv2.inRange(img_hsv, (160, 150, 0), (180, 255, 220))
            img_detect = img_mask1 + img_mask2

            # 추출한 영역 경계선 찾기
            contours, _ = cv2.findContours(
                img_detect, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # 경계선 개수만큼 반복
            for cnt in contours:
                # 경계선의 너비가 최소 영역 이상일 때만 반복문 이어서 실행(조건 미달시 다음 경계선 좌표로 반복문 실행)
                if cv2.contourArea(cnt) < self.area_min:
                    continue

                # 경계선을 포함하는 최소 테두리 좌표 구하기
                rect = cv2.minAreaRect(cnt)     # 사각형 윤곽선 좌표
                box = cv2.boxPoints(rect)       # 사각형 꼭짓점 좌표
                box = np.int0(box)              # 정수형 좌표로 변환

                # 경계선 형태의 좌표 데이터에서 일반 좌표 데이터 추출
                x_min = box[box[:, :, 0].argmin()][0][0]
                y_min = box[box[:, :, 1].argmin()][0][1]
                x_max = box[box[:, :, 0].argmax()][0][0]
                y_max = box[box[:, :, 1].argmax()][0][1]

                # 탐지 결과 리스트에 추가
                result.append(["buoy", x_min, y_min, x_max, y_max])

        # 촬영한 이미지, 탐지 결과 반환
        return img, result

    # 종료
    def __del__(self):
        print("Camera del")

        # 카메라 반환
        self.cap.release()
