import math

# 미리 측정해둔 픽셀 거리
pixel_distance = 432


# 가로 길이와 세로 길이의 평균
def get_average_size(point1, point2):
    # x좌표 길이(가로 길이)
    distance_x = abs(point1[0] - point2[0])

    # y좌표 길이(세로 길이)
    distance_y = abs(point1[1] - point2[1])

    # 평균 계산하여 반올림 후 반환
    return round((distance_x + distance_y) / 2.0, 1)


# 객체와의 실제 떨어진 거리 계산
def get_real_distance(real_size, pixel_size):
    # 객체와의 실제 거리 계산
    real_distance = (pixel_distance * real_size) / pixel_size

    # 객체좌의 실제 거리 반올림 후 반환
    return round(real_distance, 1)


# 객체의 실제 (가로, 세로) 크기 계산
def get_real_size(real_distance, pixel_size):
    real_size = (real_distance * pixel_size) / pixel_distance

    return round(real_size, 1)


# 객체의 실제 각도 계산
def get_real_degree(real_distance, real_size):
    real_degree = math.degrees(math.asin(real_size/real_distance))

    return round(real_degree, 1)


# 목적지 좌표 계산
def get_destination(real_distance, real_degree, ship_point: list):
    destination_degree = ship_point[2] + real_degree
    destination_x = ship_point[0] + (real_distance *
                                     math.cos(math.radians(destination_degree)))
    destination_y = ship_point[1] - (real_distance *
                                     math.sin(math.radians(destination_degree)))

    return [destination_x, destination_y]


# 두 점의 가운데 좌표
def get_center_point(point1, point2):
    center_x = (point1[0] + point2[0]) / 2.0
    center_y = (point1[1] + point2[1]) / 2.0

    return [center_x, center_y]

# 두 점 사이의 거리


def get_distance(point1, point2):
    x = (point1[0] - point2[0]) ** 2
    y = (point1[1] - point2[1]) ** 2

    return round((x + y) ** 0.5, 2)


# 현재 배의 좌표 기준으로 목적지 각도 계산
def get_destination_degree(destination, ship_point):
    # 목적지가 선박보다 상단에 위치할 때
    if destination[1] < ship_point[1]:
        center_degree = math.degrees(math.asin(
            (destination[0] - ship_point[0]) / get_distance(destination, ship_point))) % 360

    # 목적지가 선박보다 하단에 위치할 때
    else:
        center_degree = (180 - math.degrees(math.asin(
            (destination[0] - ship_point[0]) / get_distance(destination, ship_point)))) % 360

    return center_degree
