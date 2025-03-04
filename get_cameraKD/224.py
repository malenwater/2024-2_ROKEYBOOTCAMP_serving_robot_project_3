import cv2
import numpy as np
import glob

# 체커보드 설정
CHECKERBOARD = (4, 8)  # 내부 코너 수 (가로 4, 세로 8)
SQUARE_SIZE = 0.0215  # 한 칸의 실제 크기 (21.5mm = 0.0215m)
CAMERA_TO_CHECKERBOARD_DIST = 0.224  # 카메라와 체커보드 사이 거리 (224mm = 0.224m)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 카메라 내부 파라미터 (K)와 왜곡 계수 (D)
K = np.array([[1.39755585e+03, 0.00000000e+00, 7.29335091e+02],
              [0.00000000e+00, 1.39879236e+03, 4.19580562e+02],
              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

D = np.array([[ 2.97613204e-02,  5.68877296e-01,  1.56553998e-03,
                -3.11259015e-05, -1.90935153e+00]])

images = glob.glob('./224/224_*.jpg')

for fname in images:
    img = cv2.imread(fname)
    h, w = img.shape[:2]

    # 왜곡 보정을 위해 카메라 행렬과 왜곡 계수 적용
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))

    # 이미지 왜곡 보정
    undistorted_img = cv2.undistort(img, K, D, None, new_camera_matrix)

    gray = cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH +
                                             cv2.CALIB_CB_FAST_CHECK +
                                             cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 첫 번째 점과 마지막 점을 선택하여 대각선 거리 계산
        point1 = corners[0][0]  # 첫 번째 점 (x1, y1)
        point2 = corners[-1][0]  # 마지막 점 (x2, y2)

        # 점에 원을 그리기
        point1 = (int(point1[0]), int(point1[1]))  # 첫 번째 점 튜플로 변환
        point2 = (int(point2[0]), int(point2[1]))  # 마지막 점 튜플로 변환
        cv2.circle(undistorted_img, point1, 5, (0, 0, 255), -1)  # 첫 번째 점에 빨간 원
        cv2.circle(undistorted_img, point2, 5, (0, 0, 255), -1)  # 마지막 점에 빨간 원

        # 대각선 거리 계산 (유클리드 거리)
        distance = np.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)
        real_u_SQUARE_SIZE = 0.1637
        # m/pixel, cm/pixel, mm/pixel 변환
        m_per_pixel = real_u_SQUARE_SIZE / distance
        cm_per_pixel = m_per_pixel * 100
        mm_per_pixel = m_per_pixel * 1000

        # 거리 계산 (유클리드 거리)
        print(f"Distance between points: {distance:.2f} pixels")
        print(f"Distance/pixel: {m_per_pixel:.6f} m/pixel")
        print(f"Distance/pixel: {cm_per_pixel:.4f} cm/pixel")
        print(f"Distance/pixel: {mm_per_pixel:.2f} mm/pixel")

        # 각 점에 대해 출력 (첫 번째, 마지막)
        print(f"Point 1: ({point1[0]}, {point1[1]})")
        print(f"Point 2: ({point2[0]}, {point2[1]})")

        # 결과 이미지 표시
        cv2.imshow("Checkerboard Corners", undistorted_img)  # 이미지를 화면에 표시
        cv2.waitKey(0)  # 키 입력을 기다립니다.

cv2.destroyAllWindows()

# Distance between points: 996.59 pixels
# Distance/pixel: 0.000164 m/pixel
# Distance/pixel: 0.0164 cm/pixel
# Distance/pixel: 0.16 mm/pixel
# Point 1: (1200, 219)
# Point 2: (300, 647)