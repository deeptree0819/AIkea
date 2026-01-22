#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import os
from glob import glob

# -------------------------- 설정 --------------------------
CHECKERBOARD = (8, 6)      # 내부 코너 개수 (columns, rows)
SQUARE_SIZE  = 0.025       # 체스보드 한 칸 실제 크기 (m)
OUTPUT_YAML  = 'camera_calib.yaml'
SAMPLES_DIR  = 'calib_images'  # 저장할 샘플 이미지 폴더
# -----------------------------------------------------------

# 1) 샘플 디렉터리 준비
os.makedirs(SAMPLES_DIR, exist_ok=True)

# 2) 체스보드 3D object points 준비 (Z=0 평면)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# 3) 캡처된 이미지에서 찾아낸 점 저장용
objpoints = []  # 3D 점
imgpoints = []  # 2D 점

# 4) 실시간 카메라 열기
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("카메라 열기 실패!")
    exit(1)

print("----- 카메라 캘리브레이션 -----")
print("화면에 체스보드를 비춘 뒤, 잘 보일 때마다 'c' 를 눌러 샘플을 저장하세요.")
print("'q' 를 누르면 캘리브레이션을 실행하고 종료합니다.")

sample_count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임 읽기 실패")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + 
        cv2.CALIB_CB_NORMALIZE_IMAGE +
        cv2.CALIB_CB_FAST_CHECK
    )

    display = frame.copy()
    if found:
        cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)

    cv2.putText(display, f"Samples: {sample_count}", (10,30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0), 2)
    cv2.imshow('Calibration', display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c') and found:
        # 서브픽셀 보정
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 
                    30, 0.001)
        cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)

        # 포인트 저장
        objpoints.append(objp)
        imgpoints.append(corners)

        # 샘플 이미지 파일로 저장
        fname = os.path.join(SAMPLES_DIR, f'sample_{sample_count:02d}.png')
        cv2.imwrite(fname, frame)
        sample_count += 1
        print(f" 샘플 저장: {fname}")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 5) 캘리브레이션 실행
if sample_count < 10:
    print("샘플이 충분치 않습니다. 최소 10장 이상 모아주세요.")
    exit(1)

# 이미지 사이즈 (가장 마지막 이미지 기준)
img_shape = gray.shape[::-1]

ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None
)

# 6) 재투영 오차 확인
total_error = 0
for i in range(len(objpoints)):
    img2, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], K, D
    )
    error = cv2.norm(imgpoints[i], img2, cv2.NORM_L2)/len(img2)
    total_error += error
mean_error = total_error/len(objpoints)
print(f"Reprojection error: {mean_error:.6f} pixels")

# 7) YAML로 저장
data = {
    'image_width': img_shape[0],
    'image_height': img_shape[1],
    'camera_matrix': {
        'rows': 3, 'cols': 3,
        'data': K.flatten().tolist()
    },
    'distortion_model': 'plumb_bob',
    'distortion_coefficients': {
        'rows': 1, 'cols': len(D.flatten()),
        'data': D.flatten().tolist()
    }
}
with open(OUTPUT_YAML, 'w') as f:
    yaml.dump(data, f, default_flow_style=False)
print(f"캘리브레이션 결과가 '{OUTPUT_YAML}' 에 저장되었습니다.")
