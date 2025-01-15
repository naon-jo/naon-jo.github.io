---
title: "ArUco Marker"
description:  ArUco marker generation, detection and pose estimation 
date: 2025-1-14 18:00:00 +0800
categories: [Study, ArUco Marker]
tags: [aruco marker]
pin: true
math: true
mermaid: true
image:
    path: "https://velog.velcdn.com/images/nnoa/post/96c33f20-0643-48e3-9498-9ff8a9acebe7/image.png"
---

<br>


## 1.  ArUCo Marker Generation
```
# Parameters
type = 'DICT_5X5_100'
ids = [0, 1, 2, 3]  # IDs for the markers
marker_size = 200  # Size of each marker

# Load the ArUco dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.__getattribute__(type))

# Generate and place each marker in the grid
for idx, marker_id in enumerate(ids):
    print("[INFO] generating ArUCo tag type '{}' with ID '{}'".format(type, marker_id))
    marker_img = cv2.aruco.generateImageMarker(arucoDict, marker_id, marker_size)

    # Save the marker
    output_path = 'markers/' + type + '_id_' + str(idx) + '.jpg'
    cv2.imwrite(output_path, marker_img)
    print(f"[INFO] Marker saved to {output_path}")
    
    # Display the combined image
    plt.imshow(marker_img, cmap='gray')
    # plt.axis('off')  # Hide axes
    plt.title("ArUco Marker " + str(marker_id))
    plt.show()

```

![](https://velog.velcdn.com/images/nnoa/post/a9a3eba5-fbbf-4db6-8860-4e7b60110168/image.png)

<br>

### Test Image 1

```
markers_dir = 'markers/'
file_names = [
    'DICT_5X5_100_id_0.jpg',
    'DICT_5X5_100_id_1.jpg',
    'DICT_5X5_100_id_2.jpg',
    'DICT_5X5_100_id_3.jpg',
]

canvas_size = 500  # Size of the canvas
marker_size = 100  # Marker size to resize

# Create a blank canvas
canvas = np.ones((canvas_size, canvas_size), dtype=np.uint8) * 255

# Load and place each marker at a random position on the canvas
for file_name in file_names:
    # Load marker image
    marker_path = os.path.join(markers_dir, file_name)
    marker_img = cv2.imread(marker_path, cv2.IMREAD_GRAYSCALE)

    # Resize marker to desired size
    marker_img = cv2.resize(marker_img, (marker_size, marker_size))

    # Place marker at a random position
    while True:
        start_y = random.randint(0, canvas_size - marker_size)
        start_x = random.randint(0, canvas_size - marker_size)

        # Check if the area is unoccupied
        roi = canvas[start_y:start_y + marker_size, start_x:start_x + marker_size]
        if np.all(roi == 255):  # Place only if area is blank
            canvas[start_y:start_y + marker_size, start_x:start_x + marker_size] = marker_img
            break

# Display the canvas
plt.imshow(canvas, cmap='gray')
# plt.axis('off')
plt.title("Randomly Placed ArUco Markers")
plt.show()

# Save the marker
output_path = 'markers/' + 'Randomly_placed_markers.jpg'
cv2.imwrite(output_path, canvas)
print(f"[INFO] Marker saved to {output_path}")
```

![](https://velog.velcdn.com/images/nnoa/post/8d78a5ed-9d9b-4abe-813a-bc10bf99ab52/image.png)

<br>

### Test image 2
```
# Load the marker image
print("[INFO] loading image...")
marker_img = cv2.imread('markers/Randomly_placed_markers.jpg')
height, width = marker_img.shape[:2]

# 원본 이미지의 4개 꼭짓점 좌표
src_points = np.array([
    [0, 0],               # top-left
    [width - 1, 0],       # top-right
    [width - 1, height - 1], # bottom-right
    [0, height - 1]       # bottom-left
], dtype="float32")

# 기울어진 이미지의 대상 좌표 설정
# 예: 이미지를 오른쪽으로 기울이는 변환
dst_points = np.array([
    [100, 100],               # new top-left
    [width - 50, 30],        # new top-right
    [width - 10, height - 20], # new bottom-right
    [20, height - 10]        # new bottom-left
], dtype="float32")

# 투시 변환 행렬 계산
matrix = cv2.getPerspectiveTransform(src_points, dst_points)

# 투시 변환 적용
warped_img = cv2.warpPerspective(marker_img, matrix, (width, height))


plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.title("Original Image")
plt.imshow(cv2.cvtColor(marker_img, cv2.COLOR_BGR2RGB))

plt.subplot(1, 2, 2)
plt.title("3D Skewed Image")
plt.imshow(cv2.cvtColor(warped_img, cv2.COLOR_BGR2RGB))
plt.show()

# Save the result
output_path = 'markers/' + '3d_skewed_markers.jpg'
cv2.imwrite(output_path, warped_img)
print(f"[INFO] Marker saved to {output_path}")
```
![](https://velog.velcdn.com/images/nnoa/post/a0c2f7c1-daf8-4ce9-ab60-2b9ab9ba2d2f/image.png)


<br>

## 3.  ArUCo Marker Detection
```
# Load the marker image
print("[INFO] loading image...")
marker_img = cv2.imread('markers/Randomly_placed_markers.jpg')

# Create aruco detector
type = cv2.aruco.DICT_5X5_100;
print("[INFO] detecting '{}' tags...".format(type))
arucoDict = cv2.aruco.getPredefinedDictionary(type);
arucoParams = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(arucoDict, arucoParams);
corners, ids, rejectedCandidates = detector.detectMarkers(marker_img)

green_bgr = (0, 255, 0)
blue_bgr = (0, 0, 255)
red_bgr = (255, 0, 0)


for corner, id in zip(corners, ids):
    # extract the marker corners (which are always returned in 
    # top-left, top-right, bottom-right, and bottom-left order)
    corner = np.array(corner).reshape((4, 2))
    (topLeft, topRight, bottomRight, bottomLeft) = corner

    # convert each of the (x, y)-coordinate pairs to integers
    topRightPoint = (int(topRight[0]), int(topRight[1]))
    topLeftPoint = (int(topLeft[0]), int(topLeft[1]))
    bottomRightPoint = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeftPoint = (int(bottomLeft[0]), int(bottomLeft[1]))

    # draw the bounding box of the ArUCo detection
    cv2.line(marker_img, topLeftPoint, topRightPoint, green_bgr, 2)
    cv2.line(marker_img, topRightPoint, bottomRightPoint, green_bgr, 2)
    cv2.line(marker_img, bottomRightPoint, bottomLeftPoint, green_bgr, 2)
    cv2.line(marker_img, bottomLeftPoint, topLeftPoint, green_bgr, 2)

    # draw each of the center (x, y) coordinates of the ArUco marker
    cv2.circle(marker_img, topLeftPoint, 4, red_bgr, -1)
    cv2.circle(marker_img, topRightPoint, 4, red_bgr, -1)
    cv2.circle(marker_img, bottomRightPoint, 4, red_bgr, -1)
    cv2.circle(marker_img, bottomLeftPoint, 4, red_bgr, -1)

    # compute and draw the center (x, y) coordinates of the ArUco marker
    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    cv2.circle(marker_img, (cX, cY), 5, blue_bgr, -1)

    # draw the ArUco marker ID on the image
    cv2.putText(marker_img, str(id), (topLeftPoint[0], topLeftPoint[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, green_bgr, 2)
    print("[INFO] ArUco marker ID: {}".format(id))

# Display the test image
plt.imshow(marker_img)
plt.title("Detected ArUco Markers")
plt.show()

# Save the result
output_path = 'markers/' + 'detected_markers.jpg'
cv2.imwrite(output_path, marker_img)
print(f"[INFO] Marker saved to {output_path}")
```
![](https://velog.velcdn.com/images/nnoa/post/895d0560-f8fe-4877-90ab-76234dde6f7d/image.png)

![](https://velog.velcdn.com/images/nnoa/post/96c33f20-0643-48e3-9498-9ff8a9acebe7/image.png)


<br>

## 4. Camera Calibration

```
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the marker image
print("[INFO] loading image...")
marker_img = cv2.imread('markers/3d_skewed_markers.jpg')
gray_img = cv2.cvtColor(marker_img, cv2.COLOR_BGR2GRAY)

# Create aruco detector
type = cv2.aruco.DICT_5X5_100;
print("[INFO] detecting '{}' tags...".format(type))
arucoDict = cv2.aruco.getPredefinedDictionary(type);
arucoParams = cv2.aruco.DetectorParameters()
detector   = cv2.aruco.ArucoDetector(arucoDict, arucoParams);
corners, ids, rejectedCandidates = detector.detectMarkers(marker_img)

calibration_matrix_path = 'calibration_matrix.npy'
distortion_coefficients_path = 'distortion_coefficients.npy'

k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)
print('k : ', k, 'd : ', d)

'''
frame - Frame from the video stream
matrix_coefficients - Intrinsic matrix of the calibrated camera
distortion_coefficients - Distortion coefficients associated with your camera

return:-
frame - The frame with the axis drawn on it
'''

print("[INFO] loading image...")


corners, ids, rejected_img_points = detector.detectMarkers(gray_img, arucoDict, parameters=arucoParams)

# If markers are detected
if len(corners) > 0:
    for i in range(0, len(ids)):
        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d)
        # Draw a square around the markers
        cv2.aruco.drawDetectedMarkers(marker_img, corners) 

        # Draw Axis
        cv2.drawFrameAxes(marker_img, k, d, rvec, tvec, 0.01)    # drawFrameAxes는 opencv 독립 모듈이라 aruco에 없음!

plt.imshow(marker_img)
plt.show()
```


### Reference.

[1] [https://docs.opencv.org/3.1.0/d9/d6d/tutorial_table_of_content_aruco.html](https://docs.opencv.org/3.1.0/d9/d6d/tutorial_table_of_content_aruco.html) <br>
[2] [https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python](https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python)

<br>