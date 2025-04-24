---
title: "umr_ws: 라이다 전방 물체 감지"
description: 
date: 2025-4-16 18:00:00 +0800
categories: [Project, umr_ws]
tags: []
pin: true
math: true
mermaid: true
---

<br>

## 1. 개요

### 물체 거리 검출

1. /scan 토픽으로부터 LIDAR 센서 데이터 (LaserScan)를 받아
2. 전방 범위(-30도 ~ +30도)에 0.5m 이하 거리의 장애물이 있는지를 확인


라이다의 정보는 고정 각도마다 상대거리값을 출력시켜주는 포인트 클라우드(Point Cloud)의 형식이다. 이 데이터는 ```sensor_msgs/LaserScan Message``` 메시지 형식의 ```ranges``` field로 가져올 수 있다.

```
Header header
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds]
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].
```

<br>

## 2. 구현

### check_obstacle_front
```check_obstacle_front``` 함수는 전방 장애물을 감지하고 거리를 반환한다.

전방 범위에 해당하는 거리 값은 r을 그대로 사용하고, 전방이 아닌 범위의 값은 inf로 처리했다. 그후 min()으로 가장 가까운 값을 구한다.

```python
angles = [angle_min + angle_increment * i for i, value in enumerate(ranges)]
front_ranges = [
    r if abs(normalize_angle(angle)) < front_angle_range_rad else float("inf") 
    for r, angle in zip(ranges, angles)
]
```

그런데 조건에 맞지 않는 값을 inf로 처리할 경우, 다른 코드에서 이를 다룰 때 inf가 실제로 측정된 정상 값이라고 오해할 수 있다. inf로 처리해도 전방 각도 외의 값을 제외하는 데는 문제가 없으나, 후속 분석이나 디버깅, 확장성 측면에서 문제가 예상되므로, 전방에 해당하는 값만 append하도록 변경했다. ```front_ranges```는 전방에 해당하는 진짜 측정값만 가지게 된다.

```python
front_ranges = []
for i in range(len(ranges)):
    angle = angle_min + i * angle_increment
    if abs(normalize_angle(angle)) <= front_angle_range_rad / 2:
        front_ranges.append(ranges[i])
```

```if msg.range_min <= r <= msg.range_max```에 의해 유효하지 않은 거리값을 제외하고 ```valid_ranges```에 담는다. 너무 가깝거나 너무 먼 경우, 혹은 장애물이 존재하지 않는 경우 ```valid_ranges```가 비어있게 된다. 이 경우에 inf를 반환한다.

```python
# Keep only valid values within sensor's measurable range
valid_ranges = [r for r in front_ranges if msg.range_min <= r <= msg.range_max]

return min(valid_ranges) if valid_ranges else float("inf")
```

```lidar_obstacle_detector``` 노드에서 ```check_obstacle_front``` 함수를 호출하여 물체 감지 여부를 판단한다. ```distance_min```이 ```distance_thresholdd```보다 작을 경우 전방에 장애물이 존재하는 것으로 판단하고, 그 거리를 퍼블리시한다. 이를 서브스크라이브하는 노드에서 로봇 행동 결정에 이용할 수 있다.

```python
distance_min = LPFL.check_obstacle_front(
    msg = msg, 
    front_angle_range_deg = front_angle_range_deg)
detected = distance_min < distance_threshold
self.get_logger().info(f"detected: {detected}, dist: {distance_min:.2f} m")
```

<br>