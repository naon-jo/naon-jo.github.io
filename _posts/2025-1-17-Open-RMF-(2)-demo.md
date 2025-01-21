---
title: "Open-RMF (2) Demos"
description: Open-RMF installation and demos
date: 2025-1-17 18:00:00 +0800
categories: [Study, Open-RMF]
tags: [open-rmf]
pin: true
math: true
mermaid: true
image: https://velog.velcdn.com/images/nnoa/post/361d7919-01b6-43bb-9ee8-ff3cf000e4c6/image.png
---

## 1. Overview


## 2. Task Request
Task를 요청할 때, 사용자가 로봇의 이름을 지정하는 것이 아니라 RMF가 최적의 로봇을 계산하여 Task를 할당한다.



### 2.1. Loop Task
```
ros2 run rmf_demos_tasks dispatch_loop -s s07 -f n12 -n 3 --use_sim_time
```

### 2.2. Delivery Task
배달은 일반적으로 픽업 위치로 향하여 품목을 적재한 다음 품목을 하역하는 하역 위치로 이동하는 과정을 포함한다. 픽업 및 하역 장소에서 모바일 로봇을 로봇팔, 컨베이어 또는 다른 자동화 시스템과 인터페이스할 수 있다.

적재 시스템을 ```dispensers```, 하역 시스템을 ```ingestors```로 표현한다.


#### rmf_fleet_adapter package




### 2.3. Clean Task
```
ros2 run rmf_demos_tasks dispatch_delivery -p mopcart_pickup -pd mopcart_dispenser -d spill -di mopcart_collector --use_sim_time
```

<br>

## 3. Run Demo

```
ros2 run rmf_demos_tasks dispatch_delivery -p mopcart_pickup -pd mopcart_dispenser -d spill -di mopcart_collector --use_sim_time
```

### 3.1. Launch 실행
launch 파일을 실행하면 Gazebo와 rviz2가 실행된다. 실행한 파일은 Ofifce world로, 로봇이 돌아다닐 수 있는 실내 사무실 환경을 구축한 것이다.
```
# ros2 launch rmf_demos_gz office.launch.xml
```

Gazebo

![](https://velog.velcdn.com/images/nnoa/post/2bee7493-823c-47bf-be15-86945e996291/image.png)


rviz2

![](https://velog.velcdn.com/images/nnoa/post/361d7919-01b6-43bb-9ee8-ff3cf000e4c6/image.png)


### 3.2. Task Request

Delivery와 Loop 두 가지 유형의 Task를 실행하도록 한다.

- Delivery Task <br>
robot에게 pantry로부터 hardware_2까지 coke를 배달하도록 요청할 수 있다.
```
ros2 run rmf_demos_tasks dispatch_delivery -p pantry -ph coke_dispenser -d hardware_2 -dh coke_ingestor --use_sim_time
```


- Loop Task
```
ros2 run rmf_demos_tasks dispatch_loop -s s07 -f n12 -n 3 --use_sim_time
```
<br>