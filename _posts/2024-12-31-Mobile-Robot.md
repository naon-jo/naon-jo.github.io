---
title: "Mobile Robot"
description: 
date: 2024-12-31 18:00:00 +0800
categories: [Study, Mobile Robot]
tags: [mobile robot, autonomous driving]
pin: true
math: true
mermaid: true
image:
    path: "https://www.hyundai.co.kr/image/upload/asset_library/MDA00000000000014299/0b05ebc2e0be47bab877daa94c731de6.png"
---

<br>

## 1. 실외 모바일 로봇

실외 모바일 로봇을 주제로 기술 조사
- 사례 : 현대 로보틱스랩의 MobED
- 주요 기술
- 관련 연구

<br>

## 2. 국내외 사례

### 모베드(MobED, Mobile Eccentric Droid)

![](https://www.hyundai.co.kr/image/upload/asset_library/MDA00000000000014299/0b05ebc2e0be47bab877daa94c731de6.png){: width="972" height="590" .w-50 .right}

대부분의 모바일 플랫폼은 매우 간단한 이동 구현에 초점을 맞추고 있기 때문에 이동에 한계가 있다. 간단한 이동 능력을 가진 모바일 플랫폼은 한정된 조건에서만 활용이 가능하다.<br>
현대 로보틱스 랩의 MobED는 이러한 한계를 극복하는 신개념 모바일 플랫폼이다. 단순히 바퀴 달린 운송수단의 개념을 넘어, 역동적이며 안정적인 주행 성능을 갖춘 완전히 새로운 모바일 로봇이다.<br>
<br>



<br>

## 3. 주요 기술

### 3.1. SW

#### LiDAR SLAM 및 Visual SLAM
다양한 센서 융합을 통한 state estimation 기능과 정밀 측위 알고리즘<br>
지도 변화 구간 자동감지 및 업데이트 기능 및 자율 이동을 위한 지도 생성을 수행한다.

#### Active Posture Control 알고리즘
지면 적응을 위한 Active Posture Control 알고리즘<br>
4족 보행 로봇과 유사한 자유도를 가지는 MobED의 제어를 위해 센서 정보를 바탕으로 지면 상태와 관절의 정보를 파악한 후 자세 제어 알고리즘을 통해 각 관절을 제어한다.

#### 자세 제어 알고리즘
Kinematics/Dynamics/Optimazation/State Estimation 기반 알고리즘<br>
실시간으로 최적화되는 자세 제어 알고리즘을 통해 목적에 따라 최저 지상고와 휠베이스를 조절한다. 예를 들어, 붐비는 인도 환경이나 엘리베이터 등의 실내 공간에서는 휠베이스를 축소해 보행자의 불편을 줄일 수 있다.


<br>

### 3.2. HW 

#### 편심 매커니즘 기반 엑센트릭 휠(Eccentric Wheel)
인-휠 모터와 편심축 적용으로 구동, 제동, 조향<br>
일반적으로 바퀴로 움직이는 모빌리티는 바퀴의 중심에 축을 연결해 기동하는 방식을 채택하는데,엑센트릭 휠은 바퀴 가장자리에 축을 연결한 방식이다.<br>
바퀴에 연결된 편심축을 자유롭게 회전 시킬 수 있다. 편심축의 회전으로 노면 상태에 따라 능동적으로 적응해 흔들림을 최소화하고, 목적에 따라서 차체 최저 지상고와 휠베이스를 자유롭게 조절할 수 있다.<br>


#### 개별 동력 4륜 조향
총 4개의 휠이 독립적으로 조향<br>
각 바퀴마다 탑재된 세 개의 모터가 개별 바퀴의 동력과 조향 제어 기능을 수행하여 사선 주행과 360도 선회 주행이 가능하다.<br>
엘리베이터를 이용한 층간 이동부터 복잡한 실외 환경까지, 평면과 수직축 상에 존재하는 제약을 효율적으로 극복할 수 있다.

![](https://www.hyundai.co.kr/image/upload/asset_library/MDA00000000000014304/d6a42733fefe44639882111c7554b613.gif)


<br>

## 4. Related Research

### DynaVINS: A Visual-Inertial SLAM for Dynamic Environments

Authors: Seungwon Song, Hyungtae Lim, Alex Junho Lee, Hyun Myung<br>
Publisher: [IEEE](https://ieeexplore.ieee.org/document/9870851) RA-L (Robotics and Automation Letters)  <br>
Published: 2022<br>

동적 환경을 위한 robust visual-inertial state estimator 알고리즘에 관한 연구 &ensp;[Github](https://github.com/url-kaist/dynaVINS)<br>
- 동적 객체와 일시적으로 정적인 객체는 false positive loop closing 문제를 유발한다.
- 이를 해결하기 위해 동적 및 일시적으로 정적인 객체에 대해 로버스트한 VI-SLAM 프레임워크인 DynaVINS를 제안한다.
1. IMU preintegration으로 추정된 pose prior를 활용하여 동적 객체의 특징을 거부할 수 있는 robust bundle adjustment(BA)를 제안한다.
2. 루프가 닫힐 때 일시적으로 정적이 되는 객체의 영향을 줄이기 위해 키프레임 그룹화와 다중 가설 기반 제약 조건 그룹화 방법을 제안한다.

<br>

### Fast, Lightweight, and Robust Path Planning for Low-power Embedded Environments

Authors : Hwanhee Lee, Ilyong Yoon, Sangwoo Kim, and Dong Jin Hyun<br>
Journal : 2021 The 21st International Conference on Control, Automation and Systems [(ICCAS 2021)](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11024490)<br>
Publisher : ICROS<br>
Published : 2021.10<br>

저전력 임베디드 환경에서의 빠르고 가볍고 로버스트한 경로 계획 알고리즘에 관한 연구
- 저전력 임베디드 환경에서 안전한 로봇 내비게이션을 위한 빠르고 가벼우면서도 로버스트한 경로 계획 알고리즘을 제안한다.
- 제안하는 방법은 다음과 같은 단계로 구성된다.
1. 대략적이고 빠르게 초기 경로를 찾는 단계
2. 초기 경로를 로버스트하게 단축하는 단계
- 실험 결과, 제안한 알고리즘은 평균 반복 횟수와 평균 계산 시간, 평균 경로 길이 모두 RRT*, Informed RRT*보다 나은 성능을 보이며 저전력 임베디드에서 허용 가능한 수준을 보였다.

[[Review] Fast, Lightweight, and Robust Path Planning for Low-power Embedded Environments](https://naon-jo.github.io/posts/Review-Fast,-Lightweight,-and-Robust-Path-Planning-for-Low-power-Embedded-Environments/)

<br>
