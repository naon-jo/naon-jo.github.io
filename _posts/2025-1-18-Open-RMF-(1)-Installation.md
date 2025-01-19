---
title: "Open-RMF (1) Installation"
description: Open-RMF installation and demos
date: 2025-1-17 18:00:00 +0800
categories: [Study, Open-RMF]
tags: [open-rmf]
pin: true
math: true
mermaid: true
---

<br>

## 1. Installation

docker에서 진행한다.
```
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --network host --name network_test osrf/ros:galactic-desktop
만약 docker - docker 간 통신은 되는데, docker - local 간 통신은 안되면 위 코드로 컨테이너 만드심 됩니당
```

```
$ docker start deli
$ docker attach deli
```

### Setup Gazebo repositories
```
# sudo apt update
# sudo apt install -y wget
# sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
# wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

### Binary install
```
# apt-cache search ros-jazzy-rmf
```

### Building from sources

#### Install Additional Dependencies
Open RMF packages의 dependencies 중 ROS 관련이 아닌 dependencies를 다운로드한다.
```
# sudo apt update && sudo apt install \
    git cmake python3-vcstool curl \
    qt5-default \
    -y
# python3 -m pip install flask-socketio
# sudo apt-get install python3-colcon*
```

#### Install rosdep
```
# sudo apt install python3-rosdep
# sudo rosdep init
# rosdep update
```

#### Download the source code
ROS2 워크스페이스 생성 후 demo repository를 다운로드한다.
```
# mkdir -p ~/deli_rmf/src
# ls
deli_rmf
# cd deli_rmf
# wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
# vcs import src < rmf.repos
```

![](https://velog.velcdn.com/images/nnoa/post/9af915c4-163d-4d32-81dd-a6d9607edd9f/image.png)


ROS2 사전 세팅이 완료됐는지 확인한다.
```
# cd ~/deli_rmf
# rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

#### Compiling Instructions
```
# cd ~/deli_rmf
# source install/setup.bash
# colcon build
```

<br>

## 2. Demos
```
# cd ~/deli_rmf/src/demonstrations/rmf_demos
# ls
CONTRIBUTING.md  README.md  rmf_demos         rmf_demos_bridges        rmf_demos_gz    rmf_demos_tasks
LICENSE          docs       rmf_demos_assets  rmf_demos_fleet_adapter  rmf_demos_maps
```

### rmf_demos_gz
다음 경로에 Open RMF에서 제공하는 demo들을 확인할 수 있다.
```
# cd ~/deli_rmf/src/demonstrations/rmf_demos/rmf_demos_gz/launch
# ls
airport_terminal.launch.xml  clinic.launch.xml  office.launch.xml                     triple_H.launch.xml
battle_royale.launch.xml     hotel.launch.xml   office_mock_traffic_light.launch.xml
campus.launch.xml            include            simulation.launch.xml
```

```
# cd ~/deli_rmf/src/rmf
# ls
ament_cmake_catch2  rmf_building_map_msgs  rmf_simulation  rmf_traffic_editor  rmf_visualization_msgs
rmf_api_msgs        rmf_internal_msgs      rmf_task        rmf_utils
rmf_battery         rmf_ros2               rmf_traffic     rmf_visualization
```

<br>

## 3. Run RMF Demos
### 3.1. Launch 실행
airport_terminal에 대한 launch 파일을 실행하면 Gazebo GUI와 rviz2가 실행된다. 
```
# ros2 launch rmf_demos_gz airport_terminal.launch.xml
```

Gazebo GUI

![](https://velog.velcdn.com/images/nnoa/post/ca880e1d-9eac-4022-b23f-3a97015317ed/image.png)

rviz2

![](https://velog.velcdn.com/images/nnoa/post/ae8bd8dc-5586-4222-b134-ad64801d06a8/image.png)

### 3.2. Task Request
Task를 요청할 때, 사용자가 로봇의 이름을 지정하는 것이 아니라 RMF가 최적의 로봇을 계산하여 Task를 할당한다.

RMF가 지원하는 Task 종류는 다음과 같다.
- Loop Task
```
ros2 run rmf_demos_tasks dispatch_loop -s s07 -f n12 -n 3 --use_sim_time
```

- Delivery Task <br>
배달은 일반적으로 픽업 위치로 향하여 품목을 적재한 다음 품목을 하역하는 하역 위치로 이동하는 과정을 포함한다. 픽업 및 하역 장소에서 모바일 로봇을 로봇팔, 컨베이어 또는 다른 자동화 시스템과 인터페이스할 수 있다.
```
ros2 run rmf_demos_tasks dispatch_delivery -p mopcart_pickup -pd mopcart_dispenser -d spill -di mopcart_collector --use_sim_time
```

- Clean Task
```
ros2 run rmf_demos_tasks dispatch_delivery -p mopcart_pickup -pd mopcart_dispenser -d spill -di mopcart_collector --use_sim_time
```

<br>