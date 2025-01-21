---
title: "Open-RMF (1) Installation"
description: Open-RMF installation and demos
date: 2025-1-17 18:00:00 +0800
categories: [Study, Open-RMF]
tags: [open-rmf]
pin: true
math: true
mermaid: true
image: https://osrf.github.io/ros2multirobotbook/images/grand_unified_diagram.png
---


<br>

- Docker Container에서 Open RMF 워크스페이스를 구축한다.
- Open RMF 패키지를 설치하고, demo를 실행하여 확인한다.

<br>

## 1. Installation

Docker에서 진행한다.
```
docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --network host --name network_test osrf/ros:galactic-desktop
```

```
$ docker start deli
$ docker attach deli
```

alias_settings.sh를 작성한다.
```
#nano alias_settings.sh
alias ros_domain="export ROS_DOMAIN_ID=13"

alias jazzy="source /opt/ros/jazzy/setup.bash; ros_domain; echo \"ROS2 jazzy is activated.\"; echo \"ROS_DOMAIN is set to 13\""

source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"


get_status() {
	if [ -z $ROS_DOMAIN_ID ]; then
		echo "ROS_DOMAIN_ID : 0"
	else
		echo "ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
	fi
	
	if [ -z $ROS_LOCALHOST_ONLY ]; then
		echo "ROS_LOCALHOST_ONLY : 0"
	else
		echo "ROS_LOCALHOST_ONLY : $ROS_LOCALHOST_ONLY"
	fi
}

ws_setting() {
        jazzy
        source ~/$1/install/setup.bash
        echo "$1 workspace is activated."
}
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

### Download the source code
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

All required rosdeps installed successfully
```
![](https://velog.velcdn.com/images/nnoa/post/51c547c9-456f-4007-af45-ebdc64aeb274/image.png)

```
# source /opt/ros/jazzy/setup.bash
# colcon build
```

```
# cd ~/deli_rmf
# source install/setup.bash
```

#### Compiling Instructions
```
# cd ~/deli_rmf
# source install/setup.bash
# colcon build

Summary: 55 packages finished [23min 17s]
  19 packages had stderr output: menge_vendor nlohmann_json_schema_validator_vendor pybind11_json_vendor rmf_api_msgs rmf_building_sim_gz_plugins rmf_demos_maps rmf_fleet_adapter rmf_fleet_adapter_python rmf_robot_sim_common rmf_task rmf_task_ros2 rmf_task_sequence rmf_traffic rmf_traffic_editor rmf_traffic_editor_test_maps rmf_traffic_ros2 rmf_visualization_rviz2_plugins rmf_visualization_schedule rmf_websocket

```

![](https://velog.velcdn.com/images/nnoa/post/9d2db6d9-a9a4-4c81-bd1d-0fa0869f54f4/image.png)

<br>

## 2. Package 구성

rmf_demos에 ```rmf_demos_tasks```와 ```rmf_demos_gz```가 존재한다.

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


<br>

## 3. Run RMF Demos

airport terminal demo를 실행하면 Gazebo와 rivz2가 실행된다.
```
# ros2 launch rmf_demos_gz airport_terminal.launch.xml
```

Gazebo
![](https://velog.velcdn.com/images/nnoa/post/4d49e722-d82c-4e36-9088-74898845116a/image.png)

rviz2
![](https://velog.velcdn.com/images/nnoa/post/bc5d3bbc-dca4-49e8-8c91-72e366a47702/image.png)


