---
title: "Open-RMF (2) rmf_traffic"
description: "Scheduling and negotiating mobile robot traffic between multiple agents"
date: 2025-1-17 18:00:00 +0800
categories: [Study, Open-RMF]
tags: [open-rmf]
pin: true
math: true
mermaid: true
---

<br>

rmf_core consists of:
- **rmf_traffic: Core scheduling and traffic management systems**
- rmf_traffic_ros2: rmf_traffic for ros2
- rmf_task: Task planner for rmf
- rmf_battery: rmf battery estimation
- rmf_ros2: ros2 adapters and nodes and python bindings for rmf_core
-  rmf_utils: utility for rmf

---

<br>

## 1. Overview

robot_core의 핵심 기능은 로봇의 traffic conflict를 피하는 것이다. 이를 위한 두 가지 방법은 (1)Prevention(예방)과 (2)Resolution(해결)이다.

### Prevention
Traffic conflict 예방을 용이하게 하기 위해 플랫폼에 구애받지 않는 Traffic Schedule Database를 제공한다. Traffic Schedule은 실시간으로 업데이트되어 지연, 취소 또는 경로 변경 등을 반영한다.
RMF에 통합된 모든 fleet manager는 로봇의 예상 스케줄을 Traffic Schedule에 보고한다. Traffic schedule에 있는 정보를 통해 각 fleet manager는 다른 로봇과의 충돌을 피하는 경로를 계획할 수 있다.
이를 위해 ```rmf_traffic```은 ```Planner``` 클래스를 제공하여, 로봇이 사전에 정의된 그리드에 따른 경로를 엄격하게 따르는 표준 AGV(Automated Guided Vehicles)처럼 동작하도록 한다. 향후에는 예상치 못한 장애물을 우회할 수 있는 AMR(Autonomous Mobile Robots)과 유사한 유틸리티도 제공될 것이다.

### Negotiation
모든 traffic conflict를 완벽하게 예방하는 것은 불가능할 수 있다. 예를 들어, 로봇이 예상치 못한 장애물로 인해 지연되거나, 예측된 스케줄에 오류가 있을 수 있다.
이러한 경우를 대비하여 ```rmf_traffic```은 Negotiation(협상) 매커니즘을 제공한다. Traffic Schedule Database에서 두 개 이상의 스케줄 참가자 간에 충돌이 예상되면, 관련 fleet manager들에게 충돌 알림을 보내고 협상을 시작한다. 각 fleet manager는 선호하는 스케줄을 제출하고, 서로의 스케줄을 조정하여 충돌을 피하는 방안을 모색한다. system integrator가 배치한 third-party judge가 가장 적절한 제안을 선택하고, 해당 일정을 따르도록 각 fleet manager들에게 통보한다.
긴급 상황 등으로 인해 현재의 교통 스케줄이 적절하지 않을 경우, 특정 참가자가 의도적으로 traffic conflict를 스케줄에 게시하여 협상을 강제할 수 있다. 이때 third-party judge는 높은 우선순위를 가진 참가자를 우선시하여 협상 결과를 조정할 수 있다.

### Traffic Schedule
Traffic Schedule은 시스템 내 모든 로봇의 예상 이동 경로를 중앙에서 관리하는 데이터베이스다. 이는 미래의 예상 경로를 포함하며, 다양한 로봇 fleet의 의도된 경로 간의 충돌을 식별하고, 충돌이 발견되면 해당 fleet에게 알린다.
알림을 받은 플릿들은 앞서 설명한 협상 과정을 통해 문제를 해결한다.

![](https://osrf.github.io/ros2multirobotbook/images/rmf_core/schedule_and_fleet_adapters.png)

### Fleet Adapters
RMF에 참여하는 각 로봇 fleet은 자체 API를 RMF의 교통 스케줄링 및 협상 시스템과 연결하는 fleet adapter를 갖추어야 한다. Fleet adapter는 또한 문 열기, 엘리베이터 호출, 디스펜서 활성화 등 다양한 표준화된 스마트 인프라 인터페이스와의 통신을 처리한다.

다양한 로봇 fleet과의 호환성을 높이기 위해, 실제 fleet manager에서 일반적으로 기대되는 4가지 제어 범주를 정의한다. 이 제어 범주는 로봇의 이동과 task 수행을 관리하기 위한 수준을 의미하며, 각 로봇 fleet이 RMF에 얼마나 통합될 수 있는지를 보여준다.



## 2. rmf_traffic
[rmf_traffic](https://github.com/open-rmf/rmf_traffic) 패키지는 다중 에이전트 간의 모바일 로봇 traffic을 스케줄링하고 협상하는 알고리즘과 데이터 구조를 구현한다.
![](https://raw.githubusercontent.com/open-rmf/rmf_traffic/refs/heads/main/docs/rmf_core_integration_diagram.png)

<br>