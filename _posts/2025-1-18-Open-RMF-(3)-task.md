---
title: "Open-RMF (3) rmf_task"
description: "How tasks are defined, assigned, and executed in Open-RMF"
date: 2025-1-17 18:00:00 +0800
categories: [Study, Open-RMF]
tags: [open-rmf]
pin: true
math: true
mermaid: true
---

<br>

rmf_core consists of:
- rmf_traffic: Core scheduling and traffic management systems
- rmf_traffic_ros2: rmf_traffic for ros2
- **rmf_task: Task planner for rmf**
- rmf_battery: rmf battery estimation
- rmf_ros2: ros2 adapters and nodes and python bindings for rmf_core
-  rmf_utils: utility for rmf

---

<br>

## 1. Overview

Open RMF는 다양한 로봇과 자원을 조율하여 복잡한 작업을 수행할 수 있도록 설계된 시스템이다. 여기서 Task는 시스템이 실행하는 모든 작업의 기본 단위이다. Open RMF에서 Task를 정의하고 관리하기 위해 두 가지 주요 패키지인 ```rmf_task```와 ```rmf_task_sequence```를 사용한다.


두 패키지는 [rmf_task repository](https://github.com/open-rmf/rmf_task)에서 확인할 수 있다.


<br>

## 2. rmf_task
### 2.1. Task와 Phase
Task(작업)는 하나 이상의 Phase(단계)로 구성된다. Phase는 의미 있는 동작의 단위로, 목표를 달성하기 위해 필요한 단계별 작업을 정의한다. Task의 최종 목표를 달성하기 위해 Phase를 순차적으로 수행한다.
예를 들어, ```Delivery``` Task는 다음과 같은 Phase로 구성될 수 있다.

**Delivery Task :**<br>
	&ensp; Phase 1: 픽업 위치로 이동<br>
	&ensp; Phase 2: 물품 픽업<br>
	&ensp; Phase 3: 드롭오프 위치로 이동<br>
	&ensp; Phase 4: 물품 전달<br>

```rmf_task```는 RMF에서 Task를 정의하고 관리하기 위한 API와 기본 클래스를 제공한다.

### 2.2. rmf_task::Task
```rmf_task::Task```는 실행 가능한 Task를 정의하기 위한 추상 인터페이스다. 이를 통해 사용자는 Task가 무엇을 의미하는지, 어떻게 모델링되고 실행되는지를 정의할 수 있다.
이를 구현하기 위해 다음과 같은 클래스를 사용한다.
- [Active](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Active.html) : Task의 실행 상태를 관리한다.
- [Booking](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Booking.html) : Task의 기본 정보(작업 이름, 시작 시간, 우선순위 등)를 저장한다.
- [Description](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Description.html) : Task의 세부 사항을 정의한다. 서로 다른 작업을 구분하는 데 유용하다.
- [Model](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Model.html) : Task와 관련된 시간과 로봇의 상태 변화를 계산한다.
- [Tag](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Task_1_1Tag.html) : Task의 정적 정보를 저장한다.

### 2.3. rmf_task::TaskPlanner
```rmf_task::TaskPlanner```는 로봇들 간의 Task 할당을 최적화하는 데 사용된다. 주어진 Task 집합과 fleet에 속한 로봇들에 대하여, 로봇의 배터리 수준, 작업 우선순위, 경로 등을 고려하여 Task들이 최단 기간 내에 완료되도록 최적 순서를 결정하고 배정한다. 필요 시 로봇의 Task 일정에 재충전 Task를 자동으로 추가할 수 있다.
Task는 ```Booking``` 요소와 ```Description``` 요소로 구성된 [request](https://docs.ros.org/en/rolling/p/rmf_task/generated/classrmf__task_1_1Request.html)로 표현된다.

<br>

## 3. rmf_task_sequence
### 3.1. Task와 Phase
```rmf_task_sequence```는 Task 객체를 일련의 Phase와 Event로 세부적으로 정의하고 실행하는 기능을 제공한다. 여기에서 Phase는 실행 가능한 Event의 집합으로 구성된다. Event는 작업을 수행하기 위해 로봇에서 실제로 실행되는 구체적인 동작 단위를 의미한다.
예를 들어, ```Delivery``` Task는 다음과 같이 구현된다.

**Delivery Task :**<br>
	&ensp; Phase 1: 픽업 위치로 이동<br>
	&ensp;&emsp; Event: GoToPlace<br>
	&ensp; Phase 2: 물품 픽업<br>
	&ensp;&emsp; Event: PickUp<br>
    &ensp; Phase 3: 드롭오프 위치로 이동<br>
	&ensp;&emsp; Event: GoToPlace<br>
	&ensp; Phase 4: 물품 전달<br>
	&ensp;&emsp; Event: DropOff<br>

사용자는 Open RMF에서 제공되는 표준 Event를 사용하거나, ```perform_action```을 통해 커스텀 Event를 정의할 수 있다.

### 3.2. rmf_task_sequence::Task

Event는 Task 내에서 특정 행동이나 결과를 달성하는 개별 단위를 의미한다. 다음과 같은 Event가 기본 제공된다.
- Bundle
- DropOff
- GoToPlace : 특정 위치로 이동
- PerformAction : 사용자가 정의한 커스텀 액션 수행
- PickUp
- Placeholder
- WaitFor : 특정 조건 충족 시까지 대기

사용자는 이러한 event/phase를 연결하여 원하는 task를 정의할 수 있다.

#### perform_action

```perform_action```은 사용자가 정의한 custom action을 수행하는 이벤트다.
```perform_action```의 커스텀 범위는 제한적이어서, 사용자가 로직을 구현할 때 복잡한 내부 시스템(ex. 교통 시스템, 문, 엘리베이터와의 상호작용 등)은 직접 다룰 필요가 없도록 설계되어 있다.
```perform_action```을 실행 중인 로봇은 "read-only" traffic agent로 동작한다. 다른 로봇이 해당 로봇을 회피하도록 경로를 조정하며, 해당 로봇은 Task가 완료될 때까지 다른 로봇을 고려하지 않는다.


<br>

## 4. Usage
```rmf_task_sequence```에 사용 가능한 몇 가지 Event와 Phase가 구현되어 있다. 여기에 사용자가 커스텀 Task 정의를 추가할 수 있다. Task는 JSON 형식으로 표현되며, JSON 스키마는 [rmf_api_msgs](https://github.com/open-rmf/rmf_api_msgs/blob/main/rmf_api_msgs/schemas/task_request.json)에 정의되어 있다. 

```description```의 스키마는 [rmf_fleet_adapter](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter/schemas)에 정의되어 있다.

실제 로봇에 이러한 Event를 명령하는 방법, 즉 ```active component```는 ```rmf_fleet_adapter```에 정의되어 있다.

<br>

## 5. Custom Task
[ROSCon2022: How custom tasks are defined, assigned, and executed in Open-RMF](http://download.ros.org/downloads/roscon/2022/How%20custom%20tasks%20are%20defined,%20assigned,%20and%20executed%20in%20Open-RMF.pdf)

<br>