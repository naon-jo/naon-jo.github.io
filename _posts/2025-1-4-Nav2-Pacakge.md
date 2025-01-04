---
title: "Nav2 Package"
description: "저전력 임베디드 환경에서의 빠르고 가볍고 로버스트한 경로 계획 알고리즘"
date: 2025-1-4 18:00:00 +0800
categories: [Study, ROS2]
tags: [ROS2, Nav2]
pin: true
math: true
mermaid: true
---

<br>

Behavior Tree는 일반적으로 두 개의 작은 subtree로 나눌 수 있다. 

![](https://velog.velcdn.com/images/nnoa/post/ba33fd9f-9d60-4c52-85b5-6e4eef10faaa/image.png)


#### Navigation
- calculating a path
- following a path

contextual recovery behaviors for each of the above primary navigation behaviors



#### Recovery

behaviors for system level failures or items that were not easily dealt with internally.


## 1. Behavior Tree Nodes



Behavior Tree에서 논하는 Node는 ROS2에서의 Node와 다르다.

Behavior Tree에서 논하는 ActionNode는 ROS2에서의 Action Server와 무관하다. (정의가 같은 경우도 있음.)

nav2_behavior_tree 패키지에 navigation 전용 노드가 정의되어 있다. 미리 정의된 노드들을 Behavior Tree에 포함시킬 수 있다. 

https://docs.nav2.org/configuration/packages/configuring-bt-xml.html 

### 1.1. Action Nodes

- ComputePathToPose - ComputePathToPose Action Server Client (Planner Interface)

- FollowPath - FollowPath Action Server Client (Controller Interface)

- Spin, Wait, Backup - Behaviors Action Server Client

- ClearCostmapService - ClearCostmapService Server Clients

 
액션 노드는 액션이 정상적으로 완료되면 SUCCESS를 반환한다. 실행 중일 경우 RUNNING, 그 외에는 FALURE을 반환한다.

ClearCostmapService 액션 노드는 액션 서버 클라이언트가 아니라 서비스 클라이언트임에 주의한다.



### 1.2. Condition Nodes

- GoalUpdated - Checks if the goal on the goal topic has been updated

- GoalReached - Checks if the goal has been reached

- InitialPoseReceived - Checks to see if a pose on the intial_pose topic has been received

- isBatteryLow - Checks to see if the battery is low by listening on the battery top



condition이 true이면 SUCCESS를 반환, false이면 FAILURE을 반환한다. Nav2 BT에서 사용되는 주요 condition은 GoalUpdated이다. GoalUpdated는 특정 서브 트리에서 비동기적으로 check되는 condition인데, goal이 업데이트되면 repaln하게 한다.



### 1.3. Decorator Nodes

- Distance Controller - Will tick children nodes every time the robot has traveled a certain distance

- Rate Controller - Controls the ticking of its child node at a constant frequency. The tick rate is an exposed port

- Goal Updater - Will update the goal of children nodes via ports on the BT

- Single Trigger - Will only tick its child node once, and will return FAILURE for all subsequent ticks

- Speed Controller - Controls the ticking of its child node at a rate proportional to the robot’s speed



### 1.4. Control: PipelineSequence

PipelineSequence는 child가 RUNNING을 반환하면 이전 children을 re-tick한다. 

https://docs.nav2.org/behavior_trees/overview/nav2_specific_nodes.html 



