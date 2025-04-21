---
title: "umr_ws: System Architecture"
description: ""
date: 2025-1-4 18:00:00 +0800
categories: [Project, umr_ws]
tags: []
pin: true
math: true
mermaid: true
image:
    path: ""
---

<br>


## 1. rchitecture

![Image](https://github.com/user-attachments/assets/07ebf890-1a8e-4eb9-9b2a-085cb4eb7552)


Task Manager는 task를 의미있는 실행 단위인 phase로 해석하고, Goal Executor는 phase를 실제 수행 가능한 명령으로 변환하여 실행한다. 

이러한 설계에서 Task Manager는 task의 의미만을 추적하고, 세부 이동 방식은 독립된 실행 노드(Goal Executor)에 위임함으로써 캡슐화된다.
즉 stop point에서 로봇의 행동 판단 과정은 Task Manager가 몰라도 되므로, 이동 방식(Stop/Wait/Move)이 변경되어도 구조는 그대로 유지 가능하고, 실행 전략이 복잡해져도 Task Manager는 그대로 사용 가능하다.

<br>

## 2. Task
task는 여러 단계의 phase로 정의할 수 있다. 예를 들어, ```delivery```  task는 복수의 pickup, dropoff phase로 구성되어, 픽업 장소에서 물건을 픽업해서 드랍 장소로 이동하여 드랍하는 일련의 동작을 수행한다.

<br>

### go_to_places
```go_to_places``` task는 한 개 또는 여러개의 ```go_to_place``` phase로 정의된다. 여러 장소를 순차적으로 방문하는 일련의 동작을 정의한 task이다.

<br>

## 3. Phase
로봇이 실제 실행 가능한 행동 단위이다. phase들이 시퀀스로 구성되어 하나의 phases를 이룰 수 있다.

<br>

### go_to_place
```go_to_place```는 로봇이 목적지로 주행하는 행동을 정의한 phase이다. 목적지 근처에서 정지하여 목적지에 사람 또는 장애물의 존재 여부를 확인하고, 사람이 없으면 목적지로 접근하는 행동으로 정의된다.

1. ```stop_pose```로 이동
2. ```motion_command```가 Move이면 다음 ```goal_pose```로 이동

<br>

## 4. Nodes
### Task Manager


task를 해석하는 책임은 Task Manager에 있다. Task Manager는 task를 받아 Phases를 확인하고, 그것을 구성하는 단일 Phase를 순서대로 퍼블리시한다.

Task Server는 ```motion_command```에 대해 알 의무가 없다. ```motion_command```가 Wait인지 Move인지는 Goal Executor가 확인하고, Task Server는 그 결과로써 성공 여부를 ```successed```로, 실패 원인을 ```error_code```와 ```error_msg```로 확인한다.

<br>

### Phase Interpreter

Phase Interpreter는 Task Manager가 발행한 phase의 category와 place를 확인한다.

```go_to_place``` phase의 경우, 
```stop_pose```와 ```goal_pose```를 생성한다. 생성한 ```pose```를 순서대로 퍼블리시한다. 

<br>

### Goal Executor

Task Manager는 task를 받아 phase를 순서대로 발행한다. 이 phase의 실행 책임은 Goal Executor
에 있다. Goal Executor는 phase를 **로봇이 실제 수행할 수 있는 행동 명령**으로 변환한다.

이 노드는 로봇이 주행하는 데 필요한 목적지를 ```/goals``` 토픽으로 받는다. ```/goals```는 ```goal_pose```들의 리스트로 정의되며, 순서대로 꺼내 실행한다. 
로봇이 ```goal_pose```에 도착하면 Motion Planner에서 생성된 ```motion_command```를 확인하고, Move를 받으면 다음 목표를 퍼블리시한다.

<br>