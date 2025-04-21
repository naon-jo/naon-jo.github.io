---
title: "umr_ws: Phase go_to_place"
description: ""
date: 2025-4-16 19:00:00 +0800
categories: [Project, umr_ws]
tags: []
pin: true
math: true
mermaid: true
---


## 1. 개요


Nav2가 생성한 경로(Path)를 역방향으로 따라가며, 최종 목적지(Goal point)에 가까운 지점부터 일정 거리만큼 떨어진 위치(Stop point)를 찾는다.
- ```goal_pose``` 도달 전의 위치 ```stop_pose```에서 정지한 후, ```goal_pose```로 접근한다.
- 이때 정지하는 위치 ```stop_pose```은 전체 ```path```에서 ```goal_pose```로부터 일정 거리 ```stop_distance```만큼 떨어진 지점이다.


<br>

Input:

- path (nav_msgs/Path)
- stop_distance (float, meters): goal_pose로부터 stop_pose까지의 거리


Step:

1. ```path.poses``` 리스트를 뒤에서부터 탐색 (goal → start 방향)
2. 각 ```pose``` 간의 거리(p1 ~ p2)를 누적
3. 누적 거리가 ```stop_distance```보다 커지면, 해당 지점의 ```pose```를 Stop point로 설정


<br>


**예시**


Start o─────o─────o─────o─────G
            ↑                 ↑
           i=1                i=4 (goal)


- ```path.poses = [p0, p1, p2, p3, p4]```
- ```p4 = goal```
- 거꾸로 순회: ```p4→p3```, ```p3→p2```, ...
- 누적 거리가 ```stop_distance```를 넘는 순간, 해당 지점이 ```stop point```

<br>


**누적거리 accumulate**

- ```p4 → p3 = 0.5m``` 간격이면, ```accumulated = 0.5```
- ```p3 → p2 = 0.5m``` 간격이면, ```accumulated = 1.0```

이렇게 누적해서 ```accumulated >= stop_distance``` 조건이 되면, 그 지점이 Goal Point에서 ```stop_distance```만큼 떨어진 stop point가 된다.


<br>


## 2. 코드 설명
### 노드 역할

- ```Goals``` 서브스크라이브 : 목적지를 받음.
- ```MotionCommand``` 서브스크라이브 : 객체인식 결과에 따른 행동 계획을 받음.
- ```NavigateToPose``` 액션 클라이언트 : 목적지를 Nav2로 전달하여 주행 명령.

```python
self.goals_sub = self.create_subscription(
    Goals, "goals", self.goals_cb, 10, callback_group=self.cb_group
)

self.motion_sub = self.create_subscription(
    MotionCommand, "motion_comand", self.motion_cb, 10, callback_group=self.cb_group
)

self.nav_client = ActionClient(
    self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group
)
```

<br>

### goal 전달

```goals_cb```에서 ```/Goal```로 받은 목적지들의 리스트를 queue에 담고, ```send_next_goal```를 호출한다.
```send_next_goal```에서 queue에 담긴 goal을 하나씩 꺼내서 Nav2의 ```NavigateToPose``` 액션 Goal로 요청을 보낸다.


```python
async def goals_cb(self, msg: Goals):
    ...
    self.goal_queue.extend(msg.goals)
    ...

async def send_next_goal(self):
    ...
    goal_pose = self.goal_queue.pop(0)
    ...
```

<br>

### motion_command

```goal_pose```에 도착한 후 다음 ```goal_pose```로 가기 위해 ```motion_command```로 MOVE 명령을 받아야 한다. 

- ```motion_cb```에서 Move를 받으면
- ```send_next_goal``` 호출

```python
if self.motion_command == MotionCode.MOVE.value:
    self.get_logger().info("Received MOVE. Proceeding to goal_pose.")
    await self.send_next_goal()
else:
    self.awaiting_move = True
    self.get_logger().info("Waiting for MOVE command...")
```

<br>
