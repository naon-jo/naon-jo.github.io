---
title: "Behavior Tree"
description:  Behavior tree library, BehaviorTree CPP V4 
date: 2025-1-9 18:00:00 +0800
categories: [Study, Behavior Tree]
tags: [behavior tree]
pin: true
math: true
mermaid: true
image:
    path: "https://velog.velcdn.com/images/nnoa/post/a3b5c081-be8a-4353-8e52-ecbd478e5ff4/image.png"
---

<br>

## 1. FSM vs BT

#### 유한 상태 기계(Finite State Machine, FSM)
FSM은 상태(state)와 전이(transition)으로 구성된다. 상태는 동시에 실행되는 행동(action)의 집합이다. 전이는 조건(condition)을 포함한다. 전이의 조건이 충족되면 상태는 다른 상태로 이동한다.

FSM은 상태가 많아지게 되면 노드가 매우 복잡해진다는 단점이 있다. 


#### 계층적 유한 상태 기계(Hierarchical Finite State Machines, HFSM)
HFSM은 이러한 FSM의 단점을 극복하기 위해 등장한 개념이다. 상태를 계층화(그룹화)함으로써 특정 문맥을 가진 상태를 재사용할 수 있게 해준다.
HFSM은 FSM에 비해 상태를 계층화하여 일부 확장성을 제공하지만, 여전히 상태 전이 조건에 의해 사용성이 제한된다. 특정 상태에서 전이되어야 한다는 조건이 있기 때문에, 같은 상태를 다른 문맥에서 사용할 수는 없다는 뜻이다.

#### 행동 트리(Behavior Tree, BT)
 이를 해결하기 위해 등장한 Behavior Tree(BT)는 상태를 캡슐화하여 모듈성을 제공한다. 같은 상태를 재작성하지 않고도 다양한 목적과 상황에서 재사용할 수 있다.

<br>

|           | Behavior Tree                    | FSM                                |
|-------------|----------------------------------|------------------------------------|
| 구조        | 트리 형태                        | 상태 다이어그램                   |
| 확장성      | 뛰어남                           | 복잡한 시스템에서 비효율적일 수 있음 |
| 가독성      | 높은 가독성                      | 상태가 많아질수록 복잡해짐         |
| 재사용성    | 노드 단위로 재사용 가능          | 재사용 어려움                      |
| 디버깅      | 각 노드의 상태를 쉽게 추적 가능 | 상태 전이에 대한 디버깅이 어려움   |

<br>

## 2. Behavior Tree
### 2.1. 기본 개념

BT는 루트 노드에서 시작하여 트리의 각 노드로 신호를 전달한다. 이 신호를 tick이라고 하며, 이는 특정 조건을 평가하거나 작업을 수행하라는 명령을 의미한다. tick은 노드가 활성화되었음을 나타내며, root 노드에서 시작해 자식 노드로 전파된다.

![](https://velog.velcdn.com/images/nnoa/post/2cf52b6d-f8d0-490d-8a53-b828b8816b61/image.png)

<br>

#### 핵심 동작
- **Tick** : root 노드에서 자식 노드로 tick 전달
- **Up** : 각 노드는 자신의 상태(Success, Failure, Running)를 부모 노드로 반환

<br>

#### 구성 요소
- **Root Node**: 트리의 시작점으로, tick을 자식 노드로 전달
- **Composite Node(Control Flow Node)** : 자식 노드의 실행 순서를 제어
- **Decorator Node** : 자식 노드의 실행 조건을 수정하거나 결과를 조작
- **Leaf Node(Execution Node, Task Node)** : 아래에 노드가 있을 수 없는 트리의 말단으로, 실제 행동(Action)이나 조건(Condition) 평가를 수행


#### 실행 과정

Behavior Tree는 루트 노드부터 시작하여 각 노드를 순차적으로 평가한다. 각 노드는 [깊이 우선 탐색](https://en.wikipedia.org/wiki/Depth-first_search)으로 평가된다.


실행 결과는 세 가지 상태로 나타난다.
- Success : 노드가 성공적으로 완료됨.
- Failure : 노드가 작업을 실패하거나 조건을 충족하지 못함.
- Running(Continue) : 노드가 작업을 계속 진행 중이며, 완료되지 않음.

<br>

### 2.2. 비동기 동작

반응형(reactive) BT를 설계할 때 중요한 컨셉은 다음과 같다.
- 비동기 동작
- 동시성

<br>

#### 비동기 vs 동기
비동기 동작으로 긴 작업을 효율적으로 처리할 수 있다. 실행 시간이 오래 걸리는 작업(Action)은 가능한 한 빨리 Running 상태를 반환하여 트리의 나머지 부분이 계속 실행될 수 있도록 한다.

비동기 동작의 주요 특성
- **Running 상태** : 비동기 노드는 tick되면 즉시 Success나 Failure를 반환하지 않고, Running 상태를 반환하며 실행을 계속한다.
- **Halt 메서드** : 실행 중인 비동기 노드는 필요 시 halt() 메서드를 호출하여 중단할 수 있다.


<br>

#### 동시성 vs 병렬성
동시성과 병렬성의 차이를 이해하면 BT.CPP에서 비동기 동작을 더 효과적으로 설계할 수 있다.

- 동시성: 단일 스레드에서 여러 작업이 교대로 실행되는 상태
- 병렬성: 여러 스레드 또는 프로세스에서 작업이 동시에 수행되는 상태

BT.CPP의 동작 방식
- 트리 실행 엔진은 **싱글 스레드**로 동작한다.
- 모든 ```tick()``` 메서드는 **순차적으로** 실행된다.
- 특정 ```tick()``` 메서드가 block되면 전체 트리의 실행 플로우도 block된다.

<br>

긴 작업을 비동기로 처리하면 트리의 다른 부분이 계속 실행될 수 있다. 아래 예에서 ```ActionE```는 비동기적으로 실행되며, 부모 노드로 Running 상태를 반환한다. 
이 구조는 트리 전체가 Running 상태로 유지되더라도, 작업의 진행 상황을 지속적으로 모니터링하고 업데이트할 수 있도록 한다.

![](https://velog.velcdn.com/images/nnoa/post/a3b5c081-be8a-4353-8e52-ecbd478e5ff4/image.png)


<br>

## 3. Node


### 3.1. Fallback (Selector)
자식 노드를 순차적으로 실행한다. 자식 중 하나라도 성공하면 전체 노드가 성공으로 간주된다. 모든 자식이 실패히면 실패로 간주한다.

- 첫 번째 자식 노드를 tick하기 전에 Fallback은 Running 상태가 된다.
- 자식 노드가 Failure를 반환하면, Fallback은 다음 자식을 tick한다.
- 마지막 자식 노드까지 Failure를 반환하면, 모든 자식 노드가 중단되고 Fallback은 Failure를 반환한다.
- 자식 노드 중 하나가 Success를 반환하면, Fallback이 중단되고 Success를 반환한다.
- 자식 노드가 Running을 반환하면, 다음과 같다.


| Type of ControlNode    | Child returns RUNNING |
|-------------------------|-----------------------|
| Fallback               | Tick again           |
| ReactiveFallback       | Restart              |

<br>

**Restart** : 전체 Fallback이 첫 번째 자식부터 다시 시작된다.
**Tick again** : 실패하지 않은 자식 노드부터 순차적으로 tick을 계속 실행한다. 이미 Failure를 반환한 이전 자식들은 다시 tick하지 않는다.

<br>

#### Fallback

![](https://velog.velcdn.com/images/nnoa/post/628e82b3-fad6-41e0-bc73-245b7ca83e8e/image.png)

<br>

#### ReactiveFallback

ReactiveFallback 노드는 이전 condition의 상태가 Failure에서 Success로 변경되는 경우 비동기 자식을 중단(interrupt)하도록 할 때 사용한다.

아래 예시는 최대 8시간까지 쉬어야 하는 오브젝트에 대한 트리이다.

![](https://velog.velcdn.com/images/nnoa/post/c7d7d43f-3dbd-478f-ac05-39aa1b8fa617/image.png)

노드 ```areYouRested?```가 Success를 반환하면 비동기 노드 ```Timeout (8hrs)```과 ```Sleep```가 중단된다.

<br>

### 3.2. Sequence
자식 노드를 순차적으로 실행한다. 자식 중 하나라도 실패하면 전체 노드가 실패로 간주된다. 모든 자식이 성공해야 성공으로 간주한다.

- 첫 번째 자식 노드를 tick하기 전에 Fallback은 Running 상태가 된다.
- 자식 노드가 Success를 반환하면, Sequence는 다음 자식을 tick한다.
- 마지막 자식까지 Success를 반환하면, 모든 자식 노드가 중단되고 Sequence는 Success를 반환한다.
- 자식 노드 중 하나가 Failure를 반환하면, Sequence가 중단되고 Failure를 반환한다.
- 자식 노드가 Running을 반환하면, 다음과 같다.


| Type of ControlNode     | Child returns FAILURE | Child returns RUNNING |
|--------------------------|-----------------------|-----------------------|
| Sequence                | Restart              | Tick again           |
| ReactiveSequence        | Restart              | Restart              |
| SequenceWithMemory      | Tick again           | Tick again           |

<br>

**Restart** : 전체 Sequence가 첫 번째 자식부터 다시 시작된다.

**Tick again** : 실패하지 않은 자식 노드부터 순차적으로 tick을 계속 실행한다. 이미 Failure를 반환한 이전 자식들은 다시 tick하지 않는다.

<br>

#### Sequence
아래 트리는 게임에서 저격수의 행동을 기술한 예시이다.

![](https://velog.velcdn.com/images/nnoa/post/8c48b21b-68ed-4f66-b2f3-43d67918ae3f/image.png)

<br>

#### ReactiveSequence
ReactiveSequence 노드는 Condition을 지속적으로 확인하는 데 유용하다. 그러나 비동기 자식을 사용할 경우, 필요 이상으로 자주 tick되지 않는지 주의해야 한다.

![](https://velog.velcdn.com/images/nnoa/post/7a342449-df86-4b7e-875f-8775b9283059/image.png)

```ApproachEnermy```는 행동이 완료될 때까지 Running을 반환하는 비동기 Action이다.
Condition ```isEnermyVisible```은 여러번 호출되며, False를 반환하면 ```ApproachEnermy```가 중지된다.

또 다른 예로, ```isBatteryOK```는 매 tick마다 체크되어야 하므로 ReactiveSequence를 사용한다.

![](https://velog.velcdn.com/images/nnoa/post/fe29bf89-318a-4df2-9c74-c41dff005dd6/image.png)

<br>

#### SequenceWithMemory
SequenceWithMemory 노드는 이미 Success를 반환한 자식 노드를 다시 tick하지 않도록 할 때 사용한다.

아래 예시는 위치 A, B, C를 한 번만 방문해야 하는 순찰 로봇의 BT이다.

![](https://velog.velcdn.com/images/nnoa/post/0d3e0f5b-2562-48c5-a0de-0befaf49bb93/image.png)

Action ```GoTo(B)```가 실패해도 ```GoTo(A)```는 다시 tick되지 않는다.

<br>

### 3.3. Decorator Node
Decorator 노드는 한 노드를 감싸서 실행 조건을 추가하거나 결과를 변경한다.
예를 들어, 특정 조건에서만 자식 노드를 실행하거나 성공/실패 결과를 반대로 바꾸는 역할을 수행한다.

<br>

### 3.4. Leaf Node
- Action Node: 구체적인 동작을 수행 (ex. 로봇 팔 움직이기)
- Condition Node: 특정 조건을 평가 (ex. 센서 값 확인)

<br>

### Reference
[1] [https://www.behaviortree.dev/docs/intro/](https://www.behaviortree.dev/docs/intro/) <br>
[2] [https://engineering.linecorp.com/ko/blog/behavior-tree](https://engineering.linecorp.com/ko/blog/behavior-tree)

<br>