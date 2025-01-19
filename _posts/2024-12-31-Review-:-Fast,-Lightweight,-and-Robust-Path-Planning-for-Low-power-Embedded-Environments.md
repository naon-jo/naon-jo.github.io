---
title: "[Review] Fast, Lightweight, and Robust Path Planning for Low-power Embedded Environments "
description: "저전력 임베디드 환경에서의 빠르고 가볍고 로버스트한 경로 계획 알고리즘"
date: 2024-12-31 18:00:00 +0800
categories: [Study, Planning and Control]
tags: [path planning, navigation, low-power system, embedded system]
pin: true
math: true
mermaid: true
---

<br>

Authors : Hwanhee Lee, Ilyong Yoon, Sangwoo Kim, and Dong Jin Hyun<br>
Journal : 2021 The 21st International Conference on Control, Automation and Systems [(ICCAS 2021)](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11024490)<br>
Publisher : ICROS<br>
Published : 2021.10<br>

---

<br>

## Abstract 

저전력 임베디드 환경에서 안전한 로봇 내비게이션을 위한 빠르고 가벼우면서도 로버스트한 경로 계획 알고리즘을 제안한다.
제안하는 방법은 다음과 같은 단계로 구성된다.

1. 대략적이고 빠르게 초기 경로를 찾는 단계
2. 초기 경로를 로버스트하게 단축하는 단계

실험 결과, 제안한 알고리즘은 평균 반복 횟수와 평균 계산 시간, 평균 경로 길이 모두 RRT\*, Informed RRT\*보다 나은 성능을 보이며 저전력 임베디드에서 허용 가능한 수준을 보였다.


## 1. Introduction
#### Proposal

- propose a fast, lightweight, and robust path planning algorithm for robot navigation environments varying every moments 
- verify its feasibility for lowpower embedded environments 

매순간 변화하는 로봇 내비게이션 환경에 대해 빠르고 가벼우면서도 로버스트한 경로 계획 알고리즘을 제안하고, 저전력 임베디드 환경에서의 실현 가능성을 검증한다.


#### Main objective

- find path immediately avoiding obstacles in unstructured environments for safe robot navigation on low-power embedded environments. 

저전력 임베디드 환경에서 안전한 로봇 내비게이션을 위해 비정형 환경에서 장애물을 즉시 회피하는 경로를 찾는다.


#### Several Experiments
- evaluate the path planning performance and the long-term operation 
- verify the consistency of performance and the feasibility for low-power embedded environments

경로 계획 성능과 장기 동작을 평가하고 저전력 임베디드 환경에서의 성능 일관성 및 실현 가능성을 검증하기 위해 여러 실험을 수행한다.

<br>

## 2. Related Works

### 2.1. graph-based method
#### Basic concept
Graph-based methods are graph traversal and path search algorithms, which discretize the state space and construct a graph of the discretized space and then find a path from a start state to an end state of the graph by checking the states. 

상태 공간을 이산화 &rarr; 이산화된 공간의 그래프를 구성 &rarr; 상태를 확인하여 그래프의 시작 상태에서 끝 상태까지의 경로를 찾는다.


#### representative algoritms
- Dijkstra’s algorithm
- A* Algorithm


<br>

### 2.2. sampling-based method
#### Basic concept
Sampling-based methods take random samples in the state space not to construct a graph structure of discretized state space

이산화된 상태 공간의 그래프 구조를 구성하지 않고 상태 공간에서 무작위 샘플을 취한다.
상태 공간을 이산화 &rarr; 무작위 샘플 취득 &rarr; 경로 탐색

#### representative algoritms
- Probabilistic Roadmap (PRMs)
- Rapidly-exploring Random Trees (RRTs)

<br>

### 3. Strategy of the proposed path planning algorithm

### 3.1. consists
&emsp; phase 1) roughly and quickly finding an initial path

&emsp; phase 2) robustly shortening the initial path

제안하는 방법은 샘플링 기반 방법의 변형으로, 

&emsp; 1. 대략적이고 빠르게 초기 경로를 찾는 단계

&emsp; 2. 초기 경로를 강력하게 단축하는 단계

로 구성된다.


<br>

### 3.2. The overall procedure


![](https://velog.velcdn.com/images/nnoa/post/7b90ed29-958d-4d4f-8823-5eafc968abab/image.png)

$$X$$는 상태공간이고, 경계가 있다고 가정한다.

$$x_s$$를 초기 상태, $$x_g$$를 목표 상태로 정의한다.

&emsp;$$x_s$$ ∈ $$X$$ : start state  <br>

&emsp;$$x_g$$ ∈ $$X$$ : goal state <br>

<br>

$$x_s$$와 $$x_g$$가 주어지면, 주어진 start state와 goal state를 root state로 하는 두 개의 independent graph structure가 초기화된다.

&emsp;2: &emsp;$$G_s.init(x_s)$$

&emsp;3: &emsp;$$G_g.init(x_g)$$

<br>

$$K$$번의 반복 횟수 내에서 7~10 단계를 반복한다.


focused sampling 영역에서 상태 $$x_s$$ ∈ $$X$$를 샘플링한다. 샘플링 영역은 상태 공간 $$X$$의 하위 집합이며, $$x_s$$와 $$x_g$$로부터 계산된다.

&emsp;7: &emsp; $$x_s$$ &larr; $$SampleState(X)$$

<br>

**대략적이고 빠르게 초기 경로를 탐색**

초기 경로 탐색을 위한 반복 횟수를 줄이기 위해 두 개의 그래프 구조 $$G_s$$와 $$G_g$$를 독립적으로, 매우 대략적으로 구축한다. 

&emsp;8: &emsp; **if** $$Build(X, G_s, G_g, x_s)$$ **then**

<br>

초기 경로를 찾기 위한 그래프 $$G_s$$와 $$G_g$$의 구축이 완료되면 초기 경로 $$P_i$$를 찾는다.

&emsp;9: &emsp; $$P_i$$ &larr; $$findInitialPath(G_s, G_g)$$

<br>

**로버스트한 최종 경로로 최적화**

ShortenPath 방법으로 초기 경로를 최적화한다. ShortenPath는 충돌 검사와 보간으로 구성된 일종의 후처리 프로세스다.

&emsp;10: &emsp; $$P_i$$ &larr; $$ShortenPath(X, G_s, G_g, P_i)$$

### 3.3. Main concepts
- generate an rough initial path as quickly as possible
- optimize it to a robust final path by using a post-processing method

위 프로세스의 주요 개념은 **대략적인 초기 경로를 최대한 빨리 생성**하고 후처리 방법을 사용하여 **로버스트한 최종 경로로 최적화**하는 것이다.


이러한 개념으로 인해 
- 적은 반복 횟수 내에서 초기 경로를 매우 빠르게 찾을 수 있으며, 
- 최종 경로 탐색을 위한 최적화 과정이 sampling domain에 포함되지 않기 때문에 반복 횟수에 의존할 필요가 없다는 장점이 있다.

<br>

## 4. Experimental results

제안된 알고리즘은 복잡한 시나리오 3에서 성공률이 95.6% 및 97.2%로, 저전력 임베디드 환경에서도 강인한 경로 계획 성능을 보였다.

[Full PDF](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11024490)<br>
<br>

## 5. Conclusions and Future Works

매순간 변화하는 로봇 내비게이션 환경에서 빠르고 가볍고 로버스트한 경로 계획 알고리즘을 제안하고 다양한 실험을 통해 저전력 임베디드 시스템에서의 실현 가능성을 검증했다. 제안된 방법은 샘플링 영역에서 반복 횟수에 의존하지 않는 자체 후처리 기법을 통해 대략적인 초기 경로를 최대한 빠르게 찾고 이 초기 경로를 최적화한다. 따라서 로봇 내비게이션 환경에서 즉각적인 경로 계획에 활용하기에 적합하다.

<br>


