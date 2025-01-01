---
title: "Vehicle Lateral Control : Pure Pursuit"
description: Pure pursuit controller
date: 2024-12-24 18:00:00 +0800
categories: [Study, Planning and Control]
tags: [autonomous driving, vehicle control, pure pursuit]
pin: true
math: true
mermaid: true
image:
    path: "https://velog.velcdn.com/images/nnoa/post/fcc2ec7a-aca2-4e5b-afb9-dccbdf448581/image.png"
---

<br>

횡방향 제어 이론에 대해 학습한다.<br>
&emsp;**- Pure Pursuit**<br>
&emsp;- Stanley<br>
&emsp;- MPC

<br>

## 1. Pure Pursuit
Pure pursuit은 차량의 후륜축의 중심을 기준점으로 하여, **차량의 운동방정식**과 **기준 경로의 geometry**만을 이용해 경로를 추종하는 알고리즘이다. <br>
운동학(기하학)만을 이용하기 때문에 차량의 미끄러짐과 같은 다이나믹한 특성은 무시된다. 고속에서 급선회 시 높은 횡방향 가속도가 발생하므로 Pure pursuit 방식과 맞지 않다. Pure pursuit을 위한 운동학 모델은 타이어의 비선형성이 없는 움직임, 예를 들어 저속에서 미끄러짐이 없는 움직임에 적절하다.

<br>

## 2. Pure pursuit의 구성요소

### 2.1 구성요소

#### Look-ahead Distance, 전방주시거리
차량의 뒷바퀴로부터 자동차가 가야 할 경로의 임의의 목표 지점(look-ahead point, 전방 목표점)까지의 거리  

#### 자동차 운동학: Bicycle 모델

<br>

### 2.2 파라미터
- $$L$$ : 자동차 앞바퀴와 뒷바퀴 사이의 거리 (고정값)  
- $$l_d$$ : 전방주시거리 (look-ahead point에 따라 변화하는 값)  
- $$\alpha$$ : 직선 $$l_d$$가 차체의 방향과 이루는 각도 (look-ahead point에 따라 변화하는 값)

![](https://velog.velcdn.com/images/nnoa/post/fcc2ec7a-aca2-4e5b-afb9-dccbdf448581/image.png)

<br>

## 3. Pure Pursuit의 원리
자동차의 선회반경 $$R$$을 결정한 후, Bicycle 모델을 통해 자동차가 가져야 할 바퀴 회전각을 결정한다.

#### (1) 자동차의 선회반경 R 결정
자동차의 뒷바퀴, 전방 목표점, 선회운동의 순간회전중심(IRC)으로 이루어진 삼각형에서, 선회운동의 중심점 측 각도는 $$2\alpha$$로 얻어진다.

![](https://velog.velcdn.com/images/nnoa/post/aa4e8956-2eb1-43aa-be0f-28230b917355/image.png)

선회반경 $$R$$ 과 $$\alpha$$, $$l_d$$의 관계를 삼각함수 공식을 통해 다음과 같은 수식으로 나타낼 수 있다.  
원의 곡률 $$\kappa$$는 반지름 $$R$$의 역수이므로, 삼각함수 공식을 적용하여 얻은 수식으로 곡률 $$\kappa$$ 를 정의할 수 있다.

$$
\frac{l_d}{\sin{2\alpha}} = \frac{R}{\sin{(\frac{\pi}{2}- \alpha)}}
$$

$$
\frac{l_d}{2 \sin{\alpha} \cos{\alpha}} = \frac{R}{\cos{\alpha}}
$$

$$
\frac{l_d}{\sin{\alpha}} = 2R
$$

$$
\kappa = \frac{1}{R} = \frac{2 \sin{\alpha}}{l_d}
$$

#### (2) 바퀴 회전각 δ 결정
앞바퀴와 뒷바퀴의 거리 $$L$$과 회전 반지름 $$R$$을 이용하여 조향각 $$\delta$$를 구하는 공식은 다음과 같다.

$$
\kappa = \frac{1}{R} = \frac{2sin{\alpha}}{l_d}
$$

$$
\delta = \tan^{-1} \frac{L}{R} = \tan^{-1} (\kappa L)
$$

$$
\delta = tan^{-1}(\frac{2L\sin{\alpha}}{l_d})
$$

이때 $$L$$은 고정값이고, $$\alpha$$와 $$l_d$$는 look-ahead point에 의해 변하는 값이다. 
따라서 조향각 $$\delta$$는 $$\alpha$$와 $$l_d$$에 의해 결정된다.

<br>

## 4. Pure Pursuit 제어기 설계 
센서로부터 전방주시거리의 오차를 획득하고, 원하는 경로로 주행하게 하는 제어기를 Pure pursuit 방식으로 설계한다.

#### 횡방향 오차 e
위 관계식을 오차의 관점에서 살펴보자. 횡방향 오차 $$e$$는 차량이 목표 경로에서부터 수직으로 떨어져 있는 거리로, 차량이 목표 경로를 주행할 때 줄여 나가고자 하는 오차를 의미한다. 이는 Pure pursuit 방식에서 전방주시거리 $$l_d$$에서의 오차다.

![](https://velog.velcdn.com/images/nnoa/post/9899a451-41ca-432f-9df3-6a15409ce70e/image.png)

삼각함수 공식에 의해 $$\alpha$$와 $$l_d$$로 표현하면

$$
\sin{\alpha} = \frac{e}{l_d}
$$

이고, 이 관계식을 자동차가 가져야 할 회전반경의 곡률 $$\kappa$$에 의해 나타내면

$$
\kappa = \frac{2}{l_d^2}e
$$

이다.
자동차의 조향각 $$\delta$$는 곡률 $$\kappa$$에 의해 결정되며, 이 곡률 $$\kappa$$는 오차 $$e$$에 상수값 $$\frac{2}{l_d ^2}$$이 곱해진 형태로 결정된다. 즉 **비례 제어기**가 설계된 것이며, 이 상수는 **비례 이득**이 된다.

<br>

전방주시거리 $$l_d$$는 제어기 설계 시 설정해주는 튜닝 파라미터로써 제어기의 성능을 결정한다.
- $$l_d$$가 큰 경우, 오차가 크더라도 목표 조향각은 작다.
- $$l_d$$가 작은 경우, 오차가 작더라도 목표 조향각은 크다.

$$l_d$$가 작다는 것은 전방주시거리가 짧다는 의미이고, 이는 운전자가 바로 앞을 보고 운전하는 것과 같다. 이 경우는 저속에서는 문제가 되지 않으나, 고속에서는 오차가 작더라도 목표 조향값이 크므로 위험하다.
이러한 현상을 완화하기 위해 전방주시거리 $$l_d$$를 속도에 비례하는 상수로 설정한다.

$$
l_d = K v_x
$$

- 비례상수 $$K$$ : 튜닝 파라미터
- 고속에서 큰 $$l_d$$값을 가지도록 함.
- 오차가 천천히 줄어들게 하여 안정적인 주행 가능

<br>
<br>

## References
[1] [Yan Ding, "Three Methods of Vehicle Lateral Control: Pure Pursuit, Stanley and MPC", Medium](https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081) <br>
[2] [MathWorks, "Pure Pursuit Controller"](https://www.mathworks.com/help/nav/ug/pure-pursuit-controller.html)

<br>