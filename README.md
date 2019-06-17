# MobileRobot_Team03

## Member
이태영
김민준
권용준
김도한

## 주제 
신호등을 인식하는 자율 주행 로봇 

## How to Run
### remotePC
1. run ```$roscore```
2. run ```$rosrun knu_ros_lecture turtle_recognize_color```

### turtlebot
1. run ```$roslaunch raspicam_node camerav2_410x308_30fps.launch```
2. run ```$roslaunch turtlebot3_bringup turtlebot3_robot.launch```

## 0617 from 민준
```초록색 : 직진```
```빨간색 : 정지```
```노란색 : 좌회전```
```파란색 : 우회전 으로 바꿔놓음```
*장애물탐지 제거하였음. 순전히 색만으로 회전, 이동 및 정지 가능한거 확인함. 아마 영상은 간단한 트랙을 색깔만으로 한바퀴 주행하도록 영상찍으면 될 것 같음
그리고 파란색 말고 다른색을 찾는게 좋을 것 같음. 파란색을 탐지하고 나면 HSV특성때문인지 화면이 노랗게 변함. 
