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
2. run ```$rosrun knu_ros_lecture team3_project_color```

### turtlebot
1. run ```$roslaunch raspicam_node camerav2_410x308_30fps.launch```
2. run ```$roslaunch turtlebot3_bringup turtlebot3_robot.launch```

## 0617
0. ```team3_project_color.cpp```로 합치면서 ```CMakeList.xml```도 수정하였음
1. ```초록색 : 직진```
2. ```빨간색 : 정지```
3. ```노란색 : 좌회전```
4. ```파란색 : 우회전 으로 바꿔놓음```
5. ```핑크색 : 저속주행
