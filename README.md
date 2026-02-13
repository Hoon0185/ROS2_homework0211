# ROS2_homework0211
터틀봇 시뮬레이터 제어 실습 과제

* 터미널 명렁어
  - 1번 터미널
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

colcon build --symlink-install
source install/setup.bash
source ~/.bashrc
  - 2번 터미널
ros2 run my_turtlebot_pkg turtlebot_move_and_gui

* 코드
turtlebot_move_and_gui.py
: 빌드 후 실행시 GUI 리모콘을 통해 터틀봇 조작 기능

<img width="231" height="368" alt="image" src="https://github.com/user-attachments/assets/2725b499-be1a-48bd-90b0-4395fe3b4b11" />

* topic
  1. 상하좌우를 통한 이동,회전 기능
  2. s 버튼을 통한 정지 기능

* service
  1. 터틀봇 시뮬레이션 초기화 요청

* action
  1. 간단한 액션 기능 구현 : 180도 회전 
