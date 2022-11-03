# gigacha_3DObstacleDetector
+ velodyne PUCK 16채널을 이용한 3차원 객체 검출 알고리즘
+ ROS1 환경에서 실행

# 실행 방법
#### 참고 깃허브 실행 방법
<pre>
<code>
$ sudo apt install libpcl-dev    # pcl c++ 라이브러리 설치
$ cd    # /home/userID로 이동
$ git clone https://github.com/enginBozkurt/LidarObstacleDetection.git    # 깃허브 주소
$ cd LidarObstacleDetection    # 패키지로 이동
$ mkdir build && cd build    # c++ 빌드 폴더 생성
$ cmake ..    # cpp 패키지 빌드
$ make    # 실행파일 생성
$ ./environment    # 실행파일 실행
</code>
</pre>

#### 내 코드 실행 방법

<pre>
<code>
$ lm && slb
$ rosrun lidar3d_od obstacledetector3d
</code>
</pre>

# Dependencies
- pcl_conversions
- pcl_ros
- roscpp
- sensor_msgs
