# mars_mapping_cobot_system

# Overview

프로젝트 목적:  Gazebo를 이용해 미지의 넓은 공간을 3D 매핑하는 다중 로봇 시뮬레이션

사용 기술 스택: ROS2 Humble, Gazeobo11, Rviz2

시뮬레이션 환경: 드론 자율주행 로봇

# Installation

rover
https://github.com/mgonzs13/ros2_rover.git
```
cd ~/git && git clone git@github.com:NovoG93/sjtu_drone.git -b ros2
cd ~/ros2_ws/src && ln -s ~/git/sjtu_drone
cd .. && rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && colcon build --packages-select-regex sjtu*
```
drone
https://github.com/noshluk2/sjtu_drone.git
```
cd ~/git && git clone git@github.com:NovoG93/sjtu_drone.git -b ros2
cd ~/ros2_ws/src && ln -s ~/git/sjtu_drone
cd .. && rosdep install -r -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && colcon build --packages-select-regex sjtu*
```
필수 의존성 목록

추가 설정이 필요한 환경 변수나 구성 파일

# 주요 기능 (Features)
수행가능동작
드론
  1.사용자가 인터페이스를 통해 xyz 형태의 좌표를 입력하면 드론이 출발점으로부터 x,y,z 좌표로 이동
  
  2.h 를 누르면 드론이 로버의 캐리어로 복귀
  
  3.

    시뮬레이션 내에서 수행할 수 있는 동작 (예: 네비게이션, 장애물 회피, 조작 등)
    주요 스크립트 및 노드 역할 설명

 # 시스템 아키텍처 (Architecture)
 rqt 
    노드 간 통신 구조 (예: ROS2 노드 다이어그램)
    메시지, 서비스, 액션 인터페이스 설명
    데이터 흐름 개요

6. 데모 및 예제 (Demo & Examples)

    실행 예제 GIF 또는 스크린샷
    ROS2 launch 파일 실행 예시
    주요 기능 테스트 방법

7. 개발 및 기여 가이드 (Development & Contribution) (필요한 경우)

    코드 스타일 가이드 (예: PEP8)
    새로운 기능 추가 시 고려할 사항
    Pull Request 및 Issue 템플릿 (필요하면 .github 폴더에 추가)

8. 참고자료 및 라이선스 (References & License)

    관련 논문, 연구 자료, 문서 링크
    사용된 오픈소스 라이브러리 목록
    프로젝트 라이선스 (MIT, Apache 2.0 등)

   멀티스레딩 배웠잖으니 그냥 한잔하게됨..
