 ## EE405 전자 디자인 랩: 지능형 자율주행차 구현 (Fall 2023)

Jetson 보드에서 작동할 수 있게 컴퓨터 비전 기술 및 ROS package를 활용하여 자율주행 시스템에 대한 내용을 담음.
실제 작동하려면 makefile 

**주요 기술 및 기능:**

* **실시간 깊이 이미지 획득 및 Point Cloud 생성:**
    * ORB-SLAM (Oriented FAST and Rotated BRIEF Simultaneous Localization and Mapping) 알고리즘을 Jetson 보드에서 실행하여 실시간으로 주변 환경에 대한 depth image 획득.
    * 깊이 이미지(Depth Image)를 입력받아 3D Point Cloud를 생성하고, 0.08m 크기의 Voxel Grid 필터를 적용해 데이터를 downsampling 하고, Pass-through 필터를 적용하여 z축의 필요없는 데이터를 제거함.
* **Local Costmap 생성:**
    * Point Cloud 데이터를 기반으로 차량 주변의 장애물 정보를 2D 그리드 형태로 표현하는 Local Costmap을 생성함.
    * Amir Darwesh의 코드 사용함 - License: Modified BSD Software License (저작권자 표기시 사용가능)
* **Motion Primitive 기반 자율주행 알고리즘 개발:**
    * 차량의 움직임을 정의하는 Motion Primitive를 기반으로 자율주행 알고리즘 개발.
    * Local Costmap 정보를 이용하여 현재 상황에 적합한 Motion Primitive를 선택하고, 차량의 경로 계획.
* **Darknet 기반 숫자 인식 및 분류:**
    * 경량화된 YOLOv4-tiny 모델을 사용하여 Jetson 보드에서 주행 중 실시간으로 숫자 인식하도록 만듦.
* **시스템 통합 및 자동화:**
    * 위에서 설명한 모든 기능을 쉘 스크립트를 통해 통합하고 자동 실행되도록 구성.
    * 이를 통해 자율주행 시스템을 손쉽게 구동하고 테스트 가능.

**테스트 영상:**

* **작동 테스트 1:** [https://youtu.be/8Pu4vNLR7Cg](https://youtu.be/8Pu4vNLR7Cg)
* **작동 테스트 2:** [https://youtu.be/YYSQ4h-jc6A](https://youtu.be/YYSQ4h-jc6A)

 
* License 
Hyunki Seong 조교님께서 제공해주신 템플릿 코드를 기반으로 작성 [BSD 2-Clause License](https://opensource.org/license/BSD-2-Clause)
* [https://github.com/Guri-cccc/EE405A_2023/tree/main/Week10](https://github.com/Guri-cccc/EE405A_2023/tree/main/Week10)
