# KG_Team2
KG 카이로스 2팀 side project

<각도에 따라 움직이는 방향범위 설정><br>
0 ~ 83 : go left<br>
83 ~ 95 : go straight<br>
95 ~ 180 : go right<br>

<코드 설명><br>
udp_cam_cv_serial(pi_client).py<br>
->라즈베리파이에서 카메라로 찍은 영상 데이터를 서버(PC)로 송신<br>

udp_revice_pi_opencv(pc_server).py<br>
->송신된 데이터를 수신후 차선의 각도를 계산해서 라즈베리파이로 수신<br>

encoder_motor_controll.ino<br>
->각도에 따른 모터 출력값 조절<br>