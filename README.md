# KG_Team2!!!
KG 카이로스 2팀

<각도에 따라 움직이는 방향범위 설정><br>
0 ~ 83 : go left<br>
83 ~ 95 : go straight<br>
95 ~ 180 : go right<br>

<udp_cam폴더 안에 코드 설명><br>
udp_cam_cv_serial(pi_client).py<br>
->라즈베리파이에서 동영상 송신 코드<br>

udp_vision.py<br>
->udp_cam_cv_serial(pi_client).py코드에서 serial 통신 부분을 제거한 코드<br>

udp_revice_pi_opencv(pc_server).py<br>
->컴퓨터에서 실행 코드<br>
