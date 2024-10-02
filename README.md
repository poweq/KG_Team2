# KG_Team2
KG 카이로스 2팀

<각도에 따라 움직이는 방향범위 설정><br>
0 ~ 83 : go left<br>
83 ~ 95 : go straight<br>
95 ~ 180 : go right<br>

<차선 1개에서 인식되는 코드><br>
one_line_detection폴더 안에 넣어 놨습니다.<br>

<one_line_detection폴더 안에 코드 설명><br>
one_line_detection.py<br>
->중심이 되는 중앙선으로 부터 인신된 차선이 얼마나 떨어져 있는지 출력해주는 코드<br>

one_line_detection_curve.py<br>
->차선이 직진일때랑 곡선일 때 모두 인식되는 코드<br>

one_line_AutoLaneTracer.py<br>
->차선을 인식하고 중앙으로 올 수 있게 모터출력값을 보여주는 코드<br>

one_line_AutoLaneTracer_raspberry.py<br>
->화면에서 인식된 차선의 각도를 계산해서 출력해주는 코드<br>
->라즈베리파이에서 모터 출력은 안됨!!<br>

one_line_AutoLaneTracer_raspberry_1.py<br>
->라즈베리파이 코드 넣고 돌릴 수 있는 코드<br>

one_line_AutoLaneTracer_raspberry_2.py<br>
->one_line_AutoLaneTracer_raspberry_1.py수정 코드<br>
->처음 코드를 실행시키고 10초 이따가 모터 출력되는 코드 추가