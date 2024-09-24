import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider, QPushButton, QHBoxLayout, QGridLayout
from PyQt5.QtCore import Qt
import serial

class MotorControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.serial_port = serial.Serial('COM8', 115200)  # 아두이노의 시리얼 포트 설정

    def initUI(self):
        layout = QGridLayout()

        self.motor_labels = []
        self.motor_sliders = []

        # 각 모터에 대해 슬라이더와 버튼 생성
        motors = ['A', 'B', 'C', 'D']
        for i, motor in enumerate(motors):
            # 모터 라벨
            motor_label = QLabel(f'모터 {motor} 속도: 0%', self)
            self.motor_labels.append(motor_label)
            layout.addWidget(motor_label, i, 0)

            # 모터 슬라이더
            motor_slider = QSlider(Qt.Horizontal, self)
            motor_slider.setRange(0, 255)  # PWM 값 범위
            motor_slider.setValue(0)
            motor_slider.valueChanged.connect(lambda value, m=motor, l=motor_label: self.updateLabel(value, m, l))
            self.motor_sliders.append(motor_slider)
            layout.addWidget(motor_slider, i, 1)

            # 전진 버튼
            forward_button = QPushButton(f'모터 {motor} 전진', self)
            forward_button.clicked.connect(lambda _, m=motor, s=motor_slider: self.moveMotor(m, s.value(), 'f'))
            layout.addWidget(forward_button, i, 2)

            # 후진 버튼
            backward_button = QPushButton(f'모터 {motor} 후진', self)
            backward_button.clicked.connect(lambda _, m=motor, s=motor_slider: self.moveMotor(m, s.value(), 'b'))
            layout.addWidget(backward_button, i, 3)

            # 정지 버튼
            stop_button = QPushButton(f'모터 {motor} 정지', self)
            stop_button.clicked.connect(lambda _, m=motor: self.stopMotor(m))
            layout.addWidget(stop_button, i, 4)

        # 전체 정지 버튼
        stop_all_button = QPushButton('전체 정지', self)
        stop_all_button.clicked.connect(self.stopAllMotors)
        layout.addWidget(stop_all_button, len(motors), 0, 1, 5)  # 전체 정지 버튼은 그리드의 한 줄을 차지하게 설정

        self.setLayout(layout)
        self.setWindowTitle('모터 속도 제어')
        self.show()

    def updateLabel(self, value, motor, label):
        label.setText(f'모터 {motor} 속도: {value / 255 * 100:.1f}%')

    def moveMotor(self, motor, speed, direction):
        command = f'{direction}{motor}{speed}\n'
        self.serial_port.write(command.encode())  # 전진 또는 후진 명령어와 속도를 전송
        print(f"{direction.capitalize()} command sent to Motor {motor}: {command.strip()}")

    def stopMotor(self, motor):
        command = f's{motor}\n'
        self.serial_port.write(command.encode())  # 정지 명령어 전송
        print(f"Stop command sent to Motor {motor}")

    def stopAllMotors(self):
        # 모든 모터에 대해 정지 명령을 전송
        motors = ['A', 'B', 'C', 'D']
        for motor in motors:
            command = f's{motor}\n'
            self.serial_port.write(command.encode())
            print(f"Stop command sent to Motor {motor}")
        
    def closeEvent(self, event):
        self.serial_port.close()  # 창을 닫을 때 시리얼 포트 닫기

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MotorControlApp()
    sys.exit(app.exec_())