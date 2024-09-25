#include <Encoder.h>

// 엔코더 정의
Encoder encoderA(18, 31);  // 엔코더A - A: 핀 18, B: 핀 31
Encoder encoderB(19, 38);  // 엔코더B - A: 핀 19, B: 핀 38
Encoder encoderC(3, 49);   // 엔코더C - A: 핀 3,  B: 핀 49
Encoder encoderD(2, 55);   // 엔코더D - A: 핀 2,  B: 핀 55

// 모터 제어 핀 설정
const int motorA_DirectionPin1 = 34; // 뒤 왼쪽 바퀴 (A) 방향 제어 핀 1
const int motorA_DirectionPin2 = 35; // 뒤 왼쪽 바퀴 (A) 방향 제어 핀 2
const int motorA_PWM_Pin = 12;       // 뒤 왼쪽 바퀴 (A) PWM 핀

const int motorB_DirectionPin1 = 37; // 앞 왼쪽 바퀴 (B) 방향 제어 핀 1
const int motorB_DirectionPin2 = 36; // 앞 왼쪽 바퀴 (B) 방향 제어 핀 2
const int motorB_PWM_Pin = 8;        // 앞 왼쪽 바퀴 (B) PWM 핀

const int motorC_DirectionPin1 = 43; // 앞 오른쪽 바퀴 (C) 방향 제어 핀 1
const int motorC_DirectionPin2 = 42; // 앞 오른쪽 바퀴 (C) 방향 제어 핀 2
const int motorC_PWM_Pin = 6;        // 앞 오른쪽 바퀴 (C) PWM 핀

const int motorD_DirectionPin1 = 58; // 뒤 오른쪽 바퀴 (D) 방향 제어 핀 1
const int motorD_DirectionPin2 = 59; // 뒤 오른쪽 바퀴 (D) 방향 제어 핀 2
const int motorD_PWM_Pin = 5;        // 뒤 오른쪽 바퀴 (D) PWM 핀

int steeringAngle = 90; // 초기 중앙값 설정

void setup() {
  Serial.begin(9600);  // 시리얼 모니터 0번 포트
  Serial3.begin(9600); // 시리얼 3 포트 - 블루투스 모듈과 통신

  Serial.println("블루투스 모듈을 통한 모터 제어 시작:");
  Serial.println("초기 상태:");

  // 모터 핀을 출력으로 설정
  pinMode(motorA_DirectionPin1, OUTPUT);
  pinMode(motorA_DirectionPin2, OUTPUT);
  pinMode(motorA_PWM_Pin, OUTPUT);
  
  pinMode(motorB_DirectionPin1, OUTPUT);
  pinMode(motorB_DirectionPin2, OUTPUT);
  pinMode(motorB_PWM_Pin, OUTPUT);

  pinMode(motorC_DirectionPin1, OUTPUT);
  pinMode(motorC_DirectionPin2, OUTPUT);
  pinMode(motorC_PWM_Pin, OUTPUT);

  pinMode(motorD_DirectionPin1, OUTPUT);
  pinMode(motorD_DirectionPin2, OUTPUT);
  pinMode(motorD_PWM_Pin, OUTPUT);
}

void loop() {
  // 블루투스 모듈로부터 명령어 읽기
  if (Serial3.available() > 0) {
    char command = Serial3.read(); // 블루투스 모듈로부터 1바이트 읽기
    Serial.print("받은 명령: ");
    Serial.println(command);   // 받은 명령어 시리얼 모니터에 출력
    
    if (command == 's') {
      // 정지 명령
      Serial.println("정지");
      stopMotors();
    } else {
      // 각도 명령 처리
      if (command >= '0' && command <= '9') {
        steeringAngle = map(command - '0', 0, 9, 0, 180); // 0~9를 0~180도로 매핑
        Serial.print("Steering Angle: ");
        Serial.println(steeringAngle); // 받은 각도 값을 시리얼 모니터에 출력
        setMotorSpeedBasedOnAngle(steeringAngle); // 각도에 따라 모터 속도 조정
      }
    }
  }

  // 너무 자주 출력되지 않도록 딜레이 추가
  delay(100);
}

void setMotorSpeedBasedOnAngle(int angle) {
  int pwmValue = 128; // 기본 PWM 값

  if (angle >= 85 && angle <= 95) {
    // 정면으로 가는 경우
    setMotorStateA(HIGH, LOW, pwmValue);
    setMotorStateB(HIGH, LOW, pwmValue);
    setMotorStateC(LOW, HIGH, pwmValue);
    setMotorStateD(LOW, HIGH, pwmValue);
  } 
  
  else if (angle < 85) {
    // 왼쪽으로 회전
    setMotorStateA(LOW, HIGH, pwmValue);
    setMotorStateB(LOW, HIGH, pwmValue);
    setMotorStateC(LOW, HIGH, pwmValue - (85 - angle) * 0.5); // 오른쪽 바퀴 속도 감소
    setMotorStateD(LOW, HIGH, pwmValue - (85 - angle) * 0.5);
  } 

  else if (angle > 95) {
    // 오른쪽으로 회전
    setMotorStateA(HIGH, LOW, pwmValue - (angle - 92) * 0.5); // 왼쪽 바퀴 속도 감소
    setMotorStateB(HIGH, LOW, pwmValue - (angle - 92) * 0.5);
    setMotorStateC(HIGH, LOW, pwmValue);
    setMotorStateD(HIGH, LOW, pwmValue);
  }
}

void setMotorStateA(int dir1, int dir2, int pwmValue) {
  digitalWrite(motorA_DirectionPin1, dir1);
  digitalWrite(motorA_DirectionPin2, dir2);
  analogWrite(motorA_PWM_Pin, pwmValue);
}

void setMotorStateB(int dir1, int dir2, int pwmValue) {
  digitalWrite(motorB_DirectionPin1, dir1);
  digitalWrite(motorB_DirectionPin2, dir2);
  analogWrite(motorB_PWM_Pin, pwmValue);
}

void setMotorStateC(int dir1, int dir2, int pwmValue) {
  digitalWrite(motorC_DirectionPin1, dir1);
  digitalWrite(motorC_DirectionPin2, dir2);
  analogWrite(motorC_PWM_Pin, pwmValue);
}

void setMotorStateD(int dir1, int dir2, int pwmValue) {
  digitalWrite(motorD_DirectionPin1, dir1);
  digitalWrite(motorD_DirectionPin2, dir2);
  analogWrite(motorD_PWM_Pin, pwmValue);
}

void stopMotors() {
  // 모든 모터를 정지합니다
  analogWrite(motorA_PWM_Pin, 0);
  analogWrite(motorB_PWM_Pin, 0);
  analogWrite(motorC_PWM_Pin, 0);
  analogWrite(motorD_PWM_Pin, 0);
}
