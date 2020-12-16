#include <Servo.h>

#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.
float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// Framework setting
#define _DIST_TARGET 255 
#define _DIST_MIN 100 
#define _DIST_MAX 410 

// Servo speed control
#define _SERVO_ANGLE 30 
#define _SERVO_SPEED 500

// configurable parameters
#define _DUTY_MIN 1320 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1550  // servo neutral position (90 degree)
#define _DUTY_MAX 1800  // servo full counterclockwise position (180 degree)

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;
#define _INTERVAL_DIST 20  // [3074] 거리측정주기 (ms)
#define _INTERVAL_SERVO 20 // [3078] 서보제어주기 (ms)
#define _INTERVAL_SERIAL 100 // [3078] Serial제어주기 (ms)

// PID parameters
#define _KP 1.5
#define _KI 0.001
#define _KD 70

// global variables
float dist_raw, dist_cali; // unit: mm
int a, b; // unit: mm

// servo instance
Servo myservo;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  myservo.attach(PIN_SERVO); 

  duty_curr =_DUTY_NEU;
  myservo.writeMicroseconds(_DUTY_NEU);

  error_prev = 0.0;
  
// initialize serial port
  Serial.begin(57600);

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (_INTERVAL_SERVO / 1000.0); // [3074] 서보 업데이트 1주기에 증감 가능한 duty 의 최댓값

  // [1615] 마지막 이벤트 발생 시간 초기화
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = false;
  event_servo = false;
  event_serial = false;

  a = 69;
  b = 323;

  iterm = 0;
}

float ir_distance(void){ // 거리측정함수 return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}

void loop() {
  // [1615] 거리 측정 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST)
        event_dist = true;
    
  // [1615] 서보 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO)
      event_servo= true;
    
  // [1615] Serial 제어 주기가 되었는지 검사 
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL)
      event_serial= true;

  //거리
  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
      dist_raw = filtered_ir_distance();
  
  //보정값들 연산
    dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);

  // PID control logic
    error_curr = _DIST_TARGET - dist_cali;// [3073] 현재 읽어들인 데이터와 기준 값의 차이
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm;
    error_prev = error_curr;


  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;
    
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  duty_target = min(max(duty_target, _DUTY_MIN), _DUTY_MAX); // [1615]
  
  // [1615] 마지막 샘플링 시각 업데이트
  last_sampling_time_dist += _INTERVAL_DIST;
  }

  
  //서보
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    float movement = (duty_curr - duty_target);
    if(abs(movement) > duty_chg_per_interval && movement < 0){
      duty_curr += duty_chg_per_interval;   
    }
    else if (abs(movement) > duty_chg_per_interval && movement > 0){
      duty_curr -= duty_chg_per_interval;
    }
    else{
      duty_curr = duty_target;
    }
    
    // update servo position
    myservo.writeMicroseconds(duty_curr); // 

    // [1615] 마지막 서보 조작 시각 업데이트
    last_sampling_time_servo += _INTERVAL_SERVO;
  }

  
  //시리얼
  if(event_serial) {
    event_serial = false;
     Serial.print("IR:");
     Serial.print(dist_cali);
     Serial.print(",T:");
     Serial.print(_DIST_TARGET);
     Serial.print(",P:");
     Serial.print(map(pterm,-1000,1000,510,610));
     Serial.print(",D:");
     Serial.print(map(dterm,-1000,1000,510,610));
     Serial.print(",I:");
     Serial.print(map(iterm,-1000,1000,510,610));
     Serial.print(",DTT:");
     Serial.print(map(duty_target,1000,2000,410,510));
     Serial.print(",DTC:");
     Serial.print(map(duty_curr,1000,2000,410,510));
     Serial.println(",-G:245,+G:265,m:0,M:800");

    // [1615] 마지막 Serial 업데이트 시각 업데이트
    last_sampling_time_serial += _INTERVAL_SERIAL;
  }
}
