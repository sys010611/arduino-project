#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
#define _DUTY_MIN 1350 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1550  // servo neutral position (90 degree)
#define _DUTY_MAX 1750  // servo full counterclockwise position (180 degree)

// global variables
float dist_raw, dist_prev, dist_ema, dist_prev_ema = 0, alpha = 0.3; // unit: mm
int a, b; // unit: mm
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

// initialize USS related variables
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  
// initialize serial port
  Serial.begin(57600);

  a = 68;
  b = 331;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  dist_raw = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
  dist_ema = alpha*dist_cali + (1-alpha)*dist_prev_ema;
  dist_prev_ema = dist_ema;
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(dist_raw);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.println(dist_ema);
  
  if (dist_ema < 255){
    myservo.writeMicroseconds(_DUTY_MAX);
  }
  else{
    myservo.writeMicroseconds(_DUTY_MIN);
  }
  delay(20);
}
