#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0

Servo mysv;
float alpha, dist_ema;

void setup() {
  mysv.attach(PIN_SERVO);
  mysv.writeMicroseconds(1450);  

  // initialize Serial Port
  Serial.begin(57600);

  // initialize veriable
  alpha = 0.1;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cal = 100.0 + 300.0/(300 - 70)*(raw_dist - 70);
  dist_ema = alpha*dist_cal + (1-alpha)*dist_ema;
  
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cal:");
  Serial.print(dist_cal);
  Serial.print(",dist_ema:");
  Serial.println(dist_ema+100);

  if(dist_ema < 270) mysv.writeMicroseconds(1740);
  else if(dist_ema > 270) mysv.writeMicroseconds(1240);
}
