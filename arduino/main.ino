#include <Servo.h>
Servo servo;
Servo motor;
int servo_pin = 9;
int motor_pin = 8;
int natural_pos = 90;
int servo_pos = natural_pos;
int motor_pos = natural_pos;
const int buffer_size = 10;
char buffer[buffer_size];

void setup() {
  Serial.begin(9600);
  servo.attach(servo_pin);  // attaches the servo on pin 9 to the servo object
  motor.attach(motor_pin);  // attaches the servo on pin 9 to the servo object
  servo.write(natural_pos);
  motor.write(natural_pos);
}

void loop() {
  if (Serial.available() > 0) {
    Serial.readBytes(buffer, buffer_size);
    const char s[2] = ",";
    char* pos1 = strtok(buffer, s);
    char* pos2 = strtok(NULL, s);
    if (pos1 != NULL and pos2 != NULL){
      int new_servo_pos = atoi(pos1);
      int new_motor_pos = atoi(pos2);
      int servo_diff = abs(new_servo_pos - servo_pos); 
      int motor_diff = abs(new_motor_pos - motor_pos); 
      int servo_step_sign = (new_servo_pos > servo_pos)? 1 : -1;
      int motor_step_sign = (new_motor_pos > motor_pos)? 1 : -1;
      while (servo_diff > 0 or motor_diff > 0){
        servo_pos += (servo_diff > 0)? min(10, servo_diff) * servo_step_sign : 0;
        motor_pos += (motor_diff > 0)? min(10, motor_diff) * motor_step_sign : 0;
        servo_diff = abs(new_servo_pos - servo_pos);
        motor_diff = abs(new_motor_pos - motor_pos); 
        servo.write(servo_pos);
        motor.write(motor_pos);
        delay(15);
      }
    }
  }
  servo.write(servo_pos);
  motor.write(motor_pos);
  delay(15);
}
