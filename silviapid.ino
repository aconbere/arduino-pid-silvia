#include <dbg.h>
#include <rotary_encoder.h>
#include <pid.h>
#include <pid_time.h>

#define TEMP_PIN 0
#define RELAY_PIN 1
#define ROTARY_PIN_A 14
#define ROTARY_PIN_B 15

PID *pid;
RotaryEncoder *encoder;

static float desired_temp = 100.00;
static float max_temp = 110.00;
static float min_temp = 90.00;

unsigned int windowSize = 5000;
unsigned long windowStartTime;

float clamp(float input) {
  if (input < min_temp) {
    return min_temp;
  } else if (input > max_temp) {
    return max_temp;
  } else {
    return input;
  }
}

void setup () {
  Serial.begin (115200);
  Serial.println("Start");

  pid = PID_create(
    analogRead(TEMP_PIN),
    desired_temp, // desired temp
    300, // sample time in ms
    0,   // min output
    windowSize, // max output
    2,   // Kp
    5,   // Ki
    1    // Kd
  );

  RotaryEncoder_setup(ROTARY_PIN_A, ROTARY_PIN_B);

  encoder = RotaryEncoder_create(0x03);
  windowStartTime = millis();
}

void loop () {
  int8_t result = RotaryEncoder_read(encoder);
  if (result) {
    desired_temp = clamp(desired_temp + (result * 0.02));

    Serial.print("Temp: ");
    Serial.print(desired_temp, 2);
    Serial.print("\n");
    
    pid->setpoint = desired_temp;
  }

  if (PID_should_compute(pid)) {
    if((millis() - windowStartTime) > windowSize) {
      windowStartTime += windowSize;
    }
    
    Serial.print("WindowStartTimeL ");
    Serial.print(windowStartTime, DEC);
    Serial.print("\n");

    // double next = PID_next(pid, analogRead(TEMP_PIN));
    double next = PID_next(pid, 90);
    Serial.print("next ");
    Serial.print(next, 2);
    Serial.print("\n");

    if(next < millis() - windowStartTime) {
      Serial.print("ON\n");
      digitalWrite(RELAY_PIN, HIGH);
    } else {
      Serial.print("OFF\n");
      digitalWrite(RELAY_PIN, LOW);
    }
  
  }
}

