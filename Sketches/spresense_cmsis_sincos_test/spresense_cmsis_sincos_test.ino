#include <math.h>

#define ARM_MATH_CM4
#define __FPU_PRESENT 1U
#include <cmsis/arm_math.h>

void setup() {
  Serial.begin(115200);
  uint32_t start_time, duration;
  start_time = micros();
  for (int x = 0; x < 360; ++x) {
    float radian = x * M_PI/180.0;
    float sin_y = sin(radian);
    float cos_y = cos(radian);
  }
  duration = micros() - start_time;
  Serial.println("math duration: " + String(duration));

  start_time = micros();
  for (int x = 0; x < 360; ++x) {
    float radian = x * M_PI/180.0;
    float sin_y = arm_sin_f32(radian);
    float cos_y = arm_cos_f32(radian);
  }
  duration = micros() - start_time;
  Serial.println("arm_math duration: " + String(duration));
}

void loop() {
  // put your main code here, to run repeatedly:

}
