#ifndef ULTRASONIC_3_SENSORS_BLUETOOTH_CONTROL_ULTRASONIC_SENSORS_H
#define ULTRASONIC_3_SENSORS_BLUETOOTH_CONTROL_ULTRASONIC_SENSORS_H
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum ultrasonic_sensor_direction {
  CENTER = 0,
  LEFT = 1,
  RIGHT = 2
} ultrasonicDir_t;

typedef struct {
  uint32_t IC_Val1;
  uint32_t IC_Val2;
  uint32_t Difference; // (cm)
  uint8_t Is_First_Captured;
  float Distance; // (cm)
} HCSR04_t;

// extern HCSR04_t sensors[3];

HCSR04_t* getSensorByChannel(uint32_t channel);
void DWT_Init(void);
void HCSR04_ProcessEcho(TIM_HandleTypeDef *htim);
void HCSR04_Trigger(ultrasonicDir_t currDir);
float HCSR04_GetDistance(ultrasonicDir_t dir);
void HCSR04_Reset(ultrasonicDir_t dir);
void Ultrasonic_Update(void);

#endif //ULTRASONIC_3_SENSORS_BLUETOOTH_CONTROL_ULTRASONIC_SENSORS_H