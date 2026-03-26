#ifndef ULTRASONIC_3_SENSORS_BLUETOOTH_CONTROL_ULTRASONIC_SENSORS_H
#define ULTRASONIC_3_SENSORS_BLUETOOTH_CONTROL_ULTRASONIC_SENSORS_H
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum ultrasonic_sensor_direction {
  CENTER = 0,
  LEFT = 1,
  RIGHT = 2
}ultrasonicDir;

typedef struct {
  uint32_t IC_Val1;
  uint32_t IC_Val2;
  uint32_t Difference; // (cm)
  uint8_t Is_First_Captured;
  float Distance; // (cm)
} HCSR04_t;

inline HCSR04_t* getSensorByChannel(uint32_t channel);
inline void DWT_Init(void);
inline void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif //ULTRASONIC_3_SENSORS_BLUETOOTH_CONTROL_ULTRASONIC_SENSORS_H