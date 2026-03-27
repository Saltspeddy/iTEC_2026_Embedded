#include "../Inc/HCSR04.h"
#include "main.h"

// private to this source file / library, only accessible through the getter/setter
// HCSR04_GetDistance, HCSR04_Reset
static HCSR04_t sensors[3];

HCSR04_t* getSensorByChannel(uint32_t channel)
{
    switch(channel)
    {
        case HAL_TIM_ACTIVE_CHANNEL_2: return &sensors[CENTER];
        case HAL_TIM_ACTIVE_CHANNEL_3: return &sensors[LEFT];
        case HAL_TIM_ACTIVE_CHANNEL_4: return &sensors[RIGHT];
        default: return NULL;
    }
}

void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
  uint32_t cycles = (SystemCoreClock / 1000000) * us;
  uint32_t start = DWT->CYCCNT;

  while ((DWT->CYCCNT - start) < cycles)
    ;
}

void HCSR04_Trigger(ultrasonicDir_t currDir)
{
  // select the ultrasonic sensor to trigger
  GPIO_TypeDef* Ultrasound_TRIG_GPIO_Port;
  uint16_t Ultrasound_TRIG_Pin;
  switch (currDir) {
    case CENTER:
      Ultrasound_TRIG_GPIO_Port = UltraS_Center_Trig_GPIO_Port;
      Ultrasound_TRIG_Pin = UltraS_Center_Trig_Pin;
      break;
    case LEFT:
      Ultrasound_TRIG_GPIO_Port = UltraS_Left_Trig_GPIO_Port;
      Ultrasound_TRIG_Pin = UltraS_Left_Trig_Pin;
      break;
    case RIGHT:
      Ultrasound_TRIG_GPIO_Port = UltraS_Right_Trig_GPIO_Port;
      Ultrasound_TRIG_Pin = UltraS_Right_Trig_Pin;
      break;
    default:
      return;
  }

  HAL_GPIO_WritePin(Ultrasound_TRIG_GPIO_Port, Ultrasound_TRIG_Pin, GPIO_PIN_RESET);
  delay_us(10);

  HAL_GPIO_WritePin(Ultrasound_TRIG_GPIO_Port , Ultrasound_TRIG_Pin, GPIO_PIN_SET);
  delay_us(10);

  HAL_GPIO_WritePin(Ultrasound_TRIG_GPIO_Port, Ultrasound_TRIG_Pin, GPIO_PIN_RESET);
}

void HCSR04_ProcessEcho(TIM_HandleTypeDef *htim)
{
  HCSR04_t* sensor = getSensorByChannel(htim->Channel);
  if (sensor == NULL)
    return;

  uint32_t channel;

  switch(htim->Channel)
  {
    case HAL_TIM_ACTIVE_CHANNEL_2:
      channel = TIM_CHANNEL_2;
      break;
    case HAL_TIM_ACTIVE_CHANNEL_3:
      channel = TIM_CHANNEL_3;
      break;
    case HAL_TIM_ACTIVE_CHANNEL_4:
      channel = TIM_CHANNEL_4;
      break;
    default:
      return;
  }

  if (sensor->Is_First_Captured == 0)
  {
    sensor->IC_Val1 = HAL_TIM_ReadCapturedValue(htim, channel);
    sensor->Is_First_Captured = 1;

    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING);
  }
  else if (sensor->Is_First_Captured == 1)
  {
    sensor->IC_Val2 = HAL_TIM_ReadCapturedValue(htim, channel);

    if (sensor->IC_Val2 > sensor->IC_Val1)
      sensor->Difference = sensor->IC_Val2 - sensor->IC_Val1;
    else
      sensor->Difference = (0xffffffff - sensor->IC_Val1) + sensor->IC_Val2;

    sensor->Distance = sensor->Difference * 0.034 / 2.0;

    sensor->Is_First_Captured = 0;

    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);
  }
}

float HCSR04_GetDistance(ultrasonicDir_t dir)
{
  if (dir >= 3 || dir < 0)
    return 999;
  return sensors[dir].Distance;
}

void HCSR04_Reset(ultrasonicDir_t dir)
{
  if (dir >= 3 || dir < 0)
    return;
  sensors[dir].Is_First_Captured = 0;
  sensors[dir].Distance = 999;
}