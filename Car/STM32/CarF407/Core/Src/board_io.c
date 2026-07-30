#include "car_mission.h"

#include "main.h"

#define USER_KEY_DEBOUNCE_MS 20U

/*
 * SkyStar onboard KEY is connected to PA0 with an external pull-down:
 * released is low and pressed is high. Non-blocking debounce keeps the
 * communication, line-following and speed-control tasks responsive.
 */
uint8_t CarMission_ReadStartButton(void)
{
  static GPIO_PinState previous_raw = GPIO_PIN_RESET;
  static uint32_t changed_at_ms = 0U;
  GPIO_PinState raw =
      HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin);
  uint32_t now_ms = HAL_GetTick();

  if (raw != previous_raw)
  {
    previous_raw = raw;
    changed_at_ms = now_ms;
  }

  if ((raw == GPIO_PIN_SET) &&
      ((now_ms - changed_at_ms) >= USER_KEY_DEBOUNCE_MS))
  {
    return 1U;
  }
  return 0U;
}

uint8_t CarMission_ReadStartButtonRaw(void)
{
  return (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) ==
          GPIO_PIN_SET) ? 1U : 0U;
}
