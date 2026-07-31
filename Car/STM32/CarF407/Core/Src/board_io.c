#include "car_mission.h"

#include "main.h"

#define USER_KEY_DEBOUNCE_MS 20U
#define BOARD_KEY_COUNT 4U

/*
 * The passive 1x4 key board shares GND. Each input uses the STM32 internal
 * pull-up, so released is high and pressed is low. H3 pins 5..8 on the car
 * baseboard are PE5, PE3, PC0 and PC2 respectively.
 */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t raw_pressed;
  uint8_t stable_pressed;
  uint32_t changed_at_ms;
} BoardKey;

enum
{
  BOARD_KEY_DROP = 0U,
  BOARD_KEY_LAND,
  BOARD_KEY_START,
  BOARD_KEY_RESET
};

static BoardKey board_keys[BOARD_KEY_COUNT] =
{
  {TASK_DROP_KEY_GPIO_Port, TASK_DROP_KEY_Pin, 0U, 0U, 0U},
  {TASK_LAND_KEY_GPIO_Port, TASK_LAND_KEY_Pin, 0U, 0U, 0U},
  {TASK_START_KEY_GPIO_Port, TASK_START_KEY_Pin, 0U, 0U, 0U},
  {TASK_RESET_KEY_GPIO_Port, TASK_RESET_KEY_Pin, 0U, 0U, 0U}
};

static uint8_t read_pressed(const BoardKey *key)
{
  return (HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_RESET) ?
         1U : 0U;
}

static uint8_t read_debounced(uint8_t index)
{
  BoardKey *key = &board_keys[index];
  uint8_t raw_pressed = read_pressed(key);
  uint32_t now_ms = HAL_GetTick();

  if (raw_pressed != key->raw_pressed)
  {
    key->raw_pressed = raw_pressed;
    key->changed_at_ms = now_ms;
  }

  if ((key->stable_pressed != key->raw_pressed) &&
      ((now_ms - key->changed_at_ms) >= USER_KEY_DEBOUNCE_MS))
  {
    key->stable_pressed = key->raw_pressed;
  }
  return key->stable_pressed;
}

void BoardKeys_Init(void)
{
  uint8_t index;
  uint32_t now_ms = HAL_GetTick();

  /* MX_GPIO_Init has already configured all four inputs from CarF407.ioc. */
  for (index = 0U; index < BOARD_KEY_COUNT; index++)
  {
    board_keys[index].raw_pressed = read_pressed(&board_keys[index]);
    board_keys[index].stable_pressed = board_keys[index].raw_pressed;
    board_keys[index].changed_at_ms = now_ms;
  }
}

uint8_t CarMission_ReadDropButton(void)
{
  return read_debounced(BOARD_KEY_DROP);
}

uint8_t CarMission_ReadLandButton(void)
{
  return read_debounced(BOARD_KEY_LAND);
}

uint8_t CarMission_ReadStartButton(void)
{
  return read_debounced(BOARD_KEY_START);
}

uint8_t CarMission_ReadResetButton(void)
{
  return read_debounced(BOARD_KEY_RESET);
}

uint8_t CarMission_ReadStartButtonRaw(void)
{
  return read_pressed(&board_keys[BOARD_KEY_START]);
}
