#ifndef _SDCARD_PINS_H_
#define _SDCARD_PINS_H_ 1

#if defined(CAMERA_MODEL_WROVER_KIT)

#elif defined(CAMERA_MODEL_M5STACK_CAMS3_UNIT_5MP)
#define SD_CLK_GPIO_NUM 39
#define SD_MISO_GPIO_NUM 40
#define SD_MOSI_GPIO_NUM 38
#define SD_CS_GPIO_NUM 9

#define LED_GPIO_NUM 14

#else
#error "Camera model not selected"
#endif

#endif  // _SDCARD_PINS_H_