#ifndef _MIC_PINS_H_
#define _MIC_PINS_H_ 1

#if defined(CAMERA_MODEL_M5STACK_CAMS3_UNIT) || defined(CAMERA_MODEL_M5STACK_CAMS3_UNIT_5MP)
#define PDM_CLK_GPIO_NUM 47
#define PDM_DIN_GPIO_NUM 48

#define LED_GPIO_NUM 14

#else
#error "Camera model not selected"
#endif

#endif  // _MIC_PINS_H_