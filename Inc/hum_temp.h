#ifndef __HUM_TEMP_H
#define __HUM_TEMP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/**
  * @brief  Humidity and temperature init structure definition
  */
typedef struct
{
  uint8_t Power_Mode;                         /* Power-down/Sleep/Normal Mode */
  uint8_t Data_Update_Mode;                   /* continuous update/output registers not updated until MSB and LSB reading*/
  uint8_t Reboot_Mode;                        /* Normal Mode/Reboot memory content */
  uint8_t Humidity_Resolutin;                 /* Humidity Resolution */
  uint8_t Temperature_Resolution;             /* Temperature Resolution */
  uint8_t OutputDataRate;                     /* One-shot / 1Hz / 7 Hz / 12.5 Hz */
}HUM_TEMP_InitTypeDef;

/**
  * @brief  Humidity and temperature status enumerator definition
  */
typedef enum {
    HUM_TEMP_OK = 0,
    HUM_TEMP_ERROR = 1,
    HUM_TEMP_TIMEOUT = 2
} HUM_TEMP_StatusTypeDef;

/**
  * @brief  Humidity and temperature driver structure definition
  */
typedef struct
{
  HUM_TEMP_StatusTypeDef       (*Init)(HUM_TEMP_InitTypeDef *);
  HUM_TEMP_StatusTypeDef       (*PowerOFF)(void);
  HUM_TEMP_StatusTypeDef       (*ReadID)(uint8_t *);
  HUM_TEMP_StatusTypeDef       (*Reset)(void);
  void                         (*ConfigIT)(uint16_t);
  void                         (*EnableIT)(uint8_t);
  void                         (*DisableIT)(uint8_t);
  uint8_t                      (*ITStatus)(uint16_t, uint16_t);
  void                         (*ClearIT)(uint16_t, uint16_t);
  HUM_TEMP_StatusTypeDef       (*GetHumidity)(float *);
  HUM_TEMP_StatusTypeDef       (*GetTemperature)(float *);
}HUM_TEMP_DrvTypeDef;


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __HUM_TEMP_H */
