/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __fdcan_H
#define __fdcan_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

 FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

//void HAL_CAN_ErrorCallback (CAN_HandleTypeDef * hcan);



/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */
