# VL53L0X ToF Distance Sensor Library - STM32 HAL (Tested on Nucleo-F411RE)
A simple C library (STM32 HAL) for ST's VL53L0X Time-of-Fligt distance sensor based on yetifrisstlama's VL53L0X non-Arduino library: https://github.com/yetifrisstlama/vl53l0x-non-arduino

This library includes continous and single measurements and few configuration functions.

* After the initialization of the sensor, the user should call configuration functions in this order (As recommended by the manufacturer.):
* setSignalRateLimit(), setVcselPulsePeriod() and setMeasurementTimingBudget(). More information can be found at: https://www.st.com/resource/en/datasheet/vl53l0x.pdf

## Example Usage
* In the given example, an USART peripheral configured to send message to a computer. (USART2 on Nucleo-F411RE)

Configure an I2C peripheral on CubeMX with fast mode (I2C speed).

First include the library,
```c

/* USER CODE BEGIN Includes */
#include "VL53L0X.h"
#include "stdio.h"
/* USER CODE END Includes */

```
And for initialization,
```c
 /* USER CODE BEGIN 2 */

	// Initialise a message buffer.
	char msgBuffer[52];
	for (uint8_t i = 0; i < 52; i++) {
		msgBuffer[i] = ' ';
	}

	// Initialise the VL53L0X
	statInfo_t_VL53L0X distanceStr;
	initVL53L0X(1, &hi2c1);

	// Configure the sensor for high accuracy and speed in 20 cm.
	setSignalRateLimit(200);
	setVcselPulsePeriod(VcselPeriodPreRange, 10);
	setVcselPulsePeriod(VcselPeriodFinalRange, 14);
	setMeasurementTimingBudget(300 * 1000UL);

	uint16_t distance;

  /* USER CODE END 2 */
```

And then, the while loop for the single measurement configuration,

```c

/* USER CODE BEGIN WHILE */
	while (1) {

		// uint16_t distance is the distance in millimeters.
		// statInfo_t_VL53L0X distanceStr is the statistics read from the sensor.
		distance = readRangeSingleMillimeters(&distanceStr);

		sprintf(msgBuffer, "Distance: %d\r\n", distance);

		HAL_UART_Transmit(&huart2, (uint8_t*) msgBuffer, sizeof(msgBuffer), 50);

    /* USER CODE END WHILE */

```
