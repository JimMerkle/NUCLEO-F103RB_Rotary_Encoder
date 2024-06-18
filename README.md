## Rotary Encoder - Interface with STM32 (NUCLEO-F103RB)
    STM32 provides timers capable of decoding quadrature encoded signals
    from a rotary encoder, like the KY-040 Rotary Encoder Module.
	This particular module uses different encoders depending on the manufacturer.
	The encoder switch on my module has 15 pulses per revolution with 30 detents.
	There are several manufacturers of the encoder switch itself, including Bourns,
	Alps, TT Electronics-BI, Panasonic, and others.

    Although the KY-040 module uses "CLK" and "DT" for signal names, these are the
	"A" and "B" switch outputs, that connect to the common GND signal as the rotary
	encoder rotates.  Two pull-up resistors on the board pull the signal
	high (to the "+" voltage) when either switch is open.


### Wiring Diagram
    Encoder Signal  Wire Color  NUCLEO Pin  STM32 Signal
	    CLK           Orange       CN7-28      PA0 (TIM2_CH1)
	    DT            Brown        CN7-30      PA1 (TIM2_CH2)
	    SW            Yellow       CN7-38      PC0
	    +             Red          CN7-16      3V3
        GND           Black        CN7-20      GND

### Configuring TIM2 for encoder usage
    1. Open STM32CubeIDE Pinout & Configuration tab -> Timers
    2. Select TIM2
    3. In the "TIM2 Mode and Configuration" dialog,
         select "Combined Channels" - Encoder Mode
	4. Use 65535 for counter period (default)
	5. Set "auto-reload preload" to "Enable"
	6. For both Channel 1 and Channel 2, set "Input Filter" to 10
	7. File->Save to save the .ico file - This builds the main.c file,
         configuring the GPIOs and TIM2 timer.

### main.c, printf() functionality:
#### Add include for stdio library header file:
```
/* USER CODE BEGIN Includes */
#include <stdio.h> // printf()
/* USER CODE END Includes */
```
#### Add putchar() function:
```
/* USER CODE BEGIN 0 */
#define HAL_SMALL_WAIT  40
// Define serial input and output functions using UART2
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_SMALL_WAIT);
    return 1;
}
/* USER CODE END 0 */
```
#### Disable stdio output buffering:
```
/* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  /* USER CODE END 2 */
```
### main.c, Infinite loop:
```
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t count=0;
  int16_t prev_count=100; // force display of position following each reset
  while (1)
  {
	// Read Rotary Encoder "position" via TIM2->CNT
	// The encoder uses STM32 pins PA0 and PA1
	count = (int16_t)TIM2->CNT;
	if(count !=prev_count){
		printf("%d\n",count);
		//printf("%d, A:%u, B:%u\n",count,HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0),HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
		prev_count = count;
	}
	// Read PA0 and turn the NUCLEO On-Board LED ON or OFF depending on the signal read
	if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) // switch closed - grounded
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // turn ON the LED
	else
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);   // turn OFF the LED
 
    /* USER CODE END WHILE */
 
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
```

### Examples and tutorials:
     https://wiki.st.com/stm32mcu/wiki/Getting_started_with_EXTI
     https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/