/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2025 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "cs43l22.h"
#include "math.h"
#include <stdlib.h>
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_accelerometer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINE_BUFFER_SIZE 256

// macros to define the sine signal
#define FAST_SIN_FREQ 1000
#define SLOW_SIN_FREQ 500

#define SAMPLING_RATE 48000
#define AUDIO_BUFFER_LENGTH SAMPLING_RATE / SLOW_SIN_FREQ

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t last_button_press_time = 0;//simple for debuncing
volatile uint8_t use_pwm_for_led = 1;//for led mode ctrl,1->pwm mode,0->manual mode
//add for acc data store 
int16_t xyz_buffer[3];
//add variables for PWM
//PSC of TIM10 is 16799，speed of counter is 0.1ms
#define PWM_FREQ_SLOW   20000  // slow ,whole period 2s
#define PWM_FREQ_MEDIUM 10000  // mid,whole period 1s
#define PWM_FREQ_FAST   5000   // fast,whole period 0.5s
uint32_t pwm_period_values[] = {PWM_FREQ_SLOW, PWM_FREQ_MEDIUM, PWM_FREQ_FAST};
volatile uint8_t  freq_index = 1;       //  (0=Slow, 1=Med, 2=Fast)

// add duty cycles 25%, 50%, 75%
float pwm_duty_values[] = {0.1f, 0.50f, 0.9f};
volatile uint8_t duty_index = 1;        //  default set 50% (0=25%, 1=50%, 2=75%)

// check led status
volatile uint8_t led_is_on = 0; 

// for PWM manual mode,PWM controled by user button 
volatile uint8_t is_pwm_manual = 0;


const uint8_t ISR_FLAG_RX    = 0x01;  // Received data
const uint8_t ISR_FLAG_TIM10 = 0x02;  // Timer 10 Period elapsed
const uint8_t ISR_FLAG_TIM11 = 0x04;  // Timer 10 Period elapsed

// This is the only variable that we will modify both in the ISR and in the main loop
// It has to be declared volatile to prevent the compiler from optimizing it out.
volatile uint8_t isr_flags = 0;

// Commands that we will receive from the PC
const uint8_t COMMAND_CHANGE_FREQ[] = "changefreq"; // Change the frequency
const uint8_t COMMAND_STOP[]        = "stop"; // Go to stop mode
// below are the new added commands -xuxin 
const uint8_t COMMAND_STANDBY[]     = "standby";
const uint8_t COMMAND_CHANGE_DUT[]  = "changedut";
const uint8_t COMMAND_PWM_MAN[]     = "pwmman";
const uint8_t COMMAND_LED_PWM[]     = "ledpwm";
const uint8_t COMMAND_LED_MAN[]     = "ledman";
const uint8_t COMMAND_LED_ON[]      = "ledon";
const uint8_t COMMAND_LED_OFF[]     = "ledoff";
const uint8_t COMMAND_ACC_ON[]      = "accon";
const uint8_t COMMAND_ACC_OFF[]     = "accoff";
const uint8_t COMMAND_MUTE[]        = "mute";
const uint8_t COMMAND_UNMUTE[]      = "unmute";
// Buffer for command
static uint8_t line_ready_buffer[LINE_BUFFER_SIZE]; // Stable buffer for main

// Audio buffer
int16_t buffer_audio[2 * AUDIO_BUFFER_LENGTH];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Interrupt handlers
void handle_new_line();

// Commands
void go_to_stop();
void go_to_standby(void);
// Helper functions
void init_codec_and_play();
// add  corrponding functions delare for uart commands
// for PWM and LED
void change_freq(void);
void change_duty_cycle(void);
void set_pwm_manual(void);
void led_set_pwm_mode(void);
void led_set_manual_mode(void);
void led_turn_on(void);
void led_turn_off(void);

// Commands - Accelerometer
void acc_enable(void);
void acc_disable(void);
void audio_mute(void); 
void audio_unmute(void);

void update_audio_for_pwm(uint32_t pwm_ticks);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM10_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
  BSP_ACCELERO_Init();//init the acc 
  init_codec_and_play();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // We check if that there are NO interrupts pending before going into sleep
    if (isr_flags == 0)
    {
      // Go to sleep, waiting for interrupt (WFI).
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
    }

    // This is needed for the UART transmission 
    if (isr_flags & ISR_FLAG_RX)
    {
      isr_flags &= ~ISR_FLAG_RX;
      handle_new_line();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// All of this is used to manage commands from serial interface
static uint8_t line_buffer[LINE_BUFFER_SIZE];
static uint32_t line_len = 0;
void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len)
{
  // Prevent overflow, does not handle the command
  if (line_len + len >= LINE_BUFFER_SIZE)
  {
    line_len = 0;
    return;
  }

  // Append received chunk
  memcpy(&line_buffer[line_len], buf, len);
  line_len += len;

  // Process all complete lines
  while (1)
  {
    // Look for '\n' or '\r' inside line_buffer
    uint8_t *line_feed = memchr(line_buffer, '\n', line_len);
    if (line_feed == NULL){
      line_feed = memchr(line_buffer, '\r', line_len);
      if (line_feed == NULL)
      break;
    }

    // Replace \n or \r by terminator
    *line_feed = '\0';

    // Remove optional '\r' in case input was \r\n
    if (line_feed > line_buffer && *(line_feed - 1) == '\r')
    *(line_feed - 1) = '\0';

    uint32_t processed = (line_feed + 1) - line_buffer;

    // Signal the main that there is a new line ready to be checked
    memcpy(line_ready_buffer, line_buffer, processed);
    isr_flags |= ISR_FLAG_RX;

    // Move leftover bytes to start
    line_len -= processed;
    memmove(line_buffer, line_buffer + processed, line_len);
  }
}

/**
* @brief  Handle possible new command
* @retval None
*/
void handle_new_line()
{
  //add commands for PWM
  if (memcmp(line_ready_buffer, COMMAND_CHANGE_FREQ, sizeof(COMMAND_CHANGE_FREQ)-1) == 0)
  {
      change_freq();
      CDC_Transmit_FS((uint8_t*)"Frequency changed\r\n", 19);
  }
  else if (memcmp(line_ready_buffer, COMMAND_CHANGE_DUT, sizeof(COMMAND_CHANGE_DUT)-1) == 0) {
      change_duty_cycle();
      CDC_Transmit_FS((uint8_t*)"Duty cycle changed\r\n", 20);
  }
  else if (memcmp(line_ready_buffer, COMMAND_PWM_MAN, sizeof(COMMAND_PWM_MAN)-1) == 0) {
      set_pwm_manual();
      CDC_Transmit_FS((uint8_t*)"Manual PWM setup start\r\n", 24);
  }
  //add commands for LED 
  else if (memcmp(line_ready_buffer, COMMAND_LED_PWM, sizeof(COMMAND_LED_PWM)-1) == 0) {
      led_set_pwm_mode();
      CDC_Transmit_FS((uint8_t*)"LED Mode: PWM\r\n", 15);
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_MAN, sizeof(COMMAND_LED_MAN)-1) == 0) {
      led_set_manual_mode();
      CDC_Transmit_FS((uint8_t*)"LED Mode: Manual\r\n", 18);
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_ON, sizeof(COMMAND_LED_ON)-1) == 0) {
      led_turn_on();
      CDC_Transmit_FS((uint8_t*)"LED ON\r\n", 8);
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_OFF, sizeof(COMMAND_LED_OFF)-1) == 0) {
      led_turn_off();
      CDC_Transmit_FS((uint8_t*)"LED OFF\r\n", 9);
  }
  //FOR POWER modes 
  else if (memcmp(line_ready_buffer, COMMAND_STOP, sizeof(COMMAND_STOP)-1) == 0)
  {
      CDC_Transmit_FS((uint8_t*)"Entering STOP mode...\r\n", 23);
      go_to_stop();
  }
  else if (memcmp(line_ready_buffer, COMMAND_STANDBY, sizeof(COMMAND_STANDBY)-1) == 0) {
      CDC_Transmit_FS((uint8_t*)"Entering STANDBY mode...\r\n", 26);
      go_to_standby();
  }
  //add for audio
  else if (memcmp(line_ready_buffer, COMMAND_MUTE, sizeof(COMMAND_MUTE)-1) == 0) {
      audio_mute();
      CDC_Transmit_FS((uint8_t*)"Audio Muted\r\n", 13);
  }
  else if (memcmp(line_ready_buffer, COMMAND_UNMUTE, sizeof(COMMAND_UNMUTE)-1) == 0) {
      audio_unmute();
      CDC_Transmit_FS((uint8_t*)"Audio Unmuted\r\n", 15);
  }
  // for  Accelerometer
  else if (memcmp(line_ready_buffer, COMMAND_ACC_ON, sizeof(COMMAND_ACC_ON)-1) == 0) {
      //CDC_Transmit_FS((uint8_t*)"ACC Enabled\r\n", 13);  
      acc_enable();
      CDC_Transmit_FS((uint8_t*)"ACC Enabled\r\n", 13);
  }
  else if (memcmp(line_ready_buffer, COMMAND_ACC_OFF, sizeof(COMMAND_ACC_OFF)-1) == 0) {
      acc_disable();
      CDC_Transmit_FS((uint8_t*)"ACC Disabled\r\n", 14);
  }
  else
  {
    // If we receive an unknown command, we send an error message back to the PC
    CDC_Transmit_FS((uint8_t*)"Unknown command\r\n", 17);
  }
}
void go_to_standby() 
{ 
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    // cs43l22_stop();
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnterSTANDBYMode();
}

//add interrupt callback function for change the ARR to change duty
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //for PWM LED
    if (htim->Instance == TIM10)
    {
        // Green=PD12, Orange=PD13, Red=PD14, Blue=PD15
        if (use_pwm_for_led == 1)//only in pwm mode
        {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        }
        // switch flag
        led_is_on = !led_is_on;

        uint32_t total_period = pwm_period_values[freq_index];//implement the freq ctrl, this is related to the "period" in LAB1
        uint32_t on_time = (uint32_t)(total_period * pwm_duty_values[duty_index]);//by changing the ratio of on and off ,change the period behavior of led
        uint32_t off_time = total_period - on_time;

        if (led_is_on)
        {
            //  if on, will overflow after reach on_time, then into off
            __HAL_TIM_SET_AUTORELOAD(htim, on_time);
        }
        else
        {
            __HAL_TIM_SET_AUTORELOAD(htim, off_time);
        }
    }
    
    //new logic to handle ACC value
    if (htim->Instance == TIM1)
    {
       
        BSP_ACCELERO_GetXYZ(xyz_buffer);
        
        // xyz_buffer[0] = X, [1] = Y, [2] = Z
         // positive only because direction
        int16_t abs_x = abs(xyz_buffer[0]);
        int16_t abs_y = abs(xyz_buffer[1]);
        int16_t abs_z = abs(xyz_buffer[2]);

        //using the highest value to control led
        if (abs_x > abs_y && abs_x > abs_z)
        {
            // Z  -> blue ON, others OFF
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_RESET);
        }
        else if (abs_y > abs_x && abs_y > abs_z)
        {
            // x  -> Red ON, others OFF
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);
        }
        else
        {
            // Y  -> orange ON, others OFF
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
        }
    }
}

void change_freq() 
{ 
    freq_index++;
    if (freq_index >= 3) freq_index = 0;
    //for requirement, "Proportional" change
    update_audio_for_pwm(pwm_period_values[freq_index]);
    is_pwm_manual = 0;

}
void change_duty_cycle() 
{ 
    is_pwm_manual = 0;
    duty_index++;
    if (duty_index >= 3) duty_index = 0;
}
void set_pwm_manual()
{ 
    is_pwm_manual = 1;
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); 
}

void led_set_pwm_mode() 
{ 
  use_pwm_for_led = 1; 
}
void led_set_manual_mode() 
{
  is_pwm_manual = 0;
  use_pwm_for_led = 0;
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); 
}
void led_turn_on() 
{ 
        if(use_pwm_for_led == 0)//ledman
        {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        }
}
void led_turn_off() 
{ 
        if(use_pwm_for_led == 0)//ledman
        //HAL_TIM_Base_Stop_IT(&htim10);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

void acc_enable() 
{
  if (BSP_ACCELERO_Init() != HAL_OK)
    
  {
        Error_Handler();
  }

  //start tim1 interrupt, to read acc value
  HAL_TIM_Base_Start_IT(&htim1);

}
void acc_disable() 
{ 
    HAL_TIM_Base_Stop_IT(&htim1);
    //off all the leds
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET); 
}

void audio_mute() { 
    cs43l22_mute();
}
void audio_unmute() { 
    cs43l22_unmute();
}
void init_codec_and_play()
{
  cs43l22_init();
  // sine signal
  //REUSED IN NEW FUNCTION 
  for(int i = 0; i < AUDIO_BUFFER_LENGTH;i++)
  {
    buffer_audio[2 * i] = 10000 * sin(2 * 3.14 * SLOW_SIN_FREQ * i / SAMPLING_RATE);
    buffer_audio[2 * i + 1] = 10000 * sin(2 * 3.14 * SLOW_SIN_FREQ * i / SAMPLING_RATE);
  }
  
  cs43l22_play(buffer_audio, 2 * AUDIO_BUFFER_LENGTH);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    
    if (GPIO_Pin == GPIO_PIN_0)
    {

      if (is_pwm_manual == 1)
        {
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) // pressed
            {
                __HAL_TIM_SET_COUNTER(&htim2, 0); // clear TIM2
                HAL_TIM_Base_Start(&htim2);       // begin count
            }
            else // should enable rising/falling interrupt, pressed->interrupt->enter "if"begin count->exit , loose->next time enter interrupt->enter "else" end count
            {
                HAL_TIM_Base_Stop(&htim2);        // stop count
                uint32_t press_time = __HAL_TIM_GET_COUNTER(&htim2); // get time diff
                
                // simple filter
                if (press_time > 100) 
                {
                    // set new freq
                    pwm_period_values[freq_index] = press_time;
                    update_audio_for_pwm(press_time);
                   CDC_Transmit_FS((uint8_t*)"PWM Set Done!\r\n", 15);
                }
                 
            }
        }
        // Debounce
        if (use_pwm_for_led == 0) //ledman
        {

            static uint32_t last_time = 0;
            uint32_t current_time = HAL_GetTick();
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET && (current_time - last_time > 200))
            {
                last_time = current_time;
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
            }
        }
      
    }
}

/**
* @brief Go to stop mode
* @retval None
*/
void go_to_stop()
{
  // TODO: Make sure all user LEDS are off
  //add leds off
   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
  // To avoid noise during stop mode
  cs43l22_stop();

  // Required otherwise the audio wont work after wakeup
  HAL_I2S_DeInit(&hi2s3);

  // We disable the systick interrupt before going to stop (1ms tick)
  // Otherwise we would be woken up every 1ms
  HAL_SuspendTick();

  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  // We need to reconfigure the system clock after waking up.
  // After exiting from stop mode, the system clock is reset to the default
  // which is not the same we configure in cubeMx, so we do it as done
  // during the init, by calling SystemClock_Config().
  SystemClock_Config();
  HAL_ResumeTick();

  // Required otherwise the audio wont work after wakeup
  MX_I2S3_Init();

  init_codec_and_play(pwm_period_values[freq_index]);
}

void update_audio_for_pwm(uint32_t pwm_ticks)
{

    // according　PWM value to freq.
    //  example : (20000 ticks) -> 10,000,000 / 20,000 = 500 Hz
    //  (10000 ticks) -> 10,000,000 / 10,000 = 1000 Hz

    if(pwm_ticks >20000 ||pwm_ticks ==20000)
    {
        pwm_ticks = 20000;
    }else if (pwm_ticks >= 10000|| pwm_ticks ==10000){
      pwm_ticks = 10000;
    }else {
      pwm_ticks =5000;
    }
    
    uint16_t calculated_freq = 10000000 / pwm_ticks;


    if (calculated_freq < 100) calculated_freq = 100;
    if (calculated_freq > 20000) calculated_freq = 20000;
    // same method of init function
    for(int i = 0; i < AUDIO_BUFFER_LENGTH; i++)
    {
        int16_t sample_value = (int16_t)(10000 * sin(2 * 3.14f * calculated_freq * i / SAMPLING_RATE));
        buffer_audio[2 * i]     = sample_value;
        buffer_audio[2 * i + 1] = sample_value;
    }

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
