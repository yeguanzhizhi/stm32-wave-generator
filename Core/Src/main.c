/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI              3.1415926f
#define WAVE_SIZE       128
#define VREF            3.3f
#define SAFETY          1.0f

#define DAC_MID         2048.0f
#define DAC_AMP         (2047.0f * SAFETY)
#define TIMER_CLK       240000000UL

#define FREQ_MIN        1000
#define FREQ_MAX        20000
#define FREQ_STEP       500

#define AMP_MIN         0.0f
#define AMP_MAX         3.3f
#define AMP_STEP        0.1f

#define KEY_NONE        0
#define KEY_FREQ_UP     1
#define KEY_FREQ_DOWN   2
#define KEY_AMP_UP      3
#define KEY_AMP_DOWN    4
#define KEY_WAVE_SWITCH 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

__attribute__((section(".DTCMRAM"))) uint16_t wave_sine[WAVE_SIZE];
__attribute__((section(".DTCMRAM"))) uint16_t wave_square[WAVE_SIZE];
__attribute__((section(".DTCMRAM"))) uint16_t wave_triangle[WAVE_SIZE];

static uint32_t current_freq = 1000;
static float    current_amp  = 1.0f;
static uint8_t  current_wave = 0;
static uint16_t *active_table = wave_sine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DEBOUNCE_MS 20

static uint8_t  last_freq_up=1,   pend_freq_up=0;   static uint32_t tick_freq_up=0;
static uint8_t  last_freq_down=1, pend_freq_down=0; static uint32_t tick_freq_down=0;
static uint8_t  last_amp_up=1,    pend_amp_up=0;    static uint32_t tick_amp_up=0;
static uint8_t  last_amp_down=1,  pend_amp_down=0;  static uint32_t tick_amp_down=0;
static uint8_t  last_wave=1,      pend_wave=0;      static uint32_t tick_wave=0;

// 状态机宏：一个按键的完整逻辑
#define SCAN_KEY(last, pend, tick, port, pin, code)   
    do {                                              
        uint8_t now = HAL_GPIO_ReadPin(port, pin);   
        /* 下降沿：记录时间戳，进入等待 */             
        if (last == 1 && now == 0) {                  
            tick = HAL_GetTick();                     
            pend = 1;                                 
        }                                             
        /* 等待期间仍为低，且超过消抖时间 → 确认按下 */ 
        if (pend && now == 0) {                       
            if (HAL_GetTick() - tick >= DEBOUNCE_MS) {
                ret = code;                           
                pend = 0;                             
            }                                         
        }                                             
        /* 中途松开 → 取消等待 */                      
        if (now == 1) pend = 0;                       
        last = now;                                   
    } while (0)

uint8_t Key_Scan(void)
{
    uint8_t ret = KEY_NONE;

    SCAN_KEY(last_freq_up,   pend_freq_up,   tick_freq_up,
             FREQ_UP_GPIO_Port,     FREQ_UP_Pin,     KEY_FREQ_UP);

    SCAN_KEY(last_freq_down, pend_freq_down, tick_freq_down,
             FREQ_DOWN_GPIO_Port,   FREQ_DOWN_Pin,   KEY_FREQ_DOWN);

    SCAN_KEY(last_amp_up,    pend_amp_up,    tick_amp_up,
             AMP_UP_GPIO_Port,      AMP_UP_Pin,      KEY_AMP_UP);

    SCAN_KEY(last_amp_down,  pend_amp_down,  tick_amp_down,
             AMP_DOWN_GPIO_Port,    AMP_DOWN_Pin,    KEY_AMP_DOWN);

    SCAN_KEY(last_wave,      pend_wave,      tick_wave,
             WAVE_SWITCH_GPIO_Port, WAVE_SWITCH_Pin, KEY_WAVE_SWITCH);

    return ret;
}
/* ========== 波形表填充 ========== */
static void fill_sine(uint16_t *table, float amp_scale)
{
    float angle = 0.0f;
    const float step = 2.0f * PI / WAVE_SIZE;
    for (uint16_t i = 0; i < WAVE_SIZE; i++) {
        float val = DAC_MID + DAC_AMP * amp_scale * sinf(angle);
        if (val < 0.0f)   val = 0.0f;
        if (val > 4095.0f) val = 4095.0f;
        table[i] = (uint16_t)val;
        angle += step;
    }
}

static void fill_square(uint16_t *table, float amp_scale)
{
    float hi = DAC_MID + DAC_AMP * amp_scale;
    float lo = DAC_MID - DAC_AMP * amp_scale;
    if (hi > 4095.0f) hi = 4095.0f;
    if (lo < 0.0f)    lo = 0.0f;
    uint16_t high = (uint16_t)hi;
    uint16_t low  = (uint16_t)lo;
    for (uint16_t i = 0; i < WAVE_SIZE; i++) {
        table[i] = (i < WAVE_SIZE / 2) ? high : low;
    }
}

static void fill_triangle(uint16_t *table, float amp_scale)
{
    for (uint16_t i = 0; i < WAVE_SIZE; i++) {
        float phase;
        if (i < WAVE_SIZE / 2) {
            phase = 4.0f * i / WAVE_SIZE - 1.0f;
        } else {
            phase = 3.0f - 4.0f * i / WAVE_SIZE;
        }
        float val = DAC_MID + DAC_AMP * amp_scale * phase;
        if (val < 0.0f)   val = 0.0f;
        if (val > 4095.0f) val = 4095.0f;
        table[i] = (uint16_t)val;
    }
}

static void rebuild_tables(void)
{
    float scale = current_amp / VREF;
    fill_sine(wave_sine, scale);
    fill_square(wave_square, scale);
    fill_triangle(wave_triangle, scale);
}

/* ========== 频率设置 ========== */
static void set_frequency(uint32_t freq_hz)
{
    uint32_t trigger_freq = freq_hz * WAVE_SIZE;
    uint32_t arr = TIMER_CLK / trigger_freq - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    current_freq = freq_hz;
}

/* ========== 重启 DAC DMA ========== */
static void restart_dac_dma(void)
{
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_Delay(2);                              // 等DMA彻底停稳
    SCB_CleanDCache_by_Addr((uint32_t*)active_table, WAVE_SIZE * 2);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                      (uint32_t*)active_table, WAVE_SIZE,
                      DAC_ALIGN_12B_R);
}

/* ========== 按键处理 ========== */
static void handle_key(uint8_t key)
{
    switch (key) {
        case KEY_FREQ_UP:
            if (current_freq + FREQ_STEP <= FREQ_MAX)
                set_frequency(current_freq + FREQ_STEP);
            break;

        case KEY_FREQ_DOWN:
            if (current_freq >= FREQ_MIN + FREQ_STEP)
                set_frequency(current_freq - FREQ_STEP);
            break;

        case KEY_AMP_UP:
            if (current_amp + AMP_STEP <= AMP_MAX) {
                current_amp += AMP_STEP;
                rebuild_tables();
                restart_dac_dma();
            }
            break;

        case KEY_AMP_DOWN:
            if (current_amp >= AMP_MIN + AMP_STEP) {
                current_amp -= AMP_STEP;
                rebuild_tables();
                restart_dac_dma();
            }
            break;

        case KEY_WAVE_SWITCH:
            current_wave = (current_wave + 1) % 3;
            if (current_wave == 0)      active_table = wave_sine;
            else if (current_wave == 1) active_table = wave_square;
            else                        active_table = wave_triangle;
            restart_dac_dma();
            break;

        default:
            break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  rebuild_tables();

  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  set_frequency(1000);

  SCB_CleanDCache_by_Addr((uint32_t*)wave_sine, WAVE_SIZE * 2);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                    (uint32_t*)wave_sine, WAVE_SIZE,
                    DAC_ALIGN_12B_R);

  HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	   uint8_t key = Key_Scan();
	   if (key != KEY_NONE) handle_key(key);
	   HAL_Delay(1);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 18774;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : FREQ_UP_Pin FREQ_DOWN_Pin AMP_UP_Pin AMP_DOWN_Pin
                           WAVE_SWITCH_Pin */
  GPIO_InitStruct.Pin = FREQ_UP_Pin|FREQ_DOWN_Pin|AMP_UP_Pin|AMP_DOWN_Pin
                          |WAVE_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
