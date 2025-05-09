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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sd_utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	SELECT_CARACTER_INSTRUCT,
	REFRESH_CARACTER_OPTIONS,
	WAIT_CARACTER_SELECTION,
	SELECT_STAGE_INSTRUCT,
	REFRESH_STAGE_OPTIONS,
	WAIT_STAGE_SELECTION,
	SET_CONDITIONS_SELECTED,
	GAME_ON_CURSE
} GameState;

typedef struct {
	// Relative
	int16_t pos_x; // int pos_x = 50;
	int16_t pos_y; // #define GROUND_Y 150

	// Sprite
	uint8_t walking_frame; // int walking_frame = 0;
	uint8_t flip_frame; // uint8_t flip = 1;
	uint8_t frame_width; // #define SUPERH_1_WIDTH 40
	uint8_t frame_height; // #define SUPERH_1_HEIGHT 50
	uint8_t frame_switch_distance; // #define SUPERH_FRAME_SWITCH_DISTANCE 5
	uint16_t frame_size; // #define SUPERH_1_SIZE    (SUPERH_1_WIDTH*SUPERH_1_HEIGHT*2)

	// Logic
	uint8_t actual_selection; // uint8_t PlayerA.actual_selection = 0;
	int8_t final_selection; // uint8_t hero_1_s = -1;
	bool confirm_selection; // bool hero_1_s_s = false;

	// Local buffers (pointers)
	uint8_t *walking1_bitmap; // uint8_t SUPERH_1_F1[SUPERH_1_SIZE];
	const char *walking1_filename;
	uint8_t *walking2_bitmap; // uint8_t SUPERH_1_F2[SUPERH_1_SIZE];
	const char *walking2_filename;
	uint8_t *attack_gadget_bitmap; // uint8_t SUPERH_1_ATTACK_G_1[SUPERH_1_SIZE];
	const char *attack_gadget_filename;
	uint8_t *attack_static_bitmap; // uint8_t SUPERH_1_ATTACK_1[SUPERH_1_SIZE];
	const char *attack_static_filename;
	uint8_t *jump_bitmap; // uint8_t SUPERH_1_JUMP_1[SUPERH_1_SIZE];
	const char *jump_filename;
	uint8_t *down_bitmap; // uint8_t SUPERH_1_DOWN[SUPERH_1_SIZE];
	const char *down_filename;
	uint8_t *defense_bitmap; // uint8_t SUPERH_1_DEFENSE[SUPERH_1_SIZE];
	const char *defense_filename;

	// Kinematics
	uint8_t jump_y; // uint8_t jump_y = GROUND_Y;
	int8_t jump_direction; // uint8_t jump_direction = -1;
	uint8_t jump_delay_counter; // uint8_t jump_delay_counter = 0;
	uint8_t defense; // uint8_t defense = 0;
	volatile uint8_t move_state; // volatile uint8_t move_state = 0;
	uint8_t delay_left_move; // uint8_t delay_isr = 0;
	uint8_t step_counter_left;
	uint8_t step_counter_right;
	uint8_t delay_right_move;

	// Actions
	uint8_t attack_static_flag; // Command 'U'
	uint8_t loaded_attack_static;
	uint8_t attack_gadget_flag; // Command 'R'
	uint8_t loaded_attack_gadget;
	uint8_t neutral_position_flag; // Command 'c'
	uint8_t loaded_neutral_position;
	uint8_t go_back_flag; // Command 'A'
	uint8_t move_left_flag; // Command 'l'
	uint8_t loaded_move_left;
	uint8_t move_right_flag; // Command 'r'
	uint8_t loaded_move_right;
	uint8_t bend_flag; // Command 'd'
	uint8_t loaded_bend;
	uint8_t jump_flag; // Command 'u'
	uint8_t loaded_jump;
	uint8_t defend_flag; // Command 'D'
	uint8_t loaded_defend;

	// Game attributes
	uint8_t vida; // uint8_t vida_h1 = 100;
} Player;

typedef struct {
	uint8_t frame_width; // #define GADGET_1_WIDTH 19
	uint8_t frame_height; // #define GADGET_1_HEIGHT 19
	uint8_t frame_switch_distance; // #define FRAME_SWITCH_DISTANCE 10
	uint16_t frame_size; // #define GADGET_1_SIZE    (GADGET_1_WIDTH*GADGET_1_HEIGHT*2)
	int16_t gadget_pos_x; // uint8_t pos_x_gadget = 0;
	int16_t gadget_last_x; // uint8_t pos_x_gadget_2 = 0;
	unsigned char *rotated_frame; // unsigned char *imagenRotada = 0;
	uint8_t *actual_frame;  // uint8_t GADGET_1_F1[GADGET_1_SIZE];
} Gadget;

typedef struct {
	uint8_t frame_width;
	uint8_t frame_height;
	int16_t stage_pos_x;
	int16_t stage_pos_y;
	uint8_t *full_bitmap;
} Logo;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Game limits
#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define MAX_X 220
#define GROUND_Y 150
#define JUMP_HEIGHT 120
#define JUMP_STEP 5
#define JUMP_DELAY 25
// Color palette
#define TRANSPARENT_COLOR 0x052A
#define COLOR_VERDE 0x07E0
#define COLOR_AZUL  0x001F

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Game status struct
GameState game_actual_state = SELECT_CARACTER_INSTRUCT;
uint8_t caracter_logo_buffer[50 * 50 * 2];
// Player 1 struct
uint8_t SUPERH_1_F1[40 * 50 * 2];
uint8_t SUPERH_1_F2[40 * 50 * 2];
uint8_t SUPERH_1_F3[40 * 50 * 2];

Player PlayerA = { .pos_x = 50, .pos_y = GROUND_Y,

.walking_frame = 0, .flip_frame = 1, .frame_width = 40, .frame_height = 50,
		.frame_switch_distance = 5, .frame_size = 40 * 50 * 2,

		.actual_selection = 0, .final_selection = -1,
		.confirm_selection = false,

		.walking1_bitmap = SUPERH_1_F1, .walking2_bitmap = SUPERH_1_F2,
		.attack_gadget_bitmap = SUPERH_1_F3,
		.attack_static_bitmap = SUPERH_1_F3, .jump_bitmap = SUPERH_1_F3,
		.down_bitmap = SUPERH_1_F3, .defense_bitmap = SUPERH_1_F3,

		.jump_y = GROUND_Y, .jump_direction = -1, .jump_delay_counter = 0,
		.defense = 0, .move_state = 0,

		.vida = 100 };
// Player 2 struct
uint8_t SUPERH_2_F1[40 * 50 * 2];
uint8_t SUPERH_2_F2[40 * 50 * 2];
uint8_t SUPERH_2_F3[40 * 50 * 2];

Player PlayerB = { .pos_x = 270, .pos_y = GROUND_Y,

.walking_frame = 0, .flip_frame = 0, .frame_width = 40, .frame_height = 50,
		.frame_switch_distance = 5, .frame_size = 40 * 50 * 2,

		.actual_selection = 0, .final_selection = -1,
		.confirm_selection = false,

		.walking1_bitmap = SUPERH_2_F1, .walking2_bitmap = SUPERH_2_F2,
		.attack_gadget_bitmap = SUPERH_2_F3,
		.attack_static_bitmap = SUPERH_2_F3, .jump_bitmap = SUPERH_2_F3,
		.down_bitmap = SUPERH_2_F3, .defense_bitmap = SUPERH_2_F3,

		.jump_y = GROUND_Y, .jump_direction = -1, .jump_delay_counter = 0,
		.defense = 0, .move_state = 0,

		.vida = 100 };
// Gadget struct
uint8_t GADGET_1_F1[19 * 19 * 2];
Gadget CaptainShieldA = { .frame_width = 19, .frame_height = 19,
		.frame_switch_distance = 10, .frame_size = 19 * 19 * 2, .gadget_pos_x =
				0, .gadget_last_x = GROUND_Y, .rotated_frame = NULL,
		.actual_frame = GADGET_1_F1 };

uint8_t GADGET_2_F1[19 * 19 * 2];
Gadget CaptainShieldB = { .frame_width = 19, .frame_height = 19,
		.frame_switch_distance = 10, .frame_size = 19 * 19 * 2, .gadget_pos_x =
				0, .gadget_last_x = GROUND_Y, .rotated_frame = NULL,
		.actual_frame = GADGET_2_F1 };
// Stage struct
uint8_t stage_actual_selection = 0;
bool stage_confirm_selection = false;
uint8_t stage_logo_buffer[50 * 38 * 2];

extern unsigned char STAGE_1[];
Logo Stage1 = { .frame_width = 50, .frame_height = 38, .stage_pos_x = 135,
		.stage_pos_y = 95, .full_bitmap = STAGE_1, };

extern unsigned char STAGE_2[];
Logo Stage2 = { .frame_width = 50, .frame_height = 38, .stage_pos_x = 135,
		.stage_pos_y = 95, .full_bitmap = STAGE_2, };
// Life level control
char life_text_buffer[10];
// UART reception
volatile char input = 0;
uint8_t command_uart_buffer[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM7_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void RefreshingCaracterOptions(void);
void WaitingCaracterSelection(void);
void RefreshingStageOptions(void);
void WaitingStageSelection(void);
void SettingConditionsSelected(void);
void GameOnCurse(void);

void PlayerA_action_U();
void PlayerA_action_R();
void PlayerA_action_D();
void PlayerA_action_A();
void PlayerA_action_u();
void PlayerA_action_d();
void PlayerA_action_r();
void PlayerA_action_l();
void PlayerA_action_c();

void PlayerB_action_f();
void PlayerB_action_v();
void PlayerB_action_h();
void PlayerB_action_y();
void PlayerB_action_8();
void PlayerB_action_2();
void PlayerB_action_6();
void PlayerB_action_4();
void PlayerB_action_5();

void SD_info_2(void);
void flip_horizontal(const uint8_t *src, uint8_t *dst, uint16_t width,
		uint16_t height);

void LCD_Bitmap_Black(unsigned int x, unsigned int y, unsigned int width,
		unsigned int height, unsigned char bitmap[]);

void LCD_Bitmap_withTransparency(unsigned int x, unsigned int y,
		unsigned int width, unsigned int height, unsigned char bitmap[],
		uint8_t flip);

void Rect_Grueso(unsigned int x, unsigned int y, unsigned int w, unsigned int h,
		unsigned int c);

void animate_attack(void);

unsigned char* rotateBitmap90(const unsigned char *src, uint8_t flip);
unsigned char* rotateBitmap180(const unsigned char *src);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM7_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart5, command_uart_buffer, 1);
//	HAL_TIM_Base_Start_IT(&htim7);
	LCD_Init();
	LCD_Clear(0x00);
	LCD_StreamBitmapFromSD("intro1.bin", 0, 0, 320, 240);
	HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		static GameState game_prev_state = -1;
		if (game_prev_state != game_actual_state) {
			/* ESTADOS DE UNA SOLA EJECUCIÓN */
			switch (game_actual_state) {
			case SELECT_CARACTER_INSTRUCT:
				LCD_Clear(0x00);
				LCD_Print("Selecciona", 0, 0, 2, 0xFFFF, 0x00);
				LCD_Print("tus personajes", 0, 30, 2, 0xFFFF, 0x00);
				LCD_Print("A: Volver a seleccionar", 50, 230, 1, 0xFFFF, 0x00);
				game_actual_state = REFRESH_CARACTER_OPTIONS;
				break;
			case REFRESH_CARACTER_OPTIONS:
				RefreshingCaracterOptions();
				/* ESTADO CONTINUO */
				game_actual_state = WAIT_CARACTER_SELECTION;
				game_prev_state = game_actual_state;
				break;

			case SELECT_STAGE_INSTRUCT:
				LCD_Clear(0x00);
				LCD_Print("Selecciona la", 0, 0, 2, 0xFFFF, 0x00);
				LCD_Print("Arena de combate", 0, 30, 2, 0xFFFF, 0x00);
				LCD_Print("A: Volver ", 50, 230, 1, 0xFFFF, 0x00);
				game_actual_state = REFRESH_STAGE_OPTIONS;
				break;
			case REFRESH_STAGE_OPTIONS:
				RefreshingStageOptions();
				/* ESTADO CONTINUO */
				game_actual_state = WAIT_STAGE_SELECTION;
				game_prev_state = game_actual_state;
				break;
			case SET_CONDITIONS_SELECTED:
				SettingConditionsSelected();
				/* ESTADO CONTINUO */
				game_actual_state = GAME_ON_CURSE;
				game_prev_state = game_actual_state;
				break;
			}
		}
		switch (game_actual_state) {
		case WAIT_CARACTER_SELECTION:
			WaitingCaracterSelection();
			break;
		case WAIT_STAGE_SELECTION:
			WaitingStageSelection();
			break;
		case GAME_ON_CURSE:
			GameOnCurse();
			break;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 840-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 998-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LED_Pin|LCD_D6_Pin|LCD_D3_Pin
                          |LCD_D5_Pin|LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SS_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	input = command_uart_buffer[0];  // comando recibido
	switch (input) {
	case 'U':
		PlayerA.attack_static_flag = 1;
		PlayerA.loaded_attack_static = 1;

		break;
	case 'R':
		PlayerA.attack_gadget_flag = 1;
		PlayerA.loaded_attack_gadget = 1;
		break;
	case 'c':
		PlayerA.neutral_position_flag = 1;
		PlayerA.loaded_neutral_position = 1;
		break;
	case 'A':
		PlayerA.go_back_flag = 1;
		break;
	case 'l':
		PlayerA.move_left_flag = 1;
		PlayerA.move_right_flag = 0;
		PlayerA.loaded_move_left = 1;
		break;
	case 'r':
		PlayerA.move_right_flag = 1;
		PlayerA.move_left_flag = 0;
		PlayerA.loaded_move_right = 1;
		break;
	case 'd':
		PlayerA.bend_flag = 1;
		PlayerA.loaded_bend = 1;
		break;
	case 'u':
		PlayerA.jump_flag = 1;
		PlayerA.loaded_jump = 1;
		break;
	case 'D':
		PlayerA.defend_flag = 1;
		PlayerA.loaded_defend = 1;
		break;

	case 'f':
		PlayerB.attack_static_flag = 1;
		PlayerB.loaded_attack_static = 1;
		break;
	case 'v':
		PlayerB.attack_gadget_flag = 1;
		PlayerB.loaded_attack_gadget = 1;
		break;
	case '5':
		PlayerB.neutral_position_flag = 1;
		PlayerB.loaded_neutral_position = 1;
		break;
	case 'y':
		PlayerB.go_back_flag = 1;
		break;
	case '4':
		PlayerB.move_left_flag = 1;
		PlayerB.move_right_flag = 0;
		PlayerB.loaded_move_left = 1;
		break;
	case '6':
		PlayerB.move_right_flag = 1;
		PlayerB.move_left_flag = 0;
		PlayerB.loaded_move_right = 1;
		break;
	case '2':
		PlayerB.bend_flag = 1;
		PlayerB.loaded_bend = 1;
		break;
	case '8':
		PlayerB.jump_flag = 1;
		PlayerB.loaded_jump = 1;
		break;
	case 'h':
		PlayerB.defend_flag = 1;
		PlayerB.loaded_defend = 1;
		break;

	}

	HAL_UART_Receive_IT(&huart5, command_uart_buffer, 1);
}

void SD_info_2(void) {
	if (PlayerA.final_selection == 0) {
		SD_readImageBin("cag11.bin", CaptainShieldA.actual_frame,
				CaptainShieldA.frame_size);
		// normal
		PlayerA.walking1_filename = "cash1.bin";
		PlayerA.walking2_filename = "cash2.bin";
		PlayerA.attack_gadget_filename = "caas2.bin";
		PlayerA.attack_static_filename = "caa2.bin";
		PlayerA.jump_filename = "cashj.bin";
		PlayerA.down_filename = "cad.bin";
		PlayerA.defense_filename = "cadef.bin";
		SD_readImageBin(PlayerA.walking1_filename, PlayerA.walking1_bitmap,
				PlayerA.frame_size);
		SD_readImageBin(PlayerA.walking2_filename, PlayerA.walking2_bitmap,
				PlayerA.frame_size);
	}

	switch (PlayerB.final_selection) {
	case 0:
		SD_readImageBin("cag11.bin", CaptainShieldB.actual_frame,
				CaptainShieldB.frame_size);
		// normal
		PlayerB.walking1_filename = "cash1.bin";
		PlayerB.walking2_filename = "cash2.bin";
		PlayerB.attack_gadget_filename = "caas2.bin";
		PlayerB.attack_static_filename = "caa2.bin";
		PlayerB.jump_filename = "cashj.bin";
		PlayerB.down_filename = "cad.bin";
		PlayerB.defense_filename = "cadef.bin";
		SD_readImageBin(PlayerB.walking1_filename, PlayerB.walking1_bitmap,
				PlayerB.frame_size);
		SD_readImageBin(PlayerB.walking2_filename, PlayerB.walking2_bitmap,
				PlayerB.frame_size);
		break;
	case 1:
		SD_readImageBin("smn.bin", PlayerB.walking1_bitmap, PlayerB.frame_size);
		SD_readImageBin("sma.bin", PlayerB.walking2_bitmap, PlayerB.frame_size);
		SD_readImageBin("smg.bin", PlayerB.attack_gadget_bitmap,
				PlayerB.frame_size);
		break;

	case 2:
		SD_readImageBin("bmn.bin", PlayerB.walking1_bitmap, PlayerB.frame_size);
		SD_readImageBin("bma.bin", PlayerB.walking2_bitmap, PlayerB.frame_size);
		SD_readImageBin("bmg.bin", PlayerB.attack_gadget_bitmap,
				PlayerB.frame_size);
		break;

	case 3:
		SD_readImageBin("imn.bin", PlayerB.walking1_bitmap, PlayerB.frame_size);
		SD_readImageBin("ima.bin", PlayerB.walking2_bitmap, PlayerB.frame_size);
		SD_readImageBin("img.bin", PlayerB.attack_gadget_bitmap,
				PlayerB.frame_size);
		break;

	default:
		break;
	}
}

void LCD_Bitmap_Black(unsigned int x, unsigned int y, unsigned int width,
		unsigned int height, unsigned char bitmap[]) {
	LCD_CMD(0x02C); // write_memory_start
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

	SetWindows(x, y, x + width - 1, y + height - 1);
	unsigned int k = 0;

	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			// Obtener el color del sprite
			uint8_t high = bitmap[k];
			uint8_t low = bitmap[k + 1];
			uint16_t color = (high << 8) | low;

			int screen_x = x + i;
			int screen_y = y + j;

			// Verificar que está dentro del área visible
			if (screen_x >= 0&& screen_x < IMG_WIDTH &&
			screen_y >= 0 && screen_y < IMG_HEIGHT) {

				if (color == TRANSPARENT_COLOR) {
					// Pintar negro
					high = 0x00;
					low = 0x00;
				}

				LCD_DATA(high);
				LCD_DATA(low);
			} else {
				// Si está fuera de rango
				LCD_DATA(0x00);
				LCD_DATA(0x00);
			}

			k += 2;
		}
	}

	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void LCD_Bitmap_withTransparency(unsigned int x, unsigned int y,
		unsigned int width, unsigned int height, unsigned char bitmap[],
		uint8_t flip) {
	LCD_CMD(0x02C); // write_memory_start
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

	SetWindows(x, y, x + width - 1, y + height - 1);

	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			// Índice dependiendo de si se espejea o no
			int index_in_bitmap;
			if (flip == 0) {
				index_in_bitmap = (j * width + i) * 2;
			} else {
				index_in_bitmap = (j * width + (width - 1 - i)) * 2;
			}

			uint8_t high = bitmap[index_in_bitmap];
			uint8_t low = bitmap[index_in_bitmap + 1];
			uint16_t color = (high << 8) | low;

			int screen_x = x + i;
			int screen_y = y + j;

			if (screen_x >= 0&& screen_x < IMG_WIDTH &&
			screen_y >= 0 && screen_y < IMG_HEIGHT) {

				if (color == TRANSPARENT_COLOR) {
					int bg_index = (screen_y * IMG_WIDTH + screen_x) * 2;
					if (stage_actual_selection == 0) {
						high = Stage1.full_bitmap[bg_index];
						low = Stage1.full_bitmap[bg_index + 1];
					} else {
						high = Stage2.full_bitmap[bg_index];
						low = Stage2.full_bitmap[bg_index + 1];
					}
				}

				LCD_DATA(high);
				LCD_DATA(low);
			} else {
				LCD_DATA(0x00);
				LCD_DATA(0x00);
			}
		}
	}

	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void Rect_Grueso(unsigned int x, unsigned int y, unsigned int w, unsigned int h,
		unsigned int c) {
	for (int i = 0; i < 5; i++) {
		Rect(x - i, y - i, w + (2 * i), h + (2 * i), c);
	}
}
unsigned char* rotateBitmap90(const unsigned char *src, uint8_t flip) {
	static unsigned char rotated[19 * 19 * 2]; // 19x19 píxeles, 2 bytes por píxel
	int src_idx, dst_idx;

	for (int y = 0; y < 19; y++) {
		for (int x = 0; x < 19; x++) {
			src_idx = (y * 19 + x) * 2;
			uint8_t high = src[src_idx];
			uint8_t low = src[src_idx + 1];

			int new_x, new_y;
			if (flip == 1) {  // sentido horario (derecha)
				new_x = 18 - y;
				new_y = x;
			} else {  // sentido antihorario (izquierda)
				new_x = y;
				new_y = 18 - x;
			}

			dst_idx = (new_y * 19 + new_x) * 2;
			rotated[dst_idx] = high;
			rotated[dst_idx + 1] = low;
		}
	}
	return rotated;
}
unsigned char* rotateBitmap180(const unsigned char *src) {
	static unsigned char rotated[19 * 19 * 2]; // 19x19 píxeles, 2 bytes por píxel
	int src_idx, dst_idx;

	for (int y = 0; y < 19; y++) {
		for (int x = 0; x < 19; x++) {
			src_idx = (y * 19 + x) * 2;
			uint8_t high = src[src_idx];
			uint8_t low = src[src_idx + 1];

			// Coordenadas invertidas para 180°
			int new_x = 18 - x;
			int new_y = 18 - y;

			dst_idx = (new_y * 19 + new_x) * 2;
			rotated[dst_idx] = high;
			rotated[dst_idx + 1] = low;
		}
	}
	return rotated;
}
void animate_attack(void) {
	if (PlayerA.flip_frame) {
		CaptainShieldA.gadget_pos_x = PlayerA.pos_x + 40;
		LCD_Bitmap_withTransparency(CaptainShieldA.gadget_pos_x,
		GROUND_Y + 10, CaptainShieldA.frame_width, CaptainShieldA.frame_height,
				CaptainShieldA.actual_frame, 0);
		HAL_Delay(10);
		int actual_pos = PlayerA.pos_x;
		for (int i = 0; i < (270 - actual_pos - 20); i++) {
			// Cada 5 iteraciones, cambia la imagen

			if (i % 5 == 0) {
				int paso = (i / 5) % 4;  // Cicla entre 0 y 3
				switch (paso) {
				case 0:
					CaptainShieldA.actual_frame = GADGET_1_F1;
					LCD_Bitmap_withTransparency(CaptainShieldA.gadget_pos_x + i,
					GROUND_Y + 10, 19, 19, CaptainShieldA.actual_frame, 0);
					break;
				case 1:
					CaptainShieldA.rotated_frame = rotateBitmap90(GADGET_1_F1,
							0);
					LCD_Bitmap_withTransparency(CaptainShieldA.gadget_pos_x + i,
					GROUND_Y + 10, 19, 19, CaptainShieldA.rotated_frame,
							PlayerA.flip_frame);
					CaptainShieldA.actual_frame = CaptainShieldA.rotated_frame;
					break;
				case 2:
					CaptainShieldA.rotated_frame = rotateBitmap180(GADGET_1_F1);
					LCD_Bitmap_withTransparency(CaptainShieldA.gadget_pos_x + i,
					GROUND_Y + 10, 19, 19, CaptainShieldA.rotated_frame,
							PlayerA.flip_frame);
					CaptainShieldA.actual_frame = CaptainShieldA.rotated_frame;
					break;
				case 3:
					CaptainShieldA.rotated_frame = rotateBitmap90(GADGET_1_F1,
							1);
					LCD_Bitmap_withTransparency(CaptainShieldA.gadget_pos_x + i,
					GROUND_Y + 10, 19, 19, CaptainShieldA.rotated_frame,
							PlayerA.flip_frame);
					CaptainShieldA.actual_frame = CaptainShieldA.rotated_frame;
					break;
				}
			}
			LCD_Bitmap_withTransparency(CaptainShieldA.gadget_pos_x + i,
					GROUND_Y + 10, 19, 19, CaptainShieldA.actual_frame, 0);
			HAL_Delay(5);
		}
		LCD_Bitmap_withTransparency(PlayerB.pos_x, PlayerB.pos_y,
				PlayerB.frame_width, PlayerB.frame_height,
				PlayerB.walking1_bitmap, PlayerA.flip_frame);
		if (PlayerB.vida >= 25) {
			PlayerB.vida -= 25;
			LCD_Print("   ", 270, 0, 2, 0xF800, 0x0000);
			sprintf(life_text_buffer, "%u", PlayerB.vida);
			LCD_Print(life_text_buffer, 270, 0, 2, 0xF800, 0x0000);
		}
		if (PlayerB.vida < 25) {
			PlayerB.vida = 0;
			LCD_Print("   ", 270, 0, 2, 0xF800, 0x0000);
			sprintf(life_text_buffer, "%u", PlayerB.vida);
			LCD_Print(life_text_buffer, 270, 0, 2, 0xF800, 0x0000);
			LCD_Print("YOU WIN", 100, 120, 2, 0xFFFF, 0x0000);
			PlayerA.vida = 100;
			PlayerB.vida = 100;
		}

	} else {
		CaptainShieldA.gadget_pos_x = PlayerA.pos_x - 20;
		LCD_Bitmap_withTransparency(CaptainShieldA.gadget_pos_x, GROUND_Y + 10,
				19, 19, GADGET_1_F1, 0);
		HAL_Delay(10);
		int actual_pos = PlayerA.pos_x;
		for (int i = 0; i < (actual_pos - 20); i++) {
			// Cada 5 iteraciones, cambia la imagen
			if (i % 5 == 0) {
				int paso = (i / 5) % 4;  // Cicla entre 0 y 3
				switch (paso) {
				case 0:
					CaptainShieldA.actual_frame = GADGET_1_F1;
					break;
				case 1:
					CaptainShieldA.rotated_frame = rotateBitmap90(GADGET_1_F1,
							0);
					CaptainShieldA.actual_frame = CaptainShieldA.rotated_frame;
					break;
				case 2:
					CaptainShieldA.rotated_frame = rotateBitmap180(GADGET_1_F1);
					CaptainShieldA.actual_frame = CaptainShieldA.rotated_frame;
					break;
				case 3:
					CaptainShieldA.rotated_frame = rotateBitmap90(GADGET_1_F1,
							1);
					CaptainShieldA.actual_frame = CaptainShieldA.rotated_frame;
					break;
				}
			}

			// IMPORTANTE: aquí restamos `i` para ir hacia la izquierda
			LCD_Bitmap_withTransparency(CaptainShieldA.gadget_pos_x - i,
					GROUND_Y + 10, 19, 19, CaptainShieldA.actual_frame, 1); // El último parámetro puede ser `1` si PlayerA.flip_frame horizontal
			HAL_Delay(5);
		}
	}
}

void RefreshingCaracterOptions(void) {
	switch (PlayerA.actual_selection) {
	case 0:
		SD_readImageBin("cal.bin", caracter_logo_buffer, 50 * 50 * 2);
		LCD_Bitmap_Black(55, 95, 50, 50, caracter_logo_buffer);
		LCD_Print("Capitan  ", 0, 170, 2, 0xFFFF, 0x00);
		LCD_Print("America", 20, 190, 2, 0xFFFF, 0x00);
		LCD_Print("      ", 30, 210, 2, 0x0F0F, 0x00);
		break;
	case 1:
		SD_readImageBin("sml.bin", caracter_logo_buffer, 50 * 50 * 2);
		LCD_Bitmap_Black(55, 95, 50, 50, caracter_logo_buffer);
		LCD_Print("Superman  ", 0, 170, 2, 0xFFFF, 0x00);
		LCD_Print("       ", 20, 190, 2, 0x00, 0x00);
		LCD_Print("Locked", 30, 210, 2, 0x0F0F, 0x00);
		break;
	case 2:
		SD_readImageBin("bml.bin", caracter_logo_buffer, 50 * 50 * 2);
		LCD_Bitmap_Black(55, 95, 50, 50, caracter_logo_buffer);
		LCD_Print("Batman  ", 0, 170, 2, 0xFFFF, 0x00);
		LCD_Print("       ", 20, 190, 2, 0x00, 0x00);
		LCD_Print("Locked", 30, 210, 2, 0x0F0F, 0x00);
		break;
	case 3:
		//LCD_Bitmap_Black(55, 95, 50, 50, IRONMAN_LOGO);
		SD_readImageBin("iml.bin", caracter_logo_buffer, 50 * 50 * 2);
		LCD_Bitmap_Black(55, 95, 50, 50, caracter_logo_buffer);
		LCD_Print("Ironman  ", 0, 170, 2, 0xFFFF, 0x00);
		LCD_Print("       ", 20, 190, 2, 0x00, 0x00);
		LCD_Print("Locked", 30, 210, 2, 0x0F0F, 0x00);
		break;
	default:
		break;
	}
	switch (PlayerB.actual_selection) {
	case 0:
		SD_readImageBin("cal.bin", caracter_logo_buffer, 50 * 50 * 2);
		LCD_Bitmap_Black(215, 95, 50, 50, caracter_logo_buffer);
		LCD_Print("Capitan  ", 170, 170, 2, 0xFFFF, 0x00);
		LCD_Print("America", 190, 190, 2, 0xFFFF, 0x00);
		LCD_Print("      ", 200, 210, 2, 0x0F0F, 0x00);
		break;
	case 1:
		SD_readImageBin("sml.bin", caracter_logo_buffer, 50 * 50 * 2);
		LCD_Bitmap_Black(215, 95, 50, 50, caracter_logo_buffer);
		LCD_Print("Superman  ", 170, 170, 2, 0xFFFF, 0x00);
		LCD_Print("       ", 190, 190, 2, 0x00, 0x00);
		LCD_Print("      ", 200, 210, 2, 0x0F0F, 0x00);
		break;
	case 2:
		SD_readImageBin("bml.bin", caracter_logo_buffer, 50 * 50 * 2);
		LCD_Bitmap_Black(215, 95, 50, 50, caracter_logo_buffer);
		LCD_Print("Batman  ", 170, 170, 2, 0xFFFF, 0x00);
		LCD_Print("       ", 190, 190, 2, 0x00, 0x00);
		LCD_Print("      ", 200, 210, 2, 0x0F0F, 0x00);
		break;
	case 3:
		SD_readImageBin("iml.bin", caracter_logo_buffer, 50 * 50 * 2);
		LCD_Bitmap_Black(215, 95, 50, 50, caracter_logo_buffer);
		LCD_Print("Ironman  ", 170, 170, 2, 0xFFFF, 0x00);
		LCD_Print("       ", 190, 190, 2, 0x00, 0x00);
		LCD_Print("      ", 200, 210, 2, 0x0F0F, 0x00);
		break;
	default:
		break;
	}
	// Para PlayerA.actual_selection
	if (PlayerA.actual_selection == PlayerA.final_selection) {
		Rect_Grueso(55, 95, 50, 50, COLOR_VERDE);  // Seleccionado
	} else {
		Rect_Grueso(55, 95, 50, 50, COLOR_AZUL);   // Solo enfocado
	}

	// Para hero_2
	if (PlayerB.actual_selection == PlayerB.final_selection) {
		Rect_Grueso(215, 95, 50, 50, COLOR_VERDE);
	} else {
		Rect_Grueso(215, 95, 50, 50, COLOR_AZUL);
	}
}

void WaitingCaracterSelection(void) {
	if (PlayerA.confirm_selection && PlayerB.confirm_selection) {	// PRIORIDAD
		game_actual_state = SELECT_STAGE_INSTRUCT;
	} else if (input == 'R'
			&& PlayerA.actual_selection != PlayerA.final_selection) {
		PlayerA.actual_selection = (PlayerA.actual_selection + 1) % 4;
		game_actual_state = REFRESH_CARACTER_OPTIONS;
		input = 0;
	} else if (input == 'L'
			&& PlayerA.actual_selection != PlayerA.final_selection) {
		PlayerA.actual_selection =
				(PlayerA.actual_selection > 0) ?
						PlayerA.actual_selection - 1 : 3;
		game_actual_state = REFRESH_CARACTER_OPTIONS;
		input = 0;
	} else if (input == 'v'
			&& PlayerB.actual_selection != PlayerB.final_selection) {
		PlayerB.actual_selection = (PlayerB.actual_selection + 1) % 4;
		game_actual_state = REFRESH_CARACTER_OPTIONS;
		input = 0;
	} else if (input == 't'
			&& PlayerB.actual_selection != PlayerB.final_selection) {
		PlayerB.actual_selection =
				(PlayerB.actual_selection > 0) ?
						PlayerB.actual_selection - 1 : 3;
		game_actual_state = REFRESH_CARACTER_OPTIONS;
		input = 0;
	} else if (input == 'S') {
		if (PlayerA.actual_selection == 0) {// BLOQUEO TEMPORAL: NO HAY INFORMACION DE OTROS PERSONAJES
			PlayerA.final_selection = PlayerA.actual_selection;

			PlayerA.confirm_selection = true;
		}
		game_actual_state = REFRESH_CARACTER_OPTIONS;
		input = 0;
	} else if (input == 'q') {
		if (PlayerB.actual_selection == 0 || PlayerB.actual_selection == 1
				|| PlayerB.actual_selection == 2
				|| PlayerB.actual_selection == 3) {
			PlayerB.final_selection = PlayerB.actual_selection;

			PlayerB.confirm_selection = true;
		}
		game_actual_state = REFRESH_CARACTER_OPTIONS;
		input = 0;
	} else if (input == 'A') {
		PlayerA.final_selection = -1;
		PlayerA.confirm_selection = false;
		input = 0;
		game_actual_state = REFRESH_CARACTER_OPTIONS;
	} else if (input == 'y') {
		PlayerB.final_selection = -1;
		PlayerB.confirm_selection = false;
		input = 0;
		game_actual_state = REFRESH_CARACTER_OPTIONS;
	}

}

void RefreshingStageOptions(void) {
	switch (stage_actual_selection) {
	case 0:
		SD_readImageBin("st1.bin", stage_logo_buffer, 50 * 38 * 2);
		LCD_Bitmap_Black(Stage1.stage_pos_x, Stage1.stage_pos_y,
				Stage1.frame_width, Stage1.frame_height, stage_logo_buffer);
		LCD_Print("Taller Abandonado  ", 10, 170, 2, 0xFFFF, 0x00);
		Rect_Grueso(Stage1.stage_pos_x, Stage1.stage_pos_y, Stage1.frame_width,
				Stage1.frame_height, COLOR_AZUL);
		break;
	case 1:
		SD_readImageBin("st2.bin", stage_logo_buffer, 50 * 38 * 2);
		LCD_Bitmap_Black(Stage2.stage_pos_x, Stage2.stage_pos_y,
				Stage2.frame_width, Stage2.frame_height, stage_logo_buffer);
		LCD_Print("Ciudad Futuristica ", 10, 170, 2, 0xFFFF, 0x00);
		Rect_Grueso(Stage2.stage_pos_x, Stage2.stage_pos_y, Stage2.frame_width,
				Stage2.frame_height, COLOR_AZUL);
		break;
	}
}

void WaitingStageSelection(void) {
	if (input == 'A') {
		PlayerA.final_selection = -1;
		PlayerB.final_selection = -1;
		PlayerA.confirm_selection = false;
		PlayerB.confirm_selection = false;
		stage_actual_selection = 0;
		stage_confirm_selection = false;
		game_actual_state = SELECT_CARACTER_INSTRUCT;
		input = 0;
	} else if (input == 'R') {
		stage_actual_selection = 0;
		game_actual_state = REFRESH_STAGE_OPTIONS;
		input = 0;
	} else if (input == 'L') {
		stage_actual_selection = 1;
		game_actual_state = REFRESH_STAGE_OPTIONS;
		input = 0;
	} else if (input == 'S') {
		stage_confirm_selection = true;
		switch (stage_actual_selection) {
		case 0:
			Rect_Grueso(Stage1.stage_pos_x, Stage1.stage_pos_y,
					Stage1.frame_width, Stage1.frame_height, COLOR_VERDE);
			break;
		case 1:
			Rect_Grueso(Stage2.stage_pos_x, Stage2.stage_pos_y,
					Stage2.frame_width, Stage2.frame_height, COLOR_VERDE);
			break;
		}
		SD_info_2();
		game_actual_state = SET_CONDITIONS_SELECTED;
		input = 0;
	}
}

void SettingConditionsSelected(void) {
	LCD_Clear(0x00);
	if (stage_actual_selection == 0) {
		LCD_Bitmap(0, 0, IMG_WIDTH, IMG_HEIGHT, Stage1.full_bitmap);
	} else {
		LCD_Bitmap(0, 0, IMG_WIDTH, IMG_HEIGHT, Stage2.full_bitmap);
	}
	LCD_Bitmap_withTransparency(PlayerA.pos_x, PlayerA.pos_y,
			PlayerA.frame_width, PlayerA.frame_height, PlayerA.walking1_bitmap,
			PlayerA.flip_frame);

	LCD_Bitmap_withTransparency(PlayerB.pos_x, PlayerB.pos_y,
			PlayerB.frame_width, PlayerB.frame_height, PlayerB.walking1_bitmap,
			PlayerB.flip_frame);

	sprintf(life_text_buffer, "%u", PlayerA.vida);
	LCD_Print(life_text_buffer, 0, 0, 2, 0xF800, 0x0000);
	sprintf(life_text_buffer, "%u", PlayerB.vida);
	LCD_Print(life_text_buffer, 270, 0, 2, 0xF800, 0x0000);
	HAL_TIM_Base_Start_IT(&htim7);
}

void GameOnCurse(void) {
	/* Player A */
	// Command 'U'
	if (PlayerA.attack_static_flag) PlayerA_action_U();

	// Command 'R'
	if (PlayerA.attack_gadget_flag) PlayerA_action_R();

	// Command: 'D'
	if (PlayerA.defend_flag) PlayerA_action_D();

	// Command 'A'
	if (PlayerA.go_back_flag) PlayerA_action_A();

	// Command 'u'
	if (PlayerA.jump_flag) PlayerA_action_u();

	// Command 'd'
	if (PlayerA.bend_flag) PlayerA_action_d();

	// Command 'r'
	if (PlayerA.move_right_flag) PlayerA_action_r();

	// Command 'l'
	if (PlayerA.move_left_flag) PlayerA_action_l();

	// Command 'c'
	if (PlayerA.neutral_position_flag) PlayerA_action_c();

	/* Player B */
	// Command 'f'
	if (PlayerB.attack_static_flag) PlayerB_action_f();

	// Command 'v'
	if (PlayerB.attack_gadget_flag) PlayerB_action_v();

	// Command: 'h'
	if (PlayerB.defend_flag) PlayerB_action_h();

	// Command 'y'
	if (PlayerB.go_back_flag) PlayerB_action_y();

	// Command '8'
	if (PlayerB.jump_flag) PlayerB_action_8();

	// Command '2'
	if (PlayerB.bend_flag) PlayerB_action_2();

	// Command '6'
	if (PlayerB.move_right_flag) PlayerB_action_6();

	// Command '4'
	if (PlayerB.move_left_flag) PlayerB_action_4();

	// Command '5'
	if (PlayerB.neutral_position_flag) PlayerB_action_5();
	/*
	 if (input == 'v') { // ATAQUE OPONENTE (AHORA JUGADOR B)

	 LCD_Bitmap_withTransparency(270, 150, PlayerB.frame_width,
	 PlayerB.frame_height, PlayerB.walking2_bitmap, 1);
	 PlayerA.move_state = 1; // ERROR LOGIC
	 input = 0;
	 }
	 */
	HAL_TIM_Base_Start_IT(&htim7);
}

void PlayerA_action_U() {
	if (PlayerA.loaded_attack_static) {
		SD_readImageBin(PlayerA.attack_static_filename,
				PlayerA.attack_static_bitmap, PlayerA.frame_size);
		PlayerA.loaded_attack_static = 0;
	}
	LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
			PlayerA.frame_width, PlayerA.frame_height,
			PlayerA.attack_static_bitmap, PlayerA.flip_frame);
	PlayerA.attack_static_flag = 0;
	PlayerA.neutral_position_flag = 1;
}
void PlayerA_action_R() {
	if (PlayerA.loaded_attack_gadget) {
		SD_readImageBin(PlayerA.attack_gadget_filename,
				PlayerA.attack_gadget_bitmap, PlayerA.frame_size);
		PlayerA.loaded_attack_gadget = 0;
	}

	LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
			PlayerA.frame_width, PlayerA.frame_height,
			PlayerA.attack_gadget_bitmap, PlayerA.flip_frame);
	HAL_Delay(500);

	LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
			PlayerA.frame_width, PlayerA.frame_height,
			PlayerA.attack_gadget_bitmap, PlayerA.flip_frame);
	animate_attack();
	PlayerA.attack_gadget_flag = 0;
	PlayerA.neutral_position_flag = 1;
}
void PlayerA_action_D() {
	if (PlayerA.loaded_defend) {
		SD_readImageBin(PlayerA.defense_filename, PlayerA.defense_bitmap,
				PlayerA.frame_size);
		PlayerA.loaded_defend = 0;
	}
	LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
			PlayerA.frame_width, PlayerA.frame_height,
			PlayerA.defense_bitmap, PlayerA.flip_frame);
	PlayerA.defense = 1;
	PlayerA.defend_flag = 0;
	PlayerA.neutral_position_flag = 1;
}
void PlayerA_action_A() {
	PlayerA.final_selection = -1;
	PlayerB.final_selection = -1;
	PlayerA.confirm_selection = false;
	PlayerB.confirm_selection = false;
	stage_actual_selection = 0;
	stage_confirm_selection = false;
	game_actual_state = SELECT_CARACTER_INSTRUCT;
	PlayerA.vida = 100;
	PlayerB.vida = 100;
	PlayerA.go_back_flag = 0;
	return;
}
void PlayerA_action_u() {
	if (PlayerA.loaded_jump) {
		SD_readImageBin(PlayerA.jump_filename, PlayerA.jump_bitmap,
				PlayerA.frame_size);
		PlayerA.loaded_jump = 0;
	}
	if (PlayerA.jump_delay_counter >= 2) { // 2 ciclos ~30ms si tu timer es cada 15ms
		PlayerA.jump_delay_counter = 0;

		// Actualizar posición
		PlayerA.jump_y += JUMP_STEP * PlayerA.jump_direction;

		// Dibujar personaje en nueva posición
		LCD_Bitmap_withTransparency(PlayerA.pos_x, PlayerA.jump_y,
				PlayerA.frame_width, PlayerA.frame_height,
				PlayerA.jump_bitmap, PlayerA.flip_frame);

		// Ver si llegó arriba del salto
		if (PlayerA.jump_y <= JUMP_HEIGHT) {
			PlayerA.jump_direction = 1;  // Ahora baja
		}
		// Ver si ya cayó al suelo
		if (PlayerA.jump_y >= GROUND_Y) {
			PlayerA.jump_y = GROUND_Y;
			PlayerA.jump_direction = -1;
			PlayerA.neutral_position_flag = 1;
			PlayerA.jump_flag = 0;
		}
	}
}
void PlayerA_action_d() {
	if (PlayerA.loaded_bend) {
		SD_readImageBin(PlayerA.down_filename, PlayerA.down_bitmap,
				PlayerA.frame_size);
		PlayerA.loaded_bend = 0;
	}
	LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
			PlayerA.frame_width, PlayerA.frame_height, PlayerA.down_bitmap,
			PlayerA.flip_frame);
}
void PlayerA_action_r() {
	if (PlayerA.loaded_move_right) {
		SD_readImageBin(PlayerA.walking1_filename, PlayerA.walking1_bitmap,
				PlayerA.frame_size);
		SD_readImageBin(PlayerA.walking2_filename, PlayerA.walking2_bitmap,
				PlayerA.frame_size);
		PlayerA.loaded_move_right = 0;
	}
	if (PlayerA.delay_right_move >= 5) {
		PlayerA.flip_frame = 1;
		if (PlayerA.pos_x + PlayerA.frame_switch_distance <= MAX_X) {
			PlayerA.pos_x += PlayerA.frame_switch_distance;
		}
		PlayerA.step_counter_right = (PlayerA.step_counter_right + 1) % 6;
		if (PlayerA.step_counter_right == 0)
			PlayerA.walking_frame = !PlayerA.walking_frame;

		uint8_t *frame =
				PlayerA.walking_frame ?
						PlayerA.walking1_bitmap : PlayerA.walking2_bitmap;
		LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
				PlayerA.frame_width, PlayerA.frame_height, frame,
				PlayerA.flip_frame);
		PlayerA.delay_right_move = 0;
	}
}
void PlayerA_action_l() {
	if (PlayerA.loaded_move_left) {
		SD_readImageBin(PlayerA.walking1_filename, PlayerA.walking1_bitmap,
				PlayerA.frame_size);
		SD_readImageBin(PlayerA.walking2_filename, PlayerA.walking2_bitmap,
				PlayerA.frame_size);
		PlayerA.loaded_move_left = 0;
	}
	if (PlayerA.delay_left_move >= 5) {
		PlayerA.flip_frame = 0;
		if (PlayerA.pos_x >= PlayerA.frame_switch_distance) {
			PlayerA.pos_x -= PlayerA.frame_switch_distance;
		}
		PlayerA.step_counter_left = (PlayerA.step_counter_left + 1) % 6;
		if (PlayerA.step_counter_left == 0)
			PlayerA.walking_frame = !PlayerA.walking_frame;

		uint8_t *frame =
				PlayerA.walking_frame ?
						PlayerA.walking1_bitmap : PlayerA.walking2_bitmap;
		LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
				PlayerA.frame_width, PlayerA.frame_height, frame,
				PlayerA.flip_frame);

		PlayerA.delay_left_move = 0;
	}
}
void PlayerA_action_c() {
	if (PlayerA.loaded_neutral_position) {
		SD_readImageBin(PlayerA.walking1_filename, PlayerA.walking1_bitmap,
				PlayerA.frame_size);
		SD_readImageBin(PlayerA.walking2_filename, PlayerA.walking2_bitmap,
				PlayerA.frame_size);
		PlayerA.loaded_neutral_position = 0;
	}
	LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
			PlayerA.frame_width, PlayerA.frame_height,
			PlayerA.walking1_bitmap, PlayerA.flip_frame);
	PlayerA.neutral_position_flag = 0;
	PlayerA.move_left_flag = 0;
	PlayerA.move_right_flag = 0;

	PlayerA.bend_flag = 0;
}

void PlayerB_action_f() {
	if (PlayerB.loaded_attack_static) {
		SD_readImageBin(PlayerB.attack_static_filename,
				PlayerB.attack_static_bitmap, PlayerB.frame_size);
		PlayerB.loaded_attack_static = 0;
	}
	LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
			PlayerB.frame_width, PlayerB.frame_height,
			PlayerB.attack_static_bitmap, PlayerB.flip_frame);
	PlayerB.attack_static_flag = 0;
	PlayerB.neutral_position_flag = 1;
}
void PlayerB_action_v() {
	if (PlayerB.loaded_attack_gadget) {
		SD_readImageBin(PlayerB.attack_gadget_filename,
				PlayerB.attack_gadget_bitmap, PlayerB.frame_size);
		PlayerB.loaded_attack_gadget = 0;
	}

	LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
			PlayerB.frame_width, PlayerB.frame_height,
			PlayerB.attack_gadget_bitmap, PlayerB.flip_frame);
	HAL_Delay(500);

	LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
			PlayerB.frame_width, PlayerB.frame_height,
			PlayerB.attack_gadget_bitmap, PlayerB.flip_frame);
	// Iniciar movimiento automático del gadget B
	PlayerB.move_state = 1;
	CaptainShieldB.gadget_last_x = 0;
	PlayerB.attack_gadget_flag = 0;
	PlayerB.neutral_position_flag = 1;
}
void PlayerB_action_h() {
	if (PlayerB.loaded_defend) {
		SD_readImageBin(PlayerB.defense_filename, PlayerB.defense_bitmap,
				PlayerB.frame_size);
		PlayerB.loaded_defend = 0;
	}
	LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
			PlayerB.frame_width, PlayerB.frame_height,
			PlayerB.defense_bitmap, PlayerB.flip_frame);
	PlayerB.defense = 1;
	PlayerB.defend_flag = 0;
	PlayerB.neutral_position_flag = 1;
}
void PlayerB_action_y() {
	PlayerA.final_selection = -1;
	PlayerB.final_selection = -1;
	PlayerA.confirm_selection = false;
	PlayerB.confirm_selection = false;
	stage_actual_selection = 0;
	stage_confirm_selection = false;
	game_actual_state = SELECT_CARACTER_INSTRUCT;
	PlayerA.vida = 100;
	PlayerB.vida = 100;
	PlayerA.go_back_flag = 0;
	return;
}
void PlayerB_action_8() {
	if (PlayerB.loaded_jump) {
		SD_readImageBin(PlayerB.jump_filename, PlayerB.jump_bitmap,
				PlayerB.frame_size);
		PlayerB.loaded_jump = 0;
	}
	if (PlayerB.jump_delay_counter >= 2) { // 2 ciclos ~30ms si tu timer es cada 15ms
		PlayerB.jump_delay_counter = 0;

		// Actualizar posición
		PlayerB.jump_y += JUMP_STEP * PlayerB.jump_direction;

		// Dibujar personaje en nueva posición
		LCD_Bitmap_withTransparency(PlayerB.pos_x, PlayerB.jump_y,
				PlayerB.frame_width, PlayerB.frame_height,
				PlayerB.jump_bitmap, PlayerB.flip_frame);

		// Ver si llegó arriba del salto
		if (PlayerB.jump_y <= JUMP_HEIGHT) {
			PlayerB.jump_direction = 1;  // Ahora baja
		}
		// Ver si ya cayó al suelo
		if (PlayerB.jump_y >= GROUND_Y) {
			PlayerB.jump_y = GROUND_Y;
			PlayerB.jump_direction = -1;
			PlayerB.neutral_position_flag = 1;
			PlayerB.jump_flag = 0;
		}
	}
}
void PlayerB_action_2() {
	if (PlayerB.loaded_bend) {
		SD_readImageBin(PlayerB.down_filename, PlayerB.down_bitmap,
				PlayerB.frame_size);
		PlayerB.loaded_bend = 0;
	}
	LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
			PlayerB.frame_width, PlayerB.frame_height, PlayerB.down_bitmap,
			PlayerB.flip_frame);
}
void PlayerB_action_6() {
	if (PlayerB.loaded_move_right) {
		SD_readImageBin(PlayerB.walking1_filename, PlayerB.walking1_bitmap,
				PlayerB.frame_size);
		SD_readImageBin(PlayerB.walking2_filename, PlayerB.walking2_bitmap,
				PlayerB.frame_size);
		PlayerB.loaded_move_right = 0;
	}
	if (PlayerB.delay_right_move >= 5) {
		PlayerB.flip_frame = 1;
		if (PlayerB.pos_x + PlayerB.frame_switch_distance <= MAX_X) {
			PlayerB.pos_x += PlayerB.frame_switch_distance;
		}
		PlayerB.step_counter_right = (PlayerB.step_counter_right + 1) % 6;
		if (PlayerB.step_counter_right == 0)
			PlayerB.walking_frame = !PlayerB.walking_frame;

		uint8_t *frame =
				PlayerB.walking_frame ?
						PlayerB.walking1_bitmap : PlayerB.walking2_bitmap;
		LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
				PlayerB.frame_width, PlayerB.frame_height, frame,
				PlayerB.flip_frame);
		PlayerB.delay_right_move = 0;
	}
}
void PlayerB_action_4() {
	if (PlayerB.loaded_move_left) {
		SD_readImageBin(PlayerB.walking1_filename, PlayerB.walking1_bitmap,
				PlayerB.frame_size);
		SD_readImageBin(PlayerB.walking2_filename, PlayerB.walking2_bitmap,
				PlayerB.frame_size);
		PlayerB.loaded_move_left = 0;
	}
	if (PlayerB.delay_left_move >= 5) {
		PlayerB.flip_frame = 0;
		if (PlayerB.pos_x >= PlayerB.frame_switch_distance) {
			PlayerB.pos_x -= PlayerB.frame_switch_distance;
		}
		PlayerB.step_counter_left = (PlayerB.step_counter_left + 1) % 6;
		if (PlayerB.step_counter_left == 0)
			PlayerB.walking_frame = !PlayerB.walking_frame;

		uint8_t *frame =
				PlayerB.walking_frame ?
						PlayerB.walking1_bitmap : PlayerB.walking2_bitmap;
		LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
				PlayerB.frame_width, PlayerB.frame_height, frame,
				PlayerB.flip_frame);

		PlayerB.delay_left_move = 0;
	}
}
void PlayerB_action_5() {
	if (PlayerB.loaded_neutral_position) {
		SD_readImageBin(PlayerB.walking1_filename, PlayerB.walking1_bitmap,
				PlayerB.frame_size);
		SD_readImageBin(PlayerB.walking2_filename, PlayerB.walking2_bitmap,
				PlayerB.frame_size);
		PlayerB.loaded_neutral_position = 0;
	}
	LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
			PlayerB.frame_width, PlayerB.frame_height,
			PlayerB.walking1_bitmap, PlayerB.flip_frame);
	PlayerB.neutral_position_flag = 0;
	PlayerB.move_left_flag = 0;
	PlayerB.move_right_flag = 0;

	PlayerB.bend_flag = 0;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim == &htim7) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		if (game_actual_state == GAME_ON_CURSE) {
			if (PlayerA.move_left_flag) PlayerA.delay_left_move++;
			if (PlayerA.move_right_flag) PlayerA.delay_right_move++;
			if (PlayerA.bend_flag);
			if (PlayerA.jump_flag) PlayerA.jump_delay_counter++;
			if (PlayerA.defend_flag);

			if (PlayerB.move_left_flag) PlayerB.delay_left_move++;
			if (PlayerB.move_right_flag) PlayerB.delay_right_move++;
			if (PlayerB.jump_flag) PlayerB.jump_delay_counter++;


			if (PlayerA.move_state == 1) {
				if (CaptainShieldA.gadget_last_x <= 200) {
					CaptainShieldA.gadget_last_x++;
					LCD_Bitmap_withTransparency(
							270 - 20 - CaptainShieldA.gadget_last_x,
							GROUND_Y, CaptainShieldA.frame_width,
							CaptainShieldA.frame_height,
							PlayerB.attack_gadget_bitmap, 1);
				} else {
					LCD_Bitmap_withTransparency(270, 150, PlayerB.frame_width,
							PlayerB.frame_height, PlayerB.walking1_bitmap, 1);
					CaptainShieldA.gadget_last_x = 0;
					PlayerA.move_state = 0;
				}
			}

			if (PlayerB.move_state == 1) {
				if (CaptainShieldB.gadget_last_x <= 200) {
					CaptainShieldB.gadget_last_x++;
					LCD_Bitmap_withTransparency(
						PlayerB.pos_x - CaptainShieldB.gadget_last_x,
						GROUND_Y + 10, CaptainShieldB.frame_width, CaptainShieldB.frame_height,
						CaptainShieldB.actual_frame, 1);
				} else {
					CaptainShieldB.gadget_last_x = 0;
					PlayerB.move_state = 0;
					LCD_Bitmap_withTransparency(PlayerB.pos_x, GROUND_Y,
						PlayerB.frame_width, PlayerB.frame_height,
						PlayerB.walking1_bitmap, PlayerB.flip_frame);
				}
			}


			// Colisión gadget A sobre PlayerB
			if ((270 - CaptainShieldA.gadget_last_x) == (PlayerA.pos_x + 40)
					&& PlayerA.defense == 0) {
				if (PlayerA.vida >= 25) {
					PlayerA.vida -= 25;
					LCD_Print("   ", 0, 0, 2, 0xF800, 0x0000);
					sprintf(life_text_buffer, "%u", PlayerA.vida);
					LCD_Print(life_text_buffer, 0, 0, 2, 0xF800, 0x0000);
				}
				if (PlayerA.vida < 25) {
					PlayerA.vida = 0;
					LCD_Print("   ", 0, 0, 2, 0xF800, 0x0000);
					sprintf(life_text_buffer, "%u", PlayerA.vida);
					LCD_Print(life_text_buffer, 0, 0, 2, 0xF800, 0x0000);

					LCD_Print("GAME OVER", 100, 120, 2, 0xFFFF, 0x0000);
					PlayerA.vida = 100;
					PlayerB.vida = 100;
				}

				LCD_Bitmap_withTransparency(270, 150, PlayerB.frame_width,
						PlayerB.frame_height, PlayerB.walking1_bitmap, 1);
				CaptainShieldA.gadget_last_x = 0;
				PlayerA.move_state = 0;
				LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
						PlayerA.frame_width, PlayerA.frame_height,
						PlayerA.walking1_bitmap, PlayerA.flip_frame);
				// recupera vida
			} else if ((270 - CaptainShieldA.gadget_last_x - 20)
					== (PlayerA.pos_x + 40) && PlayerA.defense == 1) {
				if (PlayerA.vida <= 95) {
					PlayerA.vida += 5;
				}
				LCD_Print("   ", 0, 0, 2, 0xF800, 0x0000);
				sprintf(life_text_buffer, "%u", PlayerA.vida);
				LCD_Print(life_text_buffer, 0, 0, 2, 0xF800, 0x0000);

				LCD_Bitmap_withTransparency(270, 150, PlayerB.frame_width,
						PlayerB.frame_height, PlayerB.walking1_bitmap, 1);
				CaptainShieldA.gadget_last_x = 0;
				PlayerA.move_state = 0;
				LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
						PlayerA.frame_width, PlayerA.frame_height,
						PlayerA.walking1_bitmap, PlayerA.flip_frame);
			}

			// Colisión gadget B sobre PlayerA
			if ((PlayerB.pos_x - CaptainShieldB.gadget_last_x) == (PlayerA.pos_x)
				&& PlayerA.defense == 0) {

				if (PlayerA.vida >= 25) {
					PlayerA.vida -= 25;
				}
				else {
					PlayerA.vida = 0;
				}

				// Actualiza texto de vida
				LCD_Print("   ", 0, 0, 2, 0xF800, 0x0000);
				sprintf(life_text_buffer, "%u", PlayerA.vida);
				LCD_Print(life_text_buffer, 0, 0, 2, 0xF800, 0x0000);

				// Verifica si perdió
				if (PlayerA.vida == 0) {
					LCD_Print("GAME OVER", 100, 120, 2, 0xFFFF, 0x0000);
					PlayerA.vida = 100;
					PlayerB.vida = 100;
				}

				// Restaurar sprite del jugador
				LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
					PlayerA.frame_width, PlayerA.frame_height,
					PlayerA.walking1_bitmap, PlayerA.flip_frame);

				// Detener gadget
				CaptainShieldB.gadget_last_x = 0;
				PlayerB.move_state = 0;

			} else if ((PlayerB.pos_x - CaptainShieldB.gadget_last_x - 20) == (PlayerA.pos_x + 40)
				&& PlayerA.defense == 1) {

				if (PlayerA.vida <= 95) {
					PlayerA.vida += 5;
				}

				LCD_Print("   ", 0, 0, 2, 0xF800, 0x0000);
				sprintf(life_text_buffer, "%u", PlayerA.vida);
				LCD_Print(life_text_buffer, 0, 0, 2, 0xF800, 0x0000);

				// Restaurar sprite del jugador
				// Restaurar fondo donde estaba el escudo
				uint16_t gadget_x = PlayerB.pos_x - CaptainShieldB.gadget_last_x;
				LCD_Bitmap_withTransparency(gadget_x, GROUND_Y + 10,
					CaptainShieldB.frame_width, CaptainShieldB.frame_height,
					CaptainShieldB.actual_frame, 1);  // Restaurar con transparencia

				// Redibujar al jugador A
				LCD_Bitmap_withTransparency(PlayerA.pos_x, GROUND_Y,
					PlayerA.frame_width, PlayerA.frame_height,
					PlayerA.walking1_bitmap, PlayerA.flip_frame);


				// Detener gadget
				CaptainShieldB.gadget_last_x = 0;
				PlayerB.move_state = 0;
			}



		}

//		HAL_TIM_Base_Start_IT(&htim7);
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
//
  /* USER CODE END Callback 1 */
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
