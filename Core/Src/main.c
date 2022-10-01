/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PinHigh GPIO_PIN_SET
#define PinLow GPIO_PIN_RESET
#define USHRT_MAX 65535
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SetPinState(channel, pin, state) (HAL_GPIO_WritePin(channel, pin, state))
//#define ClearCube() memset(cubeVexels, 0, sizeof(cubeVexels))
#define ClearCubeBytes() memset(cubeBufferBytes, 0, sizeof(cubeBufferBytes))
#define SetVoxel2(x, y) (cubeMatrix[7 - y] |= 0x01 << x)
#define TurnBitOn(number, n) (*number |= 1 << *n)
#define TurnBitOff(number, n) (number &= ~(1 << n))
#define SetFalseBits(number, n) (number &= n)
#define SetTrueBits(number, n) (number |= n)
#define StraightLine(pA, pB, xyz, t, l) (byte)(pA.xyz + round((pB.xyz - pA.xyz) * ((float)t/(float)l)))
#define ApplyBufferToRender() memcpy(cubeBytes, cubeBufferBytes, sizeof(cubeBufferBytes));
#define FPS 10


#define DL2(a, b) Draw3DLine(a, b, 0)
#define DL3(a, b, c) Draw3DLine(a, b, c)
#define DL_SELECT(_1, _2, _3, macro, ...) macro
#define DrawLine(args...) DL_SELECT(args, DL3, DL2, 1)(args)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
ubyte cubeBytes[10][13];
ubyte cubeBufferBytes[10][13];
ubyte cubeLayerBytes[2];
byte timerTicks;


Point3 corner1 = {0, 1, 9};
Point3 corner2 = {0, 9, 9};
Point3 corner3 = {9, 1, 9};
Point3 corner4 = {9, 9, 9};

Point3 corner5 = {0, 1, 0};
Point3 corner6 = {0, 9, 0};
Point3 corner7 = {9, 1, 0};
Point3 corner8 = {9, 9, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Ping_Latch()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, PinHigh);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, PinLow);
}
void SetVoxelByXYZptrs(byte* x, byte* y, byte* z)
{
	int k = *x * 100 + *y * 10 + *z;
	int n = ((k + 2 - *x * 100) % 8);
	TurnBitOn(&cubeBufferBytes[*x][((k + 2 - *x * 100) / 8)], &n);
}
void SetVoxelByXYZ(byte x, byte y, byte z)
{
	if(x > 9 || x < 0 || y > 9 || y < 0 || z > 9 || z <0)
		return;
	int k = x * 100 + y * 10 + z;
	int n = ((k + 2 - x * 100) % 8);
	TurnBitOn(&cubeBufferBytes[x][((k + 2 - x * 100) / 8)], &n);
}


void Draw3DLine(Point3 a, Point3 b, byte excludeEndPoints)
{
	byte length = 0;
	byte l1 = (byte)abs(a.x - b.x);
	byte l2 = (byte)abs(a.y - b.y);
	byte l3 = (byte)abs(a.z - b.z);
	length = l1;
	if(l2 > l1)
		length = l2;
	if(l3 > l2)
		length = l3;
	byte x,y,z;
	if(excludeEndPoints == 0)
	{
		SetVoxelByXYZ(a.x, a.y, a.z);
		SetVoxelByXYZ(b.x, b.y, b.z);
	}
	for (byte i = 1; i < length; i++)
	{
		x = StraightLine(a, b, x, i, length);// (byte)(a->x + (b->x - a->x) * i / length);
		y = StraightLine(a, b, y, i, length);
		z = StraightLine(a, b, z, i, length);
		SetVoxelByXYZ(x, y, z);
	}
}

void drawCirclePointsByXYZP(int x, int y, Point2d center, byte layerIndex, figure2dOrientation co)
{
	switch(co)
	{
		case figure2dOrientationXY:
		{
			SetVoxelByXYZ(x + center.x, y + center.y, layerIndex);
			SetVoxelByXYZ(y + center.x, x + center.y, layerIndex);
			SetVoxelByXYZ(y + center.x, -x + center.y, layerIndex);
			SetVoxelByXYZ(x + center.x, -y + center.y, layerIndex);
			SetVoxelByXYZ(-x + center.x, -y + center.y, layerIndex);
			SetVoxelByXYZ(-y + center.x, -x + center.y, layerIndex);
			SetVoxelByXYZ(-y + center.x, x + center.y, layerIndex);
			SetVoxelByXYZ(-x + center.x, y + center.y, layerIndex);
			break;
		}
		case figure2dOrientationYZ:
		{
			SetVoxelByXYZ(layerIndex, x + center.x, y + center.y);
			SetVoxelByXYZ(layerIndex, y + center.x, x + center.y);
			SetVoxelByXYZ(layerIndex, y + center.x, -x + center.y);
			SetVoxelByXYZ(layerIndex, x + center.x, -y + center.y);
			SetVoxelByXYZ(layerIndex, -x + center.x, -y + center.y);
			SetVoxelByXYZ(layerIndex, -y + center.x, -x + center.y);
			SetVoxelByXYZ(layerIndex, -y + center.x, x + center.y);
			SetVoxelByXYZ(layerIndex, -x + center.x, y + center.y);
			break;
		}
		case figure2dOrientationXZ:
		{
			SetVoxelByXYZ(x + center.x, layerIndex, y + center.y);
			SetVoxelByXYZ(y + center.x, layerIndex, x + center.y);
			SetVoxelByXYZ(y + center.x, layerIndex, -x + center.y);
			SetVoxelByXYZ(x + center.x, layerIndex, -y + center.y);
			SetVoxelByXYZ(-x + center.x, layerIndex, -y + center.y);
			SetVoxelByXYZ(-y + center.x, layerIndex, -x + center.y);
			SetVoxelByXYZ(-y + center.x, layerIndex, x + center.y);
			SetVoxelByXYZ(-x + center.x, layerIndex, y + center.y);
			break;
		}
		default:
			break;
	}
}
/*
void DrawCircle(Point2d center, double raduis, byte layerIndex, figure2dOrientation co)
{
	int x = 0;
	int y = (int)(raduis);
	double d = 1 - raduis;
	int p = 3;
	double q = -2 * raduis + 5;
	drawCirclePointsByXYZP(x, y, center, layerIndex, co);
	while (y > x)
	{
		if (d < 0)
		{
			d += p;
			p += 2;
			q += 2;
			x += 1;
		}
		else
		{
			d += q;
			p += 2;
			q += 4;
			x += 1;
			y -= 1;
		}
		drawCirclePointsByXYZP(x, y, center, layerIndex, co);
	}
}
*/
void drawSquare(Point3 topLeft, byte size, figure2dOrientation o)
{
	Point3 c1;
	Point3 c2;
	Point3 c3;
	Point3 c4;
	switch(o)
	{
		case figure2dOrientationXY:
		{
			for(byte i = 0; i < 2; i++)
			{
				c1 = (Point3){topLeft.x, topLeft.y, topLeft.z + (i * size)};
				c2 = (Point3){topLeft.x + size, topLeft.y, topLeft.z + (i * size)};
				c3 = (Point3){topLeft.x, topLeft.y + size, topLeft.z + (i * size)};
				c4 = (Point3){topLeft.x + size, topLeft.y + size, topLeft.z + (i * size)};
				//Draw3DLine(c1, c2, 0);
				DrawLine(c1,c2);
				//Draw3DLine(c2, c4, 0);
				DrawLine(c2,c4);
				//Draw3DLine(c4, c3, 0);
				DrawLine(c4,c3);
				//Draw3DLine(c1, c3, 0);
				DrawLine(c1,c3);
			}
			break;
		}
		case figure2dOrientationYZ:
		{
			for(byte i = 0; i < 2; i++)
			{
				c1 = (Point3){topLeft.x + (i * size), topLeft.y, topLeft.z};
				c2 = (Point3){topLeft.x + (i * size), topLeft.y + size, topLeft.z};
				c3 = (Point3){topLeft.x + (i * size), topLeft.y, topLeft.z + size};
				c4 = (Point3){topLeft.x + (i * size), topLeft.y + size, topLeft.z + size};
				//Draw3DLine(c1, c2, 0);
				DrawLine(c1,c2);
				//Draw3DLine(c2, c4, 0);
				DrawLine(c2,c4);
				//Draw3DLine(c4, c3, 0);
				DrawLine(c4,c3);
				//Draw3DLine(c1, c3, 0);
				DrawLine(c1,c3);
			}
			break;
		}
		case figure2dOrientationXZ:
		{
			for(byte i = 0; i < 2; i++)
			{
				c1 = (Point3){topLeft.x, topLeft.y + (i * size), topLeft.z};
				c2 = (Point3){topLeft.x + size, topLeft.y + (i * size), topLeft.z};
				c3 = (Point3){topLeft.x, topLeft.y + (i * size), topLeft.z + size};
				c4 = (Point3){topLeft.x + size, topLeft.y + (i * size), topLeft.z + size};
				//Draw3DLine(c1, c2, 0);
				DrawLine(c1,c2);
				//Draw3DLine(c2, c4, 0);
				DrawLine(c2,c4);
				//Draw3DLine(c4, c3, 0);
				DrawLine(c4,c3);
				//Draw3DLine(c1, c3, 0);
				DrawLine(c1,c3);
			}
			break;
		}
		default:
			break;
	}
}

void DrawCube(Point3 leftTopZ, byte size)
{
	drawSquare(leftTopZ, size, figure2dOrientationXY);
	drawSquare(leftTopZ, size, figure2dOrientationYZ);
	drawSquare(leftTopZ, size, figure2dOrientationXZ);
}


void Render(byte i)
{
	switch(i)
	{
		case 8:
		{
			cubeLayerBytes[1] = 0;
			cubeLayerBytes[0] = cubeBytes[i][12] & 0b00111111;
			cubeLayerBytes[0] |= 0b10000000;
			break;
		}
		case 9:
		{
			cubeLayerBytes[1] = 0;
			cubeLayerBytes[0] = cubeBytes[i][12] & 0b00111111;
			cubeLayerBytes[0] |= 0b01000000;
			break;
		}
		default:
		{
			cubeLayerBytes[0] = cubeBytes[i][12] & 0b00111111;
			cubeLayerBytes[1] = (1 << (7 -i));
			break;
		}
	}
	// ------------------------------------------------------------------------------------------
	// ------------------------------------------------------------------------------------------
	HAL_SPI_Transmit(&hspi1, (uint8_t *)cubeBytes[i], 12, 100);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)cubeLayerBytes, 2, 100);
	Ping_Latch();
}

void Redraw()
{
	byte plusZZ = 1;
	byte upDown = 1;
	double radius = 1.5;

	//Point2d center = {5.5, 5.5};
	Point3 lefttopZ;
	byte step = 1;
	byte cubeSize = 9;
	byte cubeStep = -2;
	byte i = 0;
	for(;;)
	{
	ClearCubeBytes();

	//Draw3DLine(corner1, corner2, 0);
	//Draw3DLine(corner1, corner3, 0);
	//Draw3DLine(corner3, corner4, 0);
	//Draw3DLine(corner4, corner2, 0);
	//Draw3DLine(corner5, corner6, 0);
	//Draw3DLine(corner5, corner7, 0);
	//Draw3DLine(corner7, corner8, 0);
	//Draw3DLine(corner8, corner6, 0);
	//Draw3DLine(corner4, corner8, 1);
	//Draw3DLine(corner2, corner6, 1);
	//Draw3DLine(corner3, corner7, 1);
	//Draw3DLine(corner1, corner5, 1);

	lefttopZ = (Point3){i, i, i};
	DrawCube(lefttopZ, cubeSize);

	if(i + step == 5)
	{
		cubeStep = 2;
		step = -1;
	}
	if(i == 0)
	{

		cubeStep = -2;
		step = 1;
	}

	i+=step;
	cubeSize += cubeStep;

	//DrawCube(lefttopZ, 5);
	//for(byte xx = 0; xx < 3; xx++)
	//{
	//	for(byte zz = 5; zz < 6; zz++)
	//	{
			//DrawCircle(center, radius + upDown, 5, 1);
	//	}
	//}

	ApplyBufferToRender();
	if(radius + (float)upDown >= 4)
	{
		plusZZ = -1;
	}
	if(upDown <= 0)
	{
		plusZZ = 1;
	}
	//upDown += plusZZ;
	//osDelay(200);
	HAL_Delay(50);
	//osDelay(1000 / FPS);
	}
}

byte  Randon_Number(byte lower, byte upper)
{
	return((rand() % (upper - lower + 1)) + lower);
}

int f(int a, int b){
	return a+b;
}

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
  srand(time(NULL));

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  srand(time(NULL));
  timerTicks = 0;

  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Redraw();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Controll_Button_Pin */
  GPIO_InitStruct.Pin = Controll_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Controll_Button_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
		if (htim->Instance == TIM2) {

			//DWT_Delay_us(500);
			Render(timerTicks++);
			if(timerTicks == 10)
				timerTicks = 0;
		  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

		if (htim->Instance == TIM3) {
		  }
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
  while (1)
  {
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
