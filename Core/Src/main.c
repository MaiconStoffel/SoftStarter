/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SoftStarter with STM32CubeMonitor IHM.
  * @author 	    : Maicon Stoffel / 4422 / 2024
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t testeADC = 0;
uint8_t botao=0, rx_data[3] = {0,0,0}, feliz=0, multiplica=100;
uint8_t tx_data[1] = { 49 };  // 49 é o valor ASCII para o caractere '1'
float voltage = 0;
char buffer[50];

//RAMPAS

int ANGULO_MIN=30;  //ANGULO QUE A SENOIDA VAI BATER QUANDO VAI A ESQUERDA
int ANGULO_MAX=170; //ANGULO MAIS A DIREITA "PARTIDA"
int tempoSubida=10; //setando o tempo de 10s como "padrao" para caso o usuario ligue direto
int tempoDescida=10; //setando o tempo de 10s como "padrao" para caso o usuario ligue direto

//POTENCIA

int angulo = 170;
uint32_t ARR = 0, CCR = 0, AutoReload=0;
uint8_t borda=0;
int estado_motor=0;
int bypass_ativo=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

/*
 * mantem o valor escolhido do angulo_max pela IHM diretamente no valor de angulo
 * utilizado no codigo e no circuito.
 */

    angulo = ANGULO_MAX;

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);



  //HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1799);

  while (1)
  {

      HAL_UART_Receive_IT(&huart2,rx_data,3);


/*
 * 	ativando o BYPASS a partir do momento que o angulo == angulo_min(30).
 * 	tambem estou desligando o output compare para retirar o circuito de potencia quando o bypass
 * 	fica ativo, seria inutil nao desligar o OC quando o bypass esta ativo. Para verificar se isso
 * 	realmente acontecia eu liguei o motor e esperei o bypass ficar ativo, em seguida retirei o pino
 * 	de bypass do circuito e o motor desligou automaticamente, significando que nao havia mais nenhum
 * 	acionamento nele sem que fosse o de bypass.
 */
	   if (angulo == ANGULO_MIN && bypass_ativo == 0) {
	       HAL_GPIO_WritePin(GPIOC, Bypass_Pin, GPIO_PIN_SET);
	       HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_2);
	       bypass_ativo = 1;
	       }
/*
 * verificando se o pino de emergencia esta ou nao ativo para desligar automaticamente o bypass quando
 * a emergencia ativar, seria impossivel utilizar o circuito de emergencia com o bypass ativo. Abaixo
 * tambem ligamos o Output Compare novamente para que o circuito possa funcionar normalmente para fazer
 * a etapa da rampa de descida.
 */
	  if ((HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin) == GPIO_PIN_SET && bypass_ativo) || angulo > ANGULO_MIN) {
	       HAL_GPIO_WritePin(GPIOC, Bypass_Pin, GPIO_PIN_RESET);
	       bypass_ativo = 0;
	       }

	  if(rx_data[0] == '2' && angulo == ANGULO_MAX){
	       HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_2);
	       memset(rx_data,0,3);//zerando os valores do vetor
	       angulo = ANGULO_MAX;
	  }

/*
 * 	Rampa de subida do motor travada em no maximo 150% de corrente,
 * ou seja, o motor nao irá passar dessa corrente enquanto estiver na
 * fase de subida. Travamos o angulo e a contagem do OC, fazendo com
 * que o PPM fique travado no mesmo valor até que o valor de corrente
 * desça para menos que 150%, infelizmente dessa maneira perdemos um
 * pouco da precisao da escolha do tempo da rampa de subida, mas
 * nao da pra ter tudo né?
 *
 */


	  if(rx_data[0] == '1' && voltage>3.1){
	      angulo=angulo;
	      HAL_TIM_Base_Stop(&htim3);
	  }else if(rx_data[0] == '1' && voltage<3.1){
	       HAL_TIM_Base_Start(&htim3);

	  }


/*
 * verificando o valor que recebo pela serial para ligar/desligar o motor.
 */
	  if(rx_data[0] == '2'){
	       estado_motor=0;
	      }
	  else if(rx_data[0] == '1'){
		  estado_motor=1;
		  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);

	  }

/*
* Codigo de uso do ADC e abaixo codigo para enviar pela serial para o CubeMonitor.
*/
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 300);
	      testeADC = HAL_ADC_GetValue(&hadc1);

	      voltage = (float)testeADC *(3.3/4095);

	      if (voltage > 3.29 && angulo == ANGULO_MIN) {
	    	  HAL_GPIO_WritePin(Emergency_GPIO_Port, Emergency_Pin, GPIO_PIN_SET);
                  HAL_UART_Transmit(&huart2, tx_data, sizeof(tx_data), 10);
	      }
/*
 * verifica se o botao Reset_emergency foi acionado na IHM, tambem define valor de angulo_max
 * para que quando o motor saia do emergencia nao saia acioanando direto, isso garante que o motor
 * ficara desligado depois do emergencia ser acionado
 */
	      if (rx_data[0] == '3'){
	          HAL_GPIO_WritePin(Emergency_GPIO_Port, Emergency_Pin, GPIO_PIN_RESET);
	          angulo=ANGULO_MAX;
		      HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_2);
	      }




	  HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }//while
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
/*
 * 				*SERIAL*
 *
 * CallBack do recebimento da UART, "rx_data" recebe pela serial do
 * STMCubeMonitor para realizarmos o tratamento das informacoes rebecidas.
 *
 *  Como temos alguns botoes na IHM, foi configurado para que cada botao
 *  retorne um numero caso seja precionado, a seguir a configuracao
 *  dos botoes:
 *
 *  Foi necessário efetuar o tratamento do valor desses botoes por JavaScript
 *  na IHM, pois quando enviamos pela serial o valor de subida ou descida esse
 *  valor chega no código com uma letra antes do seu número: (D) Descida
 *  (S) Subida e (A) Angulo e com mais 2 numeros logo atras, preenchendo o nosso
 *  vetor de 3 posicoes (rx_data). É necessário colocar o numero dos botoes
 *  enviado pela IHM para preencherem um vetor de 3 posicoes também, para isso
 *  coloquei eles como 100, 200 e 300, como poderemos ver a seguir:
 *
 *  Recebimento na Serial:
 *
 *      B1 -> IHM -> ==  200 ->Desligar o motor
 * 	B2 -> IHM -> ==  100 ->Ligar o motor
 * 	B3 -> IHM -> ==  300 -> Reset_Emergency
 *
 *	SLIDER1 -> IHM == Tempo_Descida
 *	SLIDER2 -> IHM == Tempo_Subida
 *	SLIDER3 -> IHM == Angulo_Subida
 *
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(angulo == ANGULO_MAX){
        if(rx_data[0] == 'S'){
        			tempoSubida = 0;	// reset value
        			tempoSubida += (rx_data[1] - '0') * 10; // first digit
        			tempoSubida += rx_data[2] - '0'; // second digit
        		}
        else if(rx_data[0] == 'D'){
        			tempoDescida = 0;	// reset value
        			tempoDescida += (rx_data[1] - '0') * 10; // first digit
        			tempoDescida += rx_data[2] - '0'; // second digit
        		}
        else if(rx_data[0]=='A'){
        	ANGULO_MAX = 0;	// reset value
        	ANGULO_MAX += (rx_data[1] - '0') * 10; // first digit
        	ANGULO_MAX += rx_data[2] - '0'; // second digit
        	ANGULO_MAX = ((ANGULO_MAX*-1) + 180);
        	angulo = ANGULO_MAX;
	    }

	}//if1

}//void

//CALLBACK INPUT CAPTURE

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){

/*
 * quando recebe 1, faz o calculo para trocar o valor do ARR do Output Compare com base no valor
 * do tempo de subida, quando recebe 2 faz a mesma coisa para valor de tempo de descida.
 */

	  if(rx_data[0]=='1'){
		  AutoReload=((84000000*tempoSubida/(ANGULO_MAX-ANGULO_MIN))/8399);
		  __HAL_TIM_SET_AUTORELOAD(&htim3, AutoReload);
	  }else if(rx_data[0] == '2'){
		  AutoReload=((84000000*tempoDescida/(ANGULO_MAX-ANGULO_MIN))/8399);
		  __HAL_TIM_SET_AUTORELOAD(&htim3, AutoReload);
	  }

	if(borda==0){
	 __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, ((angulo*10)+25));
	 borda=1;
	}
		  CCR=htim2.Instance->CCR2;
		  ARR=htim2.Instance->CNT;

}//void

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
/*
 * tim3 conta o tempo do decremento do angulo
 */
	if(htim->Instance == TIM3 && rx_data[0]=='1'){
		angulo--;
		if(angulo<ANGULO_MIN){
		angulo=ANGULO_MIN;
		}

	}else if (htim->Instance == TIM3 && rx_data[0]=='2'){
		angulo++;

	if(angulo>ANGULO_MAX){
		angulo=ANGULO_MAX;
	}
	}

	if(htim->Instance == TIM2){
		borda=0;
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,angulo*10);
		htim2.Instance->CCMR1 |= TIM_CCMR1_OC2CE;
	    }
}//void


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
