/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Drivers/BSP/STM32F4-Discovery/stm32f4_discovery_audio.h"
#ifndef __USBH_MIDI_CORE_H
#include "usbh_MIDI.h"
#endif
#include "MIDI_application.h"
#include "mtof.h"
#include "pitchbend1024.h"
#include "complexfreqangle.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct synthVoice {
    int active;
    int note;
    int velocity;
    float volume;
    float frequency;
    float phase1;
    float phase2;
} synthVoice;

typedef struct signalInterp {
    float value;
    float duration;
    float t;
    float delta;
    int resting;
} signalInterp;

typedef struct resonator {
    float out1;
    float out2;
    float a;
    float b;
    float c;
    float f;
    float bw;
} resonator;

typedef struct randomStruct {
    uint32_t next;
} randomStruct;

typedef enum {
    INACTIVE = 0, ATTACK, DECAY, SUSTAIN, RELEASE
} voiceState;

typedef enum {
    POLYVOICE = 0, MONOVOICE
} voicingMode;

typedef enum {
    AV=0, AVS, AF, AH, A0, A1, A2, A3, A4, A5, A6, AB, F0, F1, F2, F3, F4, F5, F6, FNP, FNZ, B1, B2, B3, B4, B5, B6, BNP, BNZ, NUM_FORMANT_PARAM
} formantParam;

typedef enum {
    RGP=0, RGZ, RGS, LPF, RNP, RNZ, RC1, RC2, RC3, RC4, RC5, RP2, RP3, RP4, RP5, RP6, NUM_RESONATOR
} resonatorNames;

typedef struct delayedInterp {
    float start;
    float value;
    float duration;
    formantParam param;
} delayedInterp;

typedef enum {
    CONS0 = 36,
    CONS1,
    CONS2,
    CONS3,
    VWL0,
    VWL1,
    VWL2,
    ENQUEUE_CV,
} midiCVCode;

typedef enum {
    CODE_K = 0x1,
    CODE_S = 0x2,
    CODE_R = 0x3,
    CODE_T = 0x4,
    CODE_W = 0x5,
    CODE_M = 0x6,
    CODE_Y = 0x7,
    CODE_N = 0x8,
    CODE_G = 0x9,
    CODE_Z = 0xA,
    CODE_B = 0xB,
    CODE_D = 0xC,
    CODE_P = 0xD,
    CODE_H = 0xE,
    CODE_SPACE = 0x0
} consonantCode;

typedef enum {
    CODE_U = 0x1,
    CODE_I = 0x2,
    CODE_O = 0x3,
    CODE_A = 0x4,
    CODE_E = 0x6,
    CODE_AI = 0x5,
    CODE_EI = 0x7,
    CODE_REP = 0x0
} vowelCode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

#define SAMPLE_RATE 48000.0f
#define DELTA_TIME 1.0f / SAMPLE_RATE
#define MIDI_CTRL_NORM 1.0f / 127.0f
#define PI 3.14159265358979323846f
#define PI_DT PI * DELTA_TIME
#define PI_2 PI / 2.0f

int16_t codecBuffer[64];    // 32 samples X 2 channels
extern I2S_HandleTypeDef hAudioOutI2s;
extern I2S_HandleTypeDef hAudioInI2s;
// Sound globals
USBH_HandleTypeDef hUSBHost; /* USB Host handle */
MIDI_ApplicationTypeDef Appli_state = MIDI_APPLICATION_IDLE;
void audioBlock(float *input, float *output, int32_t samples);
float mtoinc[128];
int pitchbend;
int buttonState;
uint8_t runningStatus;
float inBuffer[1024], outBuffer[1024]; // interleaved - LRLRLRLRLRLRLRLRLRLRLR - inBuffer[frame <<
synthVoice voice[16];
int keyboard[128];
float wavetable[4096];
// synth globals
float masterVolume, masterVolumeKnob;
float waveshapper, waveshapperKnob;
// complex LFO oscillator
float lfoCos, lfoSin, angleReal, angleImag, lfoDepth;
// ADSR
float attack, decay, sustain, release;

// resonator
resonator pResonator;
float pResonBW, pResonBWKnob, pResonF, pResonFKnob, pResonWet, pResonWetKnob;
resonator pAResonator;
float pAResonBW, pAResonBWKnob, pAResonF, pAResonFKnob, pAResonWet, pAResonWetKnob;

// table lookup oscillator
int32_t tablesize, tablemask;
// sinewave creation
float sinTaylor(float phase);
// wave table creation
void tableInit(int32_t type);
// 2 point interpolated lookup
float sinTable(float phase);


voicingMode voiceMode = POLYVOICE;

int voiceCount = 0;
int voiceSum = 0;
float runningVoicePhase = 0.0f;

float flutter = 0.1f;
float flutterf1 = 12.7f * DELTA_TIME;
float flutterf2 = 7.1f * DELTA_TIME;
float flutterf3 = 4.7f * DELTA_TIME;
float flutterp1 = 0.0f;
float flutterp2 = 0.0f;
float flutterp3 = 0.0f;

signalInterp formantParams[NUM_FORMANT_PARAM];
resonator resonators[NUM_RESONATOR];

#define CV_QUEUE_CAP 64
char cvKeyboard = 0;
char cvQueue[CV_QUEUE_CAP];
int cvQueueFront = 0;
int cvQueueRear = -1;
int cvQueueSize = 0;
vowelCode lastVowel = CODE_A;

#define DEL_QUEUE_CAP 32
delayedInterp delQueue[DEL_QUEUE_CAP];
int delQueueFront = 0;
int delQueueRear = -1;
int delQueueSize = 0;
float delTime = 0.0f;

randomStruct randStruct = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void USBH_UserProcess_callback(USBH_HandleTypeDef *pHost, uint8_t vId);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    int i, waveType, deglitch;

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    pitchbend = 8192;
    for (i = 0; i < 128; i++) {
        mtoinc[i] = mtof[i] * 0.0000208333f; // 1/48000
        keyboard[i] = -1;
    }
    for (i = 0; i < 16; i++) {
        voice[i].active = INACTIVE;
        voice[i].phase1 = 0.0f;
        voice[i].phase2 = 0.0f;
        voice[i].volume = 0.0f;
    }
    for (i = 0; i < NUM_FORMANT_PARAM; i++) {
        formantParams[i].value = 0;
        formantParams[i].duration = 0.01f;
        formantParams[i].t = 0.0f;
        formantParams[i].delta = 0.0f;
        formantParams[i].resting = 1;
    }
    for (i = 0; i < NUM_RESONATOR; i++) {
        resonators[i].out1 = 0.0f;
        resonators[i].out2 = 0.0f;
        resonators[i].f = 0.0f;
        resonators[i].bw = 0.0f;
        resonators[i].a = 0.0f;
        resonators[i].b = 0.0f;
        resonators[i].c = 0.0f;
    }

    masterVolume = masterVolumeKnob = 1.0f;
    waveshapper = waveshapperKnob = 0.9f;
    angleReal = realAngleMIDI[0];
    angleImag = imagAngleMIDI[0];
    lfoSin = 1.0f;
    lfoCos = 0.0f;
    lfoDepth = 0.0f;
    deglitch = 0;
    attack = 0.05f;
    release = 0.1f;
    decay = sustain = 1.0f;

    pResonator.out1 = pResonator.out2 = 0.0f;
    pResonBW = pResonBWKnob = 1000.0f;
    pResonF = pResonFKnob = 500.0f;
    pResonWet = pResonWetKnob = 0.0f;
    pAResonator.out1 = pAResonator.out2 = 0.0f;
    pAResonBW = pAResonBWKnob = 1000.0f;
    pAResonF = pAResonFKnob = 500.0f;
    pAResonWet = pAResonWetKnob = 0.0f;

    // push in an ' a' default
    cvQueueRear = (cvQueueRear == CV_QUEUE_CAP - 1) ? 0 : cvQueueRear + 1;
    cvQueue[cvQueueRear] = CODE_SPACE << 3 | CODE_A;
    cvQueueSize++;

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
//  MX_I2C1_Init();
//  MX_I2S3_Init();
//  MX_SPI1_Init();
//  MX_USB_HOST_Init();
    /* USER CODE BEGIN 2 */
    buttonState = 0;
    waveType = 0;
    tableInit(waveType);
    /*## Init Host Library ################################################*/
    USBH_Init(&hUSBHost, USBH_UserProcess_callback, 0);

    /*## Add Supported Class ##############################################*/
    USBH_RegisterClass(&hUSBHost, USBH_MIDI_CLASS);

    /*## Start Host Process ###############################################*/
    USBH_Start(&hUSBHost);

    BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_HEADPHONE, 90, 48000);
    BSP_AUDIO_OUT_Play((uint16_t*) codecBuffer, 128);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
//	  HAL_Delay(1);
//	  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
//	  HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
        /* USER CODE END WHILE */
//    MX_USB_HOST_Process();
        /* USER CODE BEGIN 3 */
        MIDI_Application();
        // check button B1
        deglitch++;
        if (deglitch > 3000) deglitch = 3000;
        if (buttonState != HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) && deglitch > 100) {
            buttonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
            if (buttonState == 1) {
                voiceMode = voiceMode == POLYVOICE ? MONOVOICE : POLYVOICE;
            }
            deglitch = 0;
        }
        //USBH_Delay(1);

        /* USBH_Background Process */
        USBH_Process(&hUSBHost);
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
    PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
    PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void) {

    /* USER CODE BEGIN I2S3_Init 0 */

    /* USER CODE END I2S3_Init 0 */

    /* USER CODE BEGIN I2S3_Init 1 */

    /* USER CODE END I2S3_Init 1 */
    hi2s3.Instance = SPI3;
    hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
    hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
    hi2s3.Init.CPOL = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2S3_Init 2 */

    /* USER CODE END I2S3_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : CS_I2C_SPI_Pin */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PDM_OUT_Pin */
    GPIO_InitStruct.Pin = PDM_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : BOOT1_Pin */
    GPIO_InitStruct.Pin = BOOT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : CLK_IN_Pin */
    GPIO_InitStruct.Pin = CLK_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
     Audio_RST_Pin */
    GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
    GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : MEMS_INT2_Pin */
    GPIO_InitStruct.Pin = MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float randf() {
    randStruct.next = (randStruct.next * 196314165) + 907633515;
    return (float)randStruct.next * 0.0000000004656612873077392578125f - 1.0f;
}

float expf_fast(float a) {
    union { float f; int x; } u;
    u.x = (int) (12102203 * a + 1064866805);
    return u.f;
}

void interpSet(signalInterp* interp, float value) {
    interp->value = value;
    interp->resting = 1;
}
void interp(signalInterp* interp, float value, float duration) {
    if (duration == 0.0f || (duration <= 0.0f && interp->duration == 0.0f)) {
        interpSet(interp, value);
        return;
    } else if (duration > 0.0f) {
        interp->duration = duration;
    }
    interp->delta = (value - interp->value) / interp->duration * DELTA_TIME;
    interp->t = 0.0f;
    interp->resting = 0;
}
void enqueueDelInterp(formantParam param, float delay, float value, float duration) {
    int curr, prior;
    delQueueRear = (delQueueRear + 1) % DEL_QUEUE_CAP;
    curr = delQueueRear;
    prior = curr == 0 ? DEL_QUEUE_CAP - 1 : curr - 1;
    while (curr != delQueueFront && delay < delQueue[prior].start) {
        delQueue[curr] = delQueue[prior];
        curr = prior;
        prior = curr == 0 ? DEL_QUEUE_CAP - 1 : curr - 1;
    }
    delayedInterp di = {delay, value, duration, param};
    delQueue[curr] = di;
    delQueueSize++;
}
delayedInterp dequeueDelInterp() {
    delayedInterp di = delQueue[delQueueFront];
    delQueueFront = (delQueueFront == DEL_QUEUE_CAP - 1) ? 0 : delQueueFront + 1;
    delQueueSize--;
    return di;
}
void emptyDelInterp() {
    delQueueFront = (delQueueRear == DEL_QUEUE_CAP - 1) ? 0 : delQueueRear + 1;
    delQueueSize = 0;
    delTime = 0.0f;
}
void setFormantTransitions(float duration) {
    formantParams[F1].duration = formantParams[F2].duration = formantParams[F3].duration = formantParams[F4].duration = formantParams[F5].duration = formantParams[FNZ].duration =
            formantParams[B1].duration = formantParams[B2].duration = formantParams[B3].duration = formantParams[B4].duration = formantParams[B5].duration = duration;
}
int voiceResting(float thresh) {
    return (formantParams[AV].value < 0.0f) ? (formantParams[AV].value > -thresh) : (formantParams[AV].value < thresh);
}
void releaseNote(midi_package_t pack) {
    int i;
    float endDelay;
    if (voiceMode == POLYVOICE) {
        for (i = 0; i < 16; i++) {
            if (voice[i].note == pack.evnt1) {
                voice[i].active = RELEASE;
                keyboard[voice[i].note] = -1;
            }
        }
    } else if (voiceMode == MONOVOICE) {
        if (pack.evnt1 >= 44) {
            if (voiceCount) {
                voiceSum -= pack.evnt1;
                voiceCount--;
                if (!voiceCount) {
                    endDelay = delQueue[delQueueRear].start - delTime;
                    endDelay = endDelay < 0.0f ? 0.0f : endDelay;
                    interp(&formantParams[F0], formantParams[F0].value, -1.0f);
                    enqueueDelInterp(AV, endDelay + 0.1f, 0.0f, 0.2f);
                    enqueueDelInterp(AVS, endDelay + 0.1f, 0.0f, 0.2f);

                    // Aspiration
                    enqueueDelInterp(AH, endDelay + 0.1f, 0.01f, 0.02f);
                    enqueueDelInterp(AH, endDelay + 0.1f, 0.0f, 0.2f);
                } else {
                    interp(&formantParams[F0], (float)voiceSum / voiceCount, -1.0f);
                }
            }
        }
    }
}
void ProcessMIDI(midi_package_t pack) {
    int i;
    uint8_t status;
    char currCV;
    float vowelDelay, vowelTransition;
    // Status messages that start with F, for all MIDI channels
    // None of these are implemented - though we will flash an LED
    // for the MIDI clock
    status = pack.evnt0 & 0xF0;
    if (status == 0xF0) {
        switch (pack.evnt0) {
        case 0xF0:	// Start of System Exclusive
        case 0xF1:	// MIDI Time Code Quarter Fram
        case 0xF2:	// Song Position Pointer
        case 0xF3:	// Song Select
        case 0xF4:	// Undefined
        case 0xF5:	// Undefined
        case 0xF6:	// Tune Request
        case 0xF7:	// End of System Exclusive
            status = runningStatus = 0x00;
            break;
        case 0xF8:	// Timing Clock (24 times a quarter note)
            HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin); // RED LED
            break;
        case 0xF9:	// Undefined
        case 0xFA:	// Start Sequence
        case 0xFB:	// Continue Sequence
        case 0xFC:	// Pause Sequence
        case 0xFD:	// Undefined
        case 0xFE:	// Active Sensing
        case 0xFF:	// Reset all synthesizers to power up
            break;

        }
    }

// MIDI running status (same status as last message) doesn't seem to work over this USB driver
// code commented out.

//	else if((pack.evnt0 & 0x80) == 0x00)
//		status = runningStatus;
    else
        runningStatus = status = pack.evnt0 & 0xF0;

    switch (status) {
    case 0x80:	// Note Off
        // turn off all voices that match the note off note
        releaseNote(pack);
        break;
    case 0x90:	// Note On
        if (pack.evnt2 == 0) { // velocity 0 means note off
            // turn off all voices that match the note off note
            releaseNote(pack);
        } else {
            // if this key is already on, end the associated note and turn it off
            if (voiceMode == POLYVOICE) {
                if (keyboard[pack.evnt1] != -1) {
                    voice[keyboard[pack.evnt1]].active = RELEASE;
                    keyboard[pack.evnt1] = -1;
                }
                // find an inactive voice and assign this key to it
                for (i = 0; i < 16; i++) {
                    if (voice[i].active == INACTIVE) {
                        voice[i].active = ATTACK;
                        voice[i].note = pack.evnt1;
                        voice[i].velocity = pack.evnt2;
                        keyboard[pack.evnt1] = i;
                        break;
                    }
                }
            } else if (voiceMode == MONOVOICE) {
                if (pack.evnt1 > ENQUEUE_CV) {
                    voiceSum += pack.evnt1;
                    voiceCount++;
                    if (voiceCount == 1 && voiceResting(0.05)) {
                        interpSet(&formantParams[F0], pack.evnt1);
                        interpSet(&formantParams[A0], pack.evnt2);
                    } else {
                        interp(&formantParams[F0], (float)voiceSum / voiceCount, -1.0f);
                        interp(&formantParams[A0], pack.evnt2, -1.0f);
                    }

                    currCV = cvQueue[cvQueueFront];
                    if (cvQueueSize > 0) {
                        cvQueueFront = (cvQueueFront + 1) % CV_QUEUE_CAP;
                        cvQueueSize--;
                    } else {
                        currCV = cvQueue[(cvQueueFront ? cvQueueFront : CV_QUEUE_CAP) - 1];
                    }
                    vowelDelay = 0.0f;
                    vowelTransition = -1.0f;
                    emptyDelInterp();
                    switch ((currCV & 0x78) >> 3) {
                    case CODE_R:
                        setFormantTransitions((voiceResting(0.01)) ? 0.0f : 0.1f);
                        enqueueDelInterp(F1, 0.0f, 310.0f, -1.0f);
                        enqueueDelInterp(F2, 0.0f, 1060.0f, -1.0f);
                        enqueueDelInterp(F3, 0.0f, 2500.0f, -1.0f);
                        enqueueDelInterp(B1, 0.0f, 70.0f, -1.0f);
                        enqueueDelInterp(B2, 0.0f, 100.0f, -1.0f);
                        enqueueDelInterp(B3, 0.0f, 200.0f, -1.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.0f, 0.001f, 0.025f);
                        enqueueDelInterp(AH, 0.025f, 0.0f, 0.05f);
                        enqueueDelInterp(AV, 0.0f, 0.2f, 0.1f);
                        enqueueDelInterp(AVS, 0.0f, 0.2f, 0.1f);

                        // Voicing
                        vowelDelay = 0.1f;
                        vowelTransition = 0.1f;
                        enqueueDelInterp(AV, 0.1f, 1.0f, 0.1f);
                        enqueueDelInterp(AVS, 0.1f, 1.0f, 0.1f);
                        break;

                    case CODE_W:
                        setFormantTransitions((voiceResting(0.1)) ? 0.0f : 0.1f);
                        enqueueDelInterp(F1, 0.0f, 290.0f, -1.0f);
                        enqueueDelInterp(F2, 0.0f, 610.0f, -1.0f);
                        enqueueDelInterp(F3, 0.0f, 2150.0f, -1.0f);
                        enqueueDelInterp(B1, 0.0f, 50.0f, -1.0f);
                        enqueueDelInterp(B2, 0.0f, 80.0f, -1.0f);
                        enqueueDelInterp(B3, 0.0f, 60.0f, -1.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.0f, 0.001f, 0.025f);
                        enqueueDelInterp(AH, 0.025f, 0.0f, 0.05f);
                        enqueueDelInterp(AV, 0.0f, 0.2f, 0.1f);
                        enqueueDelInterp(AVS, 0.0f, 0.2f, 0.1f);

                        // Voicing
                        vowelDelay = 0.1f;
                        vowelTransition = 0.1f;
                        enqueueDelInterp(AV, 0.1, 1.0, 0.1);
                        enqueueDelInterp(AVS, 0.1, 1.0, 0.1);
                        break;

                    case CODE_Y:
                        setFormantTransitions((voiceResting(0.1)) ? 0.0f : 0.1f);
                        enqueueDelInterp(F1, 0.0f, 260.0f, -1.0f);
                        enqueueDelInterp(F2, 0.0f, 2070.0f, -1.0f);
                        enqueueDelInterp(F3, 0.0f, 3020.0f, -1.0f);
                        enqueueDelInterp(B1, 0.0f, 40.0f, -1.0f);
                        enqueueDelInterp(B2, 0.0f, 250.0f, -1.0f);
                        enqueueDelInterp(B3, 0.0f, 500.0f, -1.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.0f, 0.001f, 0.025f);
                        enqueueDelInterp(AH, 0.025f, 0.0f, 0.05f);
                        enqueueDelInterp(AV, 0.0f, 0.2f, 0.1f);
                        enqueueDelInterp(AVS, 0.0f, 0.2f, 0.1f);

                        // Voicing
                        vowelDelay = 0.1f;
                        vowelTransition = 0.1f;
                        enqueueDelInterp(AV, 0.1, 1.0, 0.1);
                        enqueueDelInterp(AVS, 0.1, 1.0, 0.1);
                        break;

                    case CODE_H:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.015);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.015);

                        setFormantTransitions(0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.025f, 0.1f, 0.025f);
                        enqueueDelInterp(AH, 0.05f, 0.0f, 0.25f);

                        // Voicing
                        vowelDelay = 0.025f;
                        enqueueDelInterp(AV, 0.125, 1.0, 0.1);
                        enqueueDelInterp(AVS, 0.125, 1.0, 0.1);
                        break;

                    case CODE_K:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.015);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.015);

                        setFormantTransitions(0.0f);
                        enqueueDelInterp(F1, 0.02f, 300.0f, -1.0f);
                        enqueueDelInterp(F2, 0.02f, 1990.0f, -1.0f);
                        enqueueDelInterp(F3, 0.02f, 2850.0f, -1.0f);
                        enqueueDelInterp(B1, 0.02f, 250.0f, -1.0f);
                        enqueueDelInterp(B2, 0.02f, 160.0f, -1.0f);
                        enqueueDelInterp(B3, 0.02f, 330.0f, -1.0f);

                        // Burst
                        enqueueDelInterp(A3, 0.0f, 0.75f, 0.0f);
                        enqueueDelInterp(A4, 0.0f, 0.125f, 0.0f);
                        enqueueDelInterp(A5, 0.0f, 0.1f, 0.0f);
                        enqueueDelInterp(AB, 0.0f, 0.1f, 0.0f);
                        enqueueDelInterp(AF, 0.05f, 0.25f, 0.005f);
                        enqueueDelInterp(AF, 0.055f, 0.0f, 0.015f);
                        enqueueDelInterp(A3, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(AB, 0.1f, 0.0f, 0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.05f, 0.05f, 0.025f);
                        enqueueDelInterp(AH, 0.075f, 0.0f, 0.3f);

                        // Voicing
                        vowelDelay = 0.0201f;
                        vowelTransition = 0.2f;
                        enqueueDelInterp(AV, 0.0201, 1.0, 0.05f);
                        enqueueDelInterp(AVS, 0.0201, 1.0, 0.05f);
                        break;

                    case CODE_T:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.015);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.015);

                        setFormantTransitions(0.0f);
                        enqueueDelInterp(F1, 0.02f, 400.0f, -1.0f);
                        enqueueDelInterp(F2, 0.02f, 1600.0f, -1.0f);
                        enqueueDelInterp(F3, 0.02f, 2600.0f, -1.0f);
                        enqueueDelInterp(B1, 0.02f, 300.0f, -1.0f);
                        enqueueDelInterp(B2, 0.02f, 120.0f, -1.0f);
                        enqueueDelInterp(B3, 0.02f, 250.0f, -1.0f);

                        // Burst
                        enqueueDelInterp(A3, 0.0f, 0.01f, 0.0f);
                        enqueueDelInterp(A4, 0.0f, 0.2f, 0.0f);
                        enqueueDelInterp(A5, 0.0f, 1.0f, 0.0f);
                        enqueueDelInterp(AB, 0.0f, 1.0f, 0.0f);
                        enqueueDelInterp(AF, 0.05f, 0.25f, 0.005f);
                        enqueueDelInterp(AF, 0.055f, 0.0f, 0.015f);
                        enqueueDelInterp(A3, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(AB, 0.1f, 0.0f, 0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.05f, 0.01f, 0.025f);
                        enqueueDelInterp(AH, 0.075f, 0.0f, 0.3f);

                        // Voicing
                        vowelDelay = 0.0201f;
                        vowelTransition = 0.2f;
                        enqueueDelInterp(AV, 0.0201, 1.0, 0.05f);
                        enqueueDelInterp(AVS, 0.0201, 1.0, 0.05f);
                        break;

                    case CODE_P:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.015);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.015);

                        setFormantTransitions(0.0f);
                        enqueueDelInterp(F1, 0.02f, 400.0f, -1.0f);
                        enqueueDelInterp(F2, 0.02f, 1100.0f, -1.0f);
                        enqueueDelInterp(F3, 0.02f, 2150.0f, -1.0f);
                        enqueueDelInterp(B1, 0.02f, 60.0f, -1.0f);
                        enqueueDelInterp(B2, 0.02f, 110.0f, -1.0f);
                        enqueueDelInterp(B3, 0.02f, 130.0f, -1.0f);

                        // Burst
                        enqueueDelInterp(A3, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(AB, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(AF, 0.05f, 0.25f, 0.005f);
                        enqueueDelInterp(AF, 0.055f, 0.0f, 0.015f);
                        enqueueDelInterp(A3, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(AB, 0.1f, 0.0f, 0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.05f, 0.02f, 0.025f);
                        enqueueDelInterp(AH, 0.075f, 0.0f, 0.3f);

                        // Voicing
                        vowelDelay = 0.0201f;
                        vowelTransition = 0.2f;
                        enqueueDelInterp(AV, 0.0201, 1.0, 0.05f);
                        enqueueDelInterp(AVS, 0.0201, 1.0, 0.05f);
                        break;

                    case CODE_G:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.015);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.015);

                        setFormantTransitions(0.0f);
                        enqueueDelInterp(F1, 0.02f, 200.0f, -1.0f);
                        enqueueDelInterp(F2, 0.02f, 1990.0f, -1.0f);
                        enqueueDelInterp(F3, 0.02f, 2850.0f, -1.0f);
                        enqueueDelInterp(B1, 0.02f, 60.0f, -1.0f);
                        enqueueDelInterp(B2, 0.02f, 150.0f, -1.0f);
                        enqueueDelInterp(B3, 0.02f, 280.0f, -1.0f);

                        // Burst
                        enqueueDelInterp(A3, 0.0f, 0.5f, 0.0f);
                        enqueueDelInterp(A4, 0.0f, 0.125f, 0.0f);
                        enqueueDelInterp(A5, 0.0f, 0.15f, 0.0f);
                        enqueueDelInterp(AB, 0.0f, 0.15f, 0.0f);
                        enqueueDelInterp(AF, 0.05f, 0.05f, 0.001f);
                        enqueueDelInterp(AF, 0.051f, 0.0f, 0.01f);
                        enqueueDelInterp(A3, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(AB, 0.1f, 0.0f, 0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.05f, 0.01f, 0.025f);
                        enqueueDelInterp(AH, 0.075f, 0.0f, 0.1f);

                        // Voicing
                        vowelDelay = 0.1f;
                        vowelTransition = 0.1f;
                        enqueueDelInterp(AV, 0.1, 1.0, 0.05f);
                        enqueueDelInterp(AVS, 0.1, 1.0, 0.05f);
                        break;

                    case CODE_D:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.015);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.015);

                        setFormantTransitions(0.0f);
                        enqueueDelInterp(F1, 0.02f, 200.0f, -1.0f);
                        enqueueDelInterp(F2, 0.02f, 1600.0f, -1.0f);
                        enqueueDelInterp(F3, 0.02f, 2600.0f, -1.0f);
                        enqueueDelInterp(B1, 0.02f, 60.0f, -1.0f);
                        enqueueDelInterp(B2, 0.02f, 100.0f, -1.0f);
                        enqueueDelInterp(B3, 0.02f, 170.0f, -1.0f);

                        // Burst
                        enqueueDelInterp(A3, 0.0f, 0.25f, 0.0f);
                        enqueueDelInterp(A4, 0.0f, 1.0f, 0.0f);
                        enqueueDelInterp(A5, 0.0f, 1.2f, 0.0f);
                        enqueueDelInterp(AB, 0.0f, 1.0f, 0.0f);
                        enqueueDelInterp(AF, 0.05f, 0.05f, 0.001f);
                        enqueueDelInterp(AF, 0.051f, 0.0f, 0.01f);
                        enqueueDelInterp(A3, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(AB, 0.1f, 0.0f, 0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.05f, 0.005f, 0.025f);
                        enqueueDelInterp(AH, 0.075f, 0.0f, 0.1f);

                        // Voicing
                        vowelDelay = 0.0201f;
                        vowelTransition = 0.2f;
                        enqueueDelInterp(AV, 0.0201, 1.0, 0.05f);
                        enqueueDelInterp(AVS, 0.0201, 1.0, 0.05f);
                        break;

                    case CODE_B:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.015);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.015);

                        setFormantTransitions(0.0f);
                        enqueueDelInterp(F1, 0.02f, 200.0f, -1.0f);
                        enqueueDelInterp(F2, 0.02f, 1100.0f, -1.0f);
                        enqueueDelInterp(F3, 0.02f, 2150.0f, -1.0f);
                        enqueueDelInterp(B1, 0.02f, 300.0f, -1.0f);
                        enqueueDelInterp(B2, 0.02f, 150.0f, -1.0f);
                        enqueueDelInterp(B3, 0.02f, 220.0f, -1.0f);

                        // Burst
                        enqueueDelInterp(A3, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(AB, 0.0f, 1.0f, 0.0f);
                        enqueueDelInterp(AF, 0.05f, 0.05f, 0.001f);
                        enqueueDelInterp(AF, 0.051f, 0.0f, 0.01f);
                        enqueueDelInterp(A3, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.1f, 0.0f, 0.0f);
                        enqueueDelInterp(AB, 0.1f, 0.0f, 0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.05f, 0.005f, 0.025f);
                        enqueueDelInterp(AH, 0.075f, 0.0f, 0.1f);

                        // Voicing
                        vowelDelay = 0.0201f;
                        vowelTransition = 0.2f;
                        enqueueDelInterp(AV, 0.0201, 1.0, 0.05f);
                        enqueueDelInterp(AVS, 0.0201, 1.0, 0.05f);
                        break;

                    case CODE_S:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.025);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.025);

                        setFormantTransitions(0.0f);
                        enqueueDelInterp(F1, 0.03f, 320.0f, -1.0f);
                        enqueueDelInterp(F2, 0.03f, 1390.0f, -1.0f);
                        enqueueDelInterp(F3, 0.03f, 2530.0f, -1.0f);
                        enqueueDelInterp(B1, 0.03f, 200.0f, -1.0f);
                        enqueueDelInterp(B2, 0.03f, 80.0f, -1.0f);
                        enqueueDelInterp(B3, 0.03f, 200.0f, -1.0f);

                        // Burst
                        enqueueDelInterp(A3, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.0f, 0.4f, 0.0f);
                        enqueueDelInterp(AB, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(AF, 0.05f, 0.2f, 0.015f);
                        enqueueDelInterp(AF, 0.10f, 0.0f, 0.2f);
                        enqueueDelInterp(A5, 0.3f, 0.0f, 0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.1f, 0.02f, 0.1f);
                        enqueueDelInterp(AH, 0.15f, 0.0f, 0.15f);

                        // Voicing
                        vowelDelay = 0.1f;
                        vowelTransition = 0.05f;
                        enqueueDelInterp(AV, 0.1, 1.0, 0.1);
                        enqueueDelInterp(AVS, 0.1, 1.0, 0.1);
                        break;

                    case CODE_Z:
                        // Stop
                        enqueueDelInterp(AV, 0.0, 0.0, 0.025);
                        enqueueDelInterp(AVS, 0.0, 0.0, 0.025);

                        setFormantTransitions(0.0f);
                        enqueueDelInterp(F1, 0.03f, 240.0f, -1.0f);
                        enqueueDelInterp(F2, 0.03f, 1390.0f, -1.0f);
                        enqueueDelInterp(F3, 0.03f, 2530.0f, -1.0f);
                        enqueueDelInterp(B1, 0.03f, 70.0f, -1.0f);
                        enqueueDelInterp(B2, 0.03f, 60.0f, -1.0f);
                        enqueueDelInterp(B3, 0.03f, 180.0f, -1.0f);

                        // Burst
                        enqueueDelInterp(A3, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(A4, 0.0f, 0.0f, 0.0f);
                        enqueueDelInterp(A5, 0.0f, 0.4f, 0.0f);
                        enqueueDelInterp(AF, 0.05f, 0.1f, 0.015f);
                        enqueueDelInterp(AF, 0.10f, 0.0f, 0.2f);
                        enqueueDelInterp(AF, 0.2f, 0.0f, 0.1f);
                        enqueueDelInterp(A5, 0.3f, 0.0f, 0.0f);

                        // Aspiration
                        enqueueDelInterp(AH, 0.1f, 0.02f, 0.1f);
                        enqueueDelInterp(AH, 0.15f, 0.0f, 0.15f);

                        // Voicing
                        vowelDelay = 0.1f;
                        vowelTransition = 0.05f;
                        enqueueDelInterp(AV, 0.2, 1.0, 0.05);
                        enqueueDelInterp(AVS, 0.2, 1.0, 0.05);
                        break;

                    case CODE_M:
                        setFormantTransitions((voiceResting(0.01)) ? 0.0f : 0.05f);

                        enqueueDelInterp(FNZ, 0.0f, 450.0f, -1.0f);
                        enqueueDelInterp(F1, 0.0f, 480.0f, -1.0f);
                        enqueueDelInterp(F2, 0.0f, 1270.0f, -1.0f);
                        enqueueDelInterp(F3, 0.0f, 2130.0f, -1.0f);
                        enqueueDelInterp(B1, 0.0f, 40.0f, -1.0f);
                        enqueueDelInterp(B2, 0.0f, 200.0f, -1.0f);
                        enqueueDelInterp(B3, 0.0f, 200.0f, -1.0f);

                        // Nasal hum
                        enqueueDelInterp(AV, 0.0f, 0.2f, 0.1f);
                        enqueueDelInterp(AVS, 0.0f, 0.2f, 0.1f);

                        vowelDelay = 0.1f;
                        vowelTransition = 0.1f;
                        enqueueDelInterp(AV, 0.15f, 1.0f, 0.1f);
                        enqueueDelInterp(AVS, 0.15f, 1.0f, 0.1f);
                        break;

                    case CODE_N:
                        setFormantTransitions((voiceResting(0.01)) ? 0.0f : 0.05f);

                        enqueueDelInterp(FNZ, 0.0f, 450.0f, -1.0f);
                        enqueueDelInterp(F1, 0.0f, 480.0f, -1.0f);
                        enqueueDelInterp(F2, 0.0f, 1340.0f, -1.0f);
                        enqueueDelInterp(F3, 0.0f, 2470.0f, -1.0f);
                        enqueueDelInterp(B1, 0.0f, 40.0f, -1.0f);
                        enqueueDelInterp(B2, 0.0f, 300.0f, -1.0f);
                        enqueueDelInterp(B3, 0.0f, 300.0f, -1.0f);

                        // Nasal hum
                        enqueueDelInterp(AV, 0.0f, 0.2f, 0.1f);
                        enqueueDelInterp(AVS, 0.0f, 0.2f, 0.1f);

                        vowelDelay = 0.1;
                        vowelTransition = 0.1f;
                        enqueueDelInterp(AV, 0.15f, 1.0f, 0.1f);
                        enqueueDelInterp(AVS, 0.15f, 1.0f, 0.1f);
                        break;

                    case CODE_SPACE:
                        enqueueDelInterp(AV, 0.0f, 1.0f, 0.2f);
                        enqueueDelInterp(AVS, 0.0f, 1.0f, 0.2f);
                        setFormantTransitions((voiceResting(0.01)) ? 0.0f : 0.1f);
                        break;

                    default:
                        break;
                    }

                    // Process vowels
                    lastVowel = ((currCV & 0x7) == CODE_REP) ? lastVowel : (currCV & 0x7);
                    switch (lastVowel) {
                    case CODE_A:
                        // Front vowel
                        enqueueDelInterp(A2, 0.0f, 1.0f, 0.0f);
                        enqueueDelInterp(A2, 0.1f, 0.0f, 0.0f);

                        enqueueDelInterp(FNZ, vowelDelay, 270.0f, vowelTransition);
                        enqueueDelInterp(F1, vowelDelay, 850.0f, vowelTransition);
                        enqueueDelInterp(F2, vowelDelay, 1220.0f, vowelTransition);
                        enqueueDelInterp(F3, vowelDelay, 2810.0f, vowelTransition);
                        enqueueDelInterp(B1, vowelDelay, 80.0f, vowelTransition);
                        enqueueDelInterp(B2, vowelDelay, 70.0f, vowelTransition);
                        enqueueDelInterp(B3, vowelDelay, 70.0f, vowelTransition);
                        break;

                    case CODE_I:
                        // Front vowel
                        enqueueDelInterp(A2, 0.0f, 1.0f, 0.0f);
                        enqueueDelInterp(A2, 0.1f, 0.0f, 0.0f);

                        enqueueDelInterp(FNZ, vowelDelay, 270.0f, vowelTransition);
                        enqueueDelInterp(F1, vowelDelay, 300.0f, vowelTransition);
                        enqueueDelInterp(F2, vowelDelay, 2800.0f, vowelTransition);
                        enqueueDelInterp(F3, vowelDelay, 3300.0f, vowelTransition);
                        enqueueDelInterp(B1, vowelDelay, 45.0f, vowelTransition);
                        enqueueDelInterp(B2, vowelDelay, 200.0f, vowelTransition);
                        enqueueDelInterp(B3, vowelDelay, 400.0f, vowelTransition);
                        break;

                    case CODE_U:
                        enqueueDelInterp(FNZ, vowelDelay, 270.0f, vowelTransition);
                        enqueueDelInterp(F1, vowelDelay, 470.0f, vowelTransition);
                        enqueueDelInterp(F2, vowelDelay, 1160.0f, vowelTransition);
                        enqueueDelInterp(F3, vowelDelay, 2680.0f, vowelTransition);
                        enqueueDelInterp(B1, vowelDelay, 80.0f, vowelTransition);
                        enqueueDelInterp(B2, vowelDelay, 100.0f, vowelTransition);
                        enqueueDelInterp(B3, vowelDelay, 80.0f, vowelTransition);
                        break;

                    case CODE_E:
                        // Front vowel
                        enqueueDelInterp(A2, 0.0f, 0.5f, 0.0f);
                        enqueueDelInterp(A2, 0.1f, 0.0f, 0.0f);

                        enqueueDelInterp(FNZ, vowelDelay, 270.0f, vowelTransition);
                        enqueueDelInterp(F1, vowelDelay, 600.0f, vowelTransition);
                        enqueueDelInterp(F2, vowelDelay, 2350.0f, vowelTransition);
                        enqueueDelInterp(F3, vowelDelay, 3000.0f, vowelTransition);
                        enqueueDelInterp(B1, vowelDelay, 60.0f, vowelTransition);
                        enqueueDelInterp(B2, vowelDelay, 90.0f, vowelTransition);
                        enqueueDelInterp(B3, vowelDelay, 200.0f, vowelTransition);
                        break;

                    case CODE_O:
                        enqueueDelInterp(FNZ, vowelDelay, 270.0f, vowelTransition);
                        enqueueDelInterp(F1, vowelDelay, 540.0f, vowelTransition);
                        enqueueDelInterp(F2, vowelDelay, 1100.0f, vowelTransition);
                        enqueueDelInterp(F3, vowelDelay, 2300.0f, vowelTransition);
                        enqueueDelInterp(B1, vowelDelay, 80.0f, vowelTransition);
                        enqueueDelInterp(B2, vowelDelay, 70.0f, vowelTransition);
                        enqueueDelInterp(B3, vowelDelay, 70.0f, vowelTransition);
                        break;

                    case CODE_AI:
                        // Front vowel
                        enqueueDelInterp(A2, 0.0f, 0.5f, 0.0f);
                        enqueueDelInterp(A2, 0.1f, 0.0f, 0.0f);

                        enqueueDelInterp(FNZ, vowelDelay, 270.0f, vowelTransition);
                        enqueueDelInterp(F1, vowelDelay, 850.0f, vowelTransition);
                        enqueueDelInterp(F2, vowelDelay, 1220.0f, vowelTransition);
                        enqueueDelInterp(F3, vowelDelay, 2810.0f, vowelTransition);
                        enqueueDelInterp(B1, vowelDelay, 80.0f, vowelTransition);
                        enqueueDelInterp(B2, vowelDelay, 70.0f, vowelTransition);
                        enqueueDelInterp(B3, vowelDelay, 70.0f, vowelTransition);

                        // Dipthong
                        enqueueDelInterp(F1, vowelDelay + 0.2f, 300.0f, 0.5f);
                        enqueueDelInterp(F2, vowelDelay + 0.2f, 2800.0f, 0.5f);
                        enqueueDelInterp(F3, vowelDelay + 0.2f, 3300.0f, 0.5f);
                        enqueueDelInterp(B1, vowelDelay + 0.2f, 45.0f, 0.5f);
                        enqueueDelInterp(B2, vowelDelay + 0.2f, 200.0f, 0.5f);
                        enqueueDelInterp(B3, vowelDelay + 0.2f, 400.0f, 0.5f);
                        break;

                    case CODE_EI:
                        // Front vowel
                        enqueueDelInterp(A2, 0.0f, 0.5f, 0.0f);
                        enqueueDelInterp(A2, 0.1f, 0.0f, 0.0f);

                        enqueueDelInterp(FNZ, vowelDelay, 270.0f, vowelTransition);
                        enqueueDelInterp(F1, vowelDelay, 600.0f, vowelTransition);
                        enqueueDelInterp(F2, vowelDelay, 2350.0f, vowelTransition);
                        enqueueDelInterp(F3, vowelDelay, 3000.0f, vowelTransition);
                        enqueueDelInterp(B1, vowelDelay, 60.0f, vowelTransition);
                        enqueueDelInterp(B2, vowelDelay, 90.0f, vowelTransition);
                        enqueueDelInterp(B3, vowelDelay, 200.0f, vowelTransition);

                        // Dipthong
                        enqueueDelInterp(F1, vowelDelay + 0.2f, 300.0f, 0.5f);
                        enqueueDelInterp(F2, vowelDelay + 0.2f, 2800.0f, 0.5f);
                        enqueueDelInterp(F3, vowelDelay + 0.2f, 3300.0f, 0.5f);
                        enqueueDelInterp(B1, vowelDelay + 0.2f, 45.0f, 0.5f);
                        enqueueDelInterp(B2, vowelDelay + 0.2f, 200.0f, 0.5f);
                        enqueueDelInterp(B3, vowelDelay + 0.2f, 400.0f, 0.5f);
                        break;

                    default:
                        break;
                    }

                } else {
                    if (pack.evnt1 >= 36 && pack.evnt1 <= 42) {
                        cvKeyboard ^= 1 << (42 - pack.evnt1);
                    } else if (pack.evnt1 == 43) {
                        cvQueueRear = (cvQueueRear == CV_QUEUE_CAP - 1) ? 0 : cvQueueRear + 1;
                        cvQueue[cvQueueRear] = cvKeyboard;
                        cvKeyboard = 0;
                        cvQueueSize++;
                    }
                }
            }
        }
        break;
    case 0xA0:	// Polyphonic Pressure
        break;
    case 0xB0:	// Control Change
        switch (pack.evnt1) { // CC number
        case 1:
            flutter = ((float) pack.evnt2) / 127.0f;
        case 19:
            formantParams[F0].duration = ((float) pack.evnt2 + 1.0f) / 128.0f;
            formantParams[A0].duration = formantParams[F0].duration;
            break;
        case 18:
            masterVolumeKnob = ((float) pack.evnt2) / 127.0f;
            break;
        case 16:
            waveshapperKnob = 1.0f - 2.0f * expf_fast(((float) pack.evnt2) / -25.0f);
            break;
        case 74:
            pResonFKnob = ((float) pack.evnt2) / 127.0f * 3000.0f;
            break;
        case 71:
            pResonBWKnob = ((float) pack.evnt2) / 127.0f * 4999.0f + 1.0f;
            break;
        case 76:
            pResonWetKnob = ((float) pack.evnt2) / 127.0f;
            break;
        case 77:
            pAResonFKnob = ((float) pack.evnt2) / 127.0f * 3000.0f;
            break;
        case 93:
            pAResonBWKnob = ((float) pack.evnt2) / 127.0f * 4999.0f + 1.0f;
            break;
        case 73:
            pAResonWetKnob = ((float) pack.evnt2) / 127.0f;
            break;
        }
        break;
    case 0xC0:	// Program Change
        break;
    case 0xD0:	// After Touch
        break;
    case 0xE0:	// Pitch Bend
        pitchbend = pack.evnt2 * 128 + pack.evnt1;
        break;
    }
}


float sinTaylor(float phase) {
    // sin x = x - x^3/3! + x^5/5! - x7/7! + .....
    // adapted from phil burk's code on musicdsp.org
    float tp, yp, x, x2;

    while (phase > 1.0f)
        phase -= 1.0f;

    tp = phase * 2.0f - 1.0f;
    // as the taylor series is an approximation, this uses the more accurate data

    if (tp > 0.5f)
        yp = 1.0f - tp;
    else {
        if (tp < -0.5f)
            yp = -1.0f - tp;
        else
            yp = tp;
    }
    x = yp * 3.141592653589793f;
    x2 = x * x;

    //Taylor expansion out to x**9/9! factored  into multiply-adds
    return (x * (x2 * (x2 * (x2 * (x2 * (0.000002755731922f) - (0.000198412698413f)) + (0.008333333333333f)) - (0.166666666666667f)) + 1.0f));
}
void tableInit(int32_t type) {
    // 0 - sine init, 1 - sawtooth init
    int i;
    tablesize = 4096;
    tablemask = tablesize - 1;
    for (i = 0; i < tablesize; i++)
        wavetable[i] = sinTaylor((float) i * 0.000244140625f);
}
float sinTable(float phase) {
    float frac;
    int32_t phint;

    // calculate the fractional part
    phint = (int32_t) (phase * tablesize);
    frac = phase * tablesize - phint;

    return (wavetable[phint & tablemask] * (1.0f - frac) + wavetable[(phint + 1) & tablemask] * frac);
}

float resonatorSamp(resonator* r, float in, float frequency, float bandwidth) {
    float out, e;
    if (frequency != r->f || bandwidth != r->bw) {
        r->f = frequency;
        r->bw = bandwidth;
        e = expf_fast(-bandwidth * PI_DT);
        r->c = -e * e;
        r->b = 2.0f * e * sinTable(2.0f * DELTA_TIME * frequency + 0.75f);
        r->a = 1.0f - r->c - r->b;
    }
    out = in * r->a + r->b * r->out1 + r->c * r->out2;
    r->out2 = r->out1;
    r->out1 = out;
    return out;
}
float antiresonatorSamp(resonator* r, float in, float frequency, float bandwidth) {
    float out, e;
    if (frequency != r->f || bandwidth != r->bw) {
        r->f = frequency;
        r->bw = bandwidth;
        e = expf_fast(-bandwidth * PI_DT);
        r->c = -e * e;
        r->b = 2.0f * e * sinTable(2.0f * DELTA_TIME * frequency + 0.75f);
        r->a = 1.0f / (1.0f - r->c - r->b);
    }
    out = (in - r->b * r->out1 - r->c * r->out2) * r->a;
    r->out2 = r->out1;
    r->out1 = in;
    return out;
}

void audioBlock(float *input, float *output, int32_t samples) {
    int i, j, v, midiPitch;
    float sig, sigp, sigc, noise, lfoSinNew, masterInc, waveshapeInc,
    frequency, midiMix, pResonFInc, pResonBWInc, pResonWetInc, pAResonFInc,
    pAResonBWInc, pAResonWetInc, out;

    masterInc = (masterVolumeKnob - masterVolume) * 0.0625f;
    waveshapeInc = (waveshapperKnob - waveshapper) * 0.0625f;
    pResonFInc = (pResonFKnob - pResonF) * 0.0625f;
    pResonBWInc = (pResonBWKnob - pResonBW) * 0.0625f;
    pResonWetInc = (pResonWetKnob - pResonWet) * 0.0625f;
    pAResonFInc = (pAResonFKnob - pAResonF) * 0.0625f;
    pAResonBWInc = (pAResonBWKnob - pAResonBW) * 0.0625f;
    pAResonWetInc = (pAResonWetKnob - pAResonWet) * 0.0625f;


    for (i = 0; i < samples; i += 2) {
        // all voice synth units
        masterVolume += masterInc;
        waveshapper += waveshapeInc;
        // complex LFO calculation
        // complex multiply by phase angle for sin output
        //lfoSinNew = angleReal * lfoSin - angleImag * lfoCos;
        // complex multiply by phase angle for cos output
        //lfoCos = angleReal * lfoCos + angleImag * lfoSin;
        //lfoSin = lfoSinNew;
        // correction for accumulated computation inaccuracies
        if (lfoSin < -1.0) lfoSin = -1.0;
        if (lfoSin > 1.0f) lfoSin = 1.0f;

        out = 0.0f;

        // per voice synth units
        if (voiceMode == POLYVOICE) {
            pResonF += pResonFInc;
            pResonBW += pResonBWInc;
            pResonWet += pResonWetInc;
            pAResonF += pAResonFInc;
            pAResonBW += pAResonBWInc;
            pAResonWet += pAResonWetInc;

            for (v = 0; v < 16; v++) {
                if (voice[v].active != INACTIVE) {
                    voice[v].frequency = mtoinc[voice[v].note] * pitchbend1024[pitchbend >> 4];

                    // sawtooth oscillator waveform calculation
                    voice[v].phase1 += voice[v].frequency;
                    if (voice[v].phase1 >= 1.0f) voice[v].phase1 = voice[v].phase1 - 1.0f;

                    sig = sinTable(voice[v].phase1);
                    sig = (sig > waveshapper) ? (sig - waveshapper) / (1.0f - waveshapper) : 0.0f;

                    out += sig * voice[v].volume * voice[v].velocity * 0.0009765625f * masterVolume;

                    // update the simple envelope
                    if (voice[v].active == ATTACK) {
                        voice[v].volume += 1.0f / attack * DELTA_TIME;
                        if (voice[v].volume >= 1.0f) voice[v].active = SUSTAIN;
                    } else if (voice[v].active == SUSTAIN)
                        voice[v].volume = sustain;
                    else if (voice[v].active == RELEASE) {
                        voice[v].volume -= 1.0f / release * DELTA_TIME;
                        if (voice[v].volume <= 0.0f) voice[v].active = INACTIVE;
                    } else if (voice[v].active == INACTIVE) {
                        voice[v].volume = 0.0f;
                    }
                }
            }

            // Apply output effects to polysynth
            out = pResonWet * resonatorSamp(&pResonator, out, pResonF, pResonBW) + (1.0f - pResonWet) * out;
            out = pAResonWet * antiresonatorSamp(&pAResonator, out, pAResonF, pAResonBW) + (1.0f - pAResonWet) * out;

        } else if (voiceMode == MONOVOICE) {
            // Check delays to parameters
            while (delQueueSize > 0 && delQueue[delQueueFront].start < delTime) {
                delayedInterp di = dequeueDelInterp();
                interp(&formantParams[di.param], di.value, di.duration);
            }
            delTime += DELTA_TIME;

            // Adjust parameters
            for (j = 0; j < NUM_FORMANT_PARAM; j++) {
                if (!formantParams[j].resting) {
                    formantParams[j].resting = formantParams[j].t > formantParams[j].duration;
                    if (!formantParams[j].resting) {
                        if (j <= F0) {
                            // Per sample param change
                            formantParams[j].value += formantParams[j].delta;
                            formantParams[j].t += DELTA_TIME;
                        } else if (i == 0){
                            // Batch param change (for center frequency and bandwidth)
                            formantParams[j].value += samples * formantParams[j].delta;
                            formantParams[j].t += samples * DELTA_TIME;
                        }
                    }
                }
            }

            // Calculate the fundamental frequency
            midiPitch = (int)formantParams[F0].value;
            midiMix = formantParams[F0].value - midiPitch;
            frequency = ((1.0f - midiMix) * mtoinc[midiPitch] + midiMix * mtoinc[midiPitch + 1]) * pitchbend1024[pitchbend >> 4];

            // Calculate quasirandom f0 fluctuation
            flutterp1 += flutterf1;
            if (flutterp1 > 1.0f) flutterp1 -= 1.0f;
            flutterp2 += flutterf2;
            if (flutterp2 > 1.0f) flutterp2 -= 1.0f;
            flutterp3 += flutterf3;
            if (flutterp3 > 1.0f) flutterp3 -= 1.0f;
            frequency += flutter * frequency * 0.01f * (sinTable(flutterp1) + sinTable(flutterp2) + sinTable(flutterp3));

            // Get the current phase of the source
            runningVoicePhase += frequency;
            if (runningVoicePhase > 1.0f) runningVoicePhase = runningVoicePhase - 1.0f;

            // Calculate waveform of voincing source
            sig = sinTable(runningVoicePhase);
            sig = (sig > waveshapper) ? (sig - waveshapper) / (1.0f - waveshapper) : 0.0f;

            // Amplitude envelope application and preliminary filtering
            sig = resonatorSamp(&resonators[RGP], sig, 0.0f, 200.0f);
            sig = formantParams[AV].value * antiresonatorSamp(&resonators[RGZ], sig, 1500.0f, 6000.0f) +
                    formantParams[AVS].value * resonatorSamp(&resonators[RGS], sig, 0.0f, 200.0f);

            // Noise source
            noise = resonatorSamp(&resonators[LPF], randf(), 0.0f, 1000.0f);

            // Track splitting
            sigc = formantParams[AH].value * noise + sig;
            noise = formantParams[AF].value * noise;

            // Cascade track
            sigc = resonatorSamp(&resonators[RNP], sigc, 270.0f, 50.0f);
            sigc = antiresonatorSamp(&resonators[RNZ], sigc, formantParams[FNZ].value, 50.0f);
            sigc = resonatorSamp(&resonators[RC1], sigc, formantParams[F1].value, formantParams[B1].value);
            sigc = resonatorSamp(&resonators[RC2], sigc, formantParams[F2].value, formantParams[B2].value);
            sigc = resonatorSamp(&resonators[RC3], sigc, formantParams[F3].value, formantParams[B3].value);
            sigc = resonatorSamp(&resonators[RC4], sigc, 3300.0f, 250.0f);
            sigc = resonatorSamp(&resonators[RC5], sigc, 3750.0f, 200.0f);

            // Parallel track
            sigp = 0.0f;
            sigp += formantParams[A2].value * resonatorSamp(&resonators[RP2], noise, formantParams[F2].value, formantParams[B2].value);
            sigp += formantParams[A3].value * resonatorSamp(&resonators[RP3], noise, formantParams[F3].value, formantParams[B3].value);
            sigp += formantParams[A4].value * resonatorSamp(&resonators[RP4], noise, 3300.0f, 250.0f);
            sigp += formantParams[A5].value * resonatorSamp(&resonators[RP5], noise, 3750.0f, 200.0f);
            sigp += formantParams[A6].value * resonatorSamp(&resonators[RP6], noise, 4900.0f, 1000.0f);
            sigp += formantParams[AB].value * noise;

            // Recombination and velocity application
            out = (sigp + sigc) * formantParams[A0].value * 0.0078125f * masterVolume;

            //out = sig * formantParams[A0].value * 0.0078125f * masterVolume;
        }

        // Set output
        output[i << 1] = output[(i << 1) + 1] = out;
    }
    masterVolume = masterVolumeKnob;
    waveshapper = waveshapperKnob;
    pResonF = pResonFKnob;
    pResonBW = pResonBWKnob;
    pResonWet = pResonWetKnob;
    pAResonF = pAResonFKnob;
    pAResonBW = pAResonBWKnob;
    pAResonWet = pAResonWetKnob;
}
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void) {
    int i;
    audioBlock(inBuffer, outBuffer, 16);
    for (i = 0; i < 32; i += 2) {
        codecBuffer[i + 0] = (int16_t) ((outBuffer[i]) * 32767.0f);
        codecBuffer[i + 1] = (int16_t) ((outBuffer[i + 1]) * 32767.0f);
    }
}

void BSP_AUDIO_OUT_TransferComplete_CallBack(void) {
    int i;
    BSP_AUDIO_OUT_ChangeBuffer((uint16_t*) codecBuffer, 64);
    audioBlock(inBuffer, outBuffer, 16);
    for (i = 0; i < 32; i += 2) {
        codecBuffer[i + 32] = (int16_t) ((outBuffer[i]) * 32767.0f);
        codecBuffer[i + 33] = (int16_t) ((outBuffer[i + 1]) * 32767.0f);
    }
}

void BSP_AUDIO_OUT_Error_CallBack(void) {
    /* Stop the program with an infinite loop */
    while (1) {
    }
}
/**
 * @brief  This function handles main I2S interrupt.
 * @param  None
 * @retval 0 if correct communication, else wrong communication
 */
void I2S3_IRQHandler(void) {
    HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx);
}

/**
 * @brief  This function handles DMA Stream interrupt request.
 * @param  None
 * @retval None
 */
void I2S2_IRQHandler(void) {
    HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}
/*====================================================================================================*/
/**
 * @brief  User Process function callback
 * @param  phost: Host Handle
 * @param  id: Host Library user message ID
 * @retval none
 */
static void USBH_UserProcess_callback(USBH_HandleTypeDef *pHost, uint8_t vId) {
    switch (vId) {
    case HOST_USER_SELECT_CONFIGURATION:
        break;

    case HOST_USER_DISCONNECTION:
        Appli_state = MIDI_APPLICATION_DISCONNECT;
        // 4 - green, 3 - orange, 5 - red, 6 - blue
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
        break;

    case HOST_USER_CLASS_ACTIVE:
        Appli_state = MIDI_APPLICATION_READY;
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
        break;

    case HOST_USER_CONNECTION:
        Appli_state = MIDI_APPLICATION_START;
        HAL_GPIO_WritePin(GPIOD, 0x8000, GPIO_PIN_SET);
        break;

    default:
        break;

    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
