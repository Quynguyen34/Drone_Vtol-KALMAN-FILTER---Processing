/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Nguyen Ngoc Quy HCMUTE K20.
  * Mechatronic Engineering.
  * Gmail: quynguyenlk3402@gmail.com
  * Phone Number: +84 818 711 550
  * All rights reserved.
  * Introducing Our Team's VTOL Drone Copyright: Elevating Aerial Innovation.
  * This software is licensed under terms that can be found in the LICENSE file
  * Join us on this exciting journey as we share the future of aerial technology
  * Our VTOL Drone Copyright symbolizes progress, sustainability, and positive impact 
  * on a global scale.
  * 
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t buffer[10];
uint8_t rx_buffer[8];
int a = 1051;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*!---Register MPU6050---*/
#define MPU6050_ADDR                    0xD0
#define SMPLRT_DIV_REG                  0x19

/* Config angle */
#define GYRO_CONFIG_REG                 0X1B
/* Degree X */
#define GYRO_XOUT_H_REG                 0x43
#define GYRO_XOUT_L_REG                 0x44
/* Deree Y */
#define GYRO_YOUT_H_REG                 0x45
#define GYRO_YOUT_L_REG                 0x46
/* Degree Z */
#define GYRO_ZOUT_H_REG                 0x47
#define GYRO_ZOUT_L_REG                 0x48

/* Config Acceleration */
#define ACCEL_CONFIG_REG                0x1C
/* Axis X */
#define ACCEL_XOUT_H_REG                0x3B
#define ACCEL_XOUT_L_REG                0x3C
/* Axis Y */
#define ACCEL_YOUT_H_REG                0x3D
#define ACCEL_YOUT_L_REG                0x3E
/* Axis Z */
#define ACCEL_ZOUT_H_REG                0x3F
#define ACCEL_ZOUT_L_REG                0x40
  
#define TEMP_OUT_H_REG                  0x41
#define PWR_MGMT_1_REG                  0x6B
#define WHO_AM_I_REG                    0x75
/* Kp,Ki cho imu */
#define Kp 0.01f
#define Ki 0.002f
#define halfT 0.001f
#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.0174532925199432957692369076848
#define timeout 100

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int AngX;
int AngY;
int AngZ;
uint16_t timer = 0;

float Ax, Ay, Az, Gx, Gy, Gz, W;
int Roll, Pitch, Yaw;
int RateCalibrationNumber;
float RateCalibrateRoll, RateCalibratePitch, RateCalibrateYaw;

/*!--- Kalman filter ---!*/
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

/*!--- PWM ---!*/
#define ANGLE_MIN               (float)0
#define ANGLE_MAX               (float)180

#define PWM_PULSE_WIDTH_MIN     (float)710 // us
#define PWM_PULSE_WIDTH_CENTER  (float)1520 // us
#define PWM_PULSE_WIDTH_MAX     (float)2660 // us

#define PWM_PULSE_VALUE_0       (float)76 // 50 Hz
#define PWM_PULSE_MIN_0         PWM_PULSE_VALUE_0 * PWM_PULSE_WIDTH_MIN / PWM_PULSE_WIDTH_CENTER
#define PWM_PULSE_MAX_0         PWM_PULSE_VALUE_0 * PWM_PULSE_WIDTH_MAX / PWM_PULSE_WIDTH_CENTER

int bldc_left, bldc_right, canhta_left, canhta_right, canhduoi, canhgap_left, canhgap_right;
int test, goc;

/*!--- PID ---*/
int Time, Time_Previous;
float PID_Return[]={0,0,0};
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
// Sameple PID
float PRateRoll=0.6 ; float PRatePitch=0.6; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=3.5; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=0.03; float DRateYaw=0;
float MotorInput1, MotorInput2;
float DesiredAngleRoll, DesiredAnglePitch;
float ErrorAngleRoll, ErrorAnglePitch;
float PrevErrorAngleRoll, PrevErrorAnglePitch;
float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2; float PAnglePitch=2;
float IAngleRoll=0; float IAnglePitch=0;
float DAngleRoll=0; float DAnglePitch=0;

/*!--- Kalman ---!*/
//Kalman 1D
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (
  KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

/*!--- IMU ---!*/
void MPU6050_Init(void)
{
  uint8_t check;
  uint8_t Data;
  // Check device ID WHO_AM_I
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG,1, &check, 1, timeout);
  if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
  {
    // Power management register 0x6B we should write all 0's to wake the sensor up
    Data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, timeout);
    // Set data rate of 1kHz by writing SMPLRT_DIV register
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, timeout);
    // Set accelrometer configuration in ACCEL_CONFIG Register
    // XA_ST = 0, YA_ST = 0, ZA_ST = 0, FS_SEL = 0 -> +- 2g
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, timeout);
    // Set Gyroscopic configuration in GYRO_CONFIG Register
    // XA_ST = 0, YG_ST = 0, ZG_ST = 0, FS_SEL = 0 -> +- 250 degree/s
    Data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, timeout);
  }
}

void MPU6050_Read_Accel (void)
{
    uint8_t Rec_Data[6];
    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
    HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, timeout);
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
    // Implamentation value Accel
    Ax = Accel_X_RAW/16384.0;
    Ay = Accel_Y_RAW/16384.0;
    Az = Accel_Z_RAW/16384.0;
}

void MPU6050_Read_Gyro (void)
{
    uint8_t Rec_Data[6];
    // Read 6 BYTES of data starting from GYRO_XOUT_H register
    HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, timeout);
    Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
    // Implamentation value Gyro
    Gx = Gyro_X_RAW/131.0;
    Gy = Gyro_Y_RAW/131.0;
    Gz = Gyro_Z_RAW/131.0;
}

void MPU6050_Read_All(void)
{
  MPU6050_Read_Accel();
  MPU6050_Read_Gyro();
  Roll = atan(Ay/sqrt(Ax*Ax+Az*Az))*1/(DEG_TO_RAD);
  Pitch = -atan(Ax/sqrt(Ay*Ay+Az*Az))*1/(DEG_TO_RAD);
  Yaw = Gz;
}

void calibrate_MPU6050(void)
{
  for(RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    MPU6050_Read_All();
    RateCalibrateRoll+=Gx;
    RateCalibratePitch+=Gy;
    RateCalibrateYaw+=Gz;
    HAL_Delay(1);
  }
  RateCalibrateRoll/=2000;
  RateCalibratePitch/=2000;
  RateCalibrateYaw/=2000;
}

/*!--- PWM ---!*/
double map(float Value, float Input_Min, float Input_Max, float Output_Min, float Output_Max)
{
  if (Value < Input_Min) Value = Input_Min;
  if (Value > Input_Max) Value = Input_Max;
  double scale = (Output_Max - Output_Min) / (Input_Max - Input_Min);
  double result = Output_Min + (Value - Input_Min)*scale;
  return result;
}       
/*
void set_angle(void)
{
  //BLDC
  htim1.Instance->CCR1 = bldc_left;
  htim1.Instance->CCR2 = bldc_right;
  //Canh Ta
  htim2.Instance->CCR1 = canhta_left;
  htim2.Instance->CCR2 = canhta_right;
  //Canh gap
  htim3.Instance->CCR1 = canhgap_left;
  htim3.Instance->CCR2 = canhgap_right;
  //Canh Duoi
  htim4.Instance->CCR3 = canhduoi;
}
*/
void Servo_set_angle(int Angle, int channel)
{
    if (Angle!= 255 )
    {
      if(channel==1)
      {
        htim1.Instance->CCR1 = lroundf(map(Angle,
                                 ANGLE_MIN, ANGLE_MAX,
                                 PWM_PULSE_MIN_0, PWM_PULSE_MAX_0));
      }
      if(channel==2)
      {
        htim1.Instance->CCR2 = lroundf(map(Angle,
                                 ANGLE_MIN, ANGLE_MAX,
                                 PWM_PULSE_MIN_0, PWM_PULSE_MAX_0));
      }
   
    }
    else 
    {
      if(channel==1)
      {
        htim1.Instance->CCR1 = 0;
      }
      if(channel==2)
      {
        htim1.Instance->CCR2 = 0;
      }
    }
}

/*!--- PID Control ---!*/
// PID EQUATION
void PID_Equation(float Error, float Error_Previous, float P, float I, float D, float Iterm_Previous)
{ 
  /*
   Time_Previous = Time;
   Time = HAL_GetTick();
   int Real_Time = ((Time - Time_Previous)/1000); // seconds
   */
  
   float P_term = P*Error;
   float I_term = Iterm_Previous + I*(Error + Error_Previous)*0.02/2;
   if (I_term > 20) I_term = 20;
   else if (I_term < 20) I_term = -20;
   float D_term = D*((Error - Error_Previous)/0.02);
   float PID_Out = P_term + I_term + D_term;
   if (PID_Out > 20) PID_Out = 20;
   else if (PID_Out < 20) PID_Out = -20;
   PID_Return[0] = PID_Out;
   PID_Return[1] = Error;
   PID_Return[2] = I_term;   
}
// RESET PID
void Reset_PID(void)
{
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

// PID ANGLE CONTROL
void PID_Angle_Control(void)
{ 
  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;
  // PID ANGLE ROLL
  PID_Equation(ErrorAngleRoll, PrevErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PID_Return[0];
  PrevErrorAngleRoll = PID_Return[1];
  PrevItermAngleRoll = PID_Return[2];
  // PID ANGLE PITCH
  PID_Equation(ErrorAnglePitch, PrevErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PID_Return[0];
  PrevErrorAnglePitch = PID_Return[1];
  PrevItermAnglePitch = PID_Return[2];
}

// PID CONTROL
void PID_Control(void)
{
  ErrorRateRoll=DesiredRateRoll-Gx;
  ErrorRatePitch=DesiredRatePitch-Gy;
  ErrorRateYaw=DesiredRateYaw-Gz;
  // PID ROLL
  PID_Equation(ErrorRateRoll, PrevErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevItermRateRoll);
  InputRoll = PID_Return[0];
  PrevErrorRateRoll = PID_Return[1];
  PrevItermRateRoll = PID_Return[2];
  // PID PITCH
  PID_Equation(ErrorRatePitch, PrevErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevItermRatePitch);
  InputPitch = PID_Return[0];
  PrevErrorRatePitch = PID_Return[1];
  PrevItermRatePitch = PID_Return[2];
  // PID YAW
  PID_Equation(ErrorRateYaw, PrevErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevItermRateYaw);
  InputYaw = PID_Return[0];
  PrevErrorRateYaw = PID_Return[1];
  PrevItermRateYaw = PID_Return[2];
  /*
  // Control
  if (InputThrottle > 1800) InputThrottle = 1800;
  bldc_left = 0.20475*(InputThrottle-InputRoll-InputPitch-InputYaw);
  bldc_right = 0.20475*(InputThrottle-InputRoll+InputPitch+InputYaw);
  if (bldc_left > 2000) bldc_left = 1999;
  if (bldc_right > 2000) bldc_right = 1999;
  int ThrottleIdle = 1180;
  if (bldc_left < ThrottleIdle) bldc_left = ThrottleIdle;
  if (bldc_right < ThrottleIdle) bldc_right = ThrottleIdle;
  int ThrottleCutOff = 1000;
  if (a<1050)
  {
    bldc_left = ThrottleCutOff;
    bldc_right = ThrottleCutOff;
    Reset_PID();
  }
*/
 Servo_set_angle(bldc_left,1);
 Servo_set_angle(bldc_right,2);
}


/*!--- UART ---!*/
/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
 if(huart->Instance==USART2)
 {
     HAL_UARTEx_ReceiveToIdle_IT(&huart3,rx_buffer,8);
 } 
}
void recevie_data(void)
{
  if (rx_buffer[0] == 0x32)
  {
    bldc_left = rx_buffer[1];
    bldc_right = rx_buffer[2];
    canhta_left = rx_buffer[3];
    canhta_right = rx_buffer[4];
    canhgap_left = rx_buffer[5];
    canhgap_right = rx_buffer[6];
    canhduoi = rx_buffer[7];
    set_angle();
  }
  HAL_Delay(50);
}
*/
/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3)
  {
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, buffer, 10);
    if (Size == 9)
    {
      if (buffer[0] == 0xAA)
      {
        for (uint8_t channel = 0; channel < 8; channel ++)
        {
          Angle[channel] = buffer[channel + 1];
        }
      }
    }
  }
}
*/
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  //HAL_UARTEx_ReceiveToIdle_IT(&huart3,rx_buffer,7);
  
  MPU6050_Init();
  calibrate_MPU6050();
  HAL_Delay(2000);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*!--- Test IMU ---!*/
    MPU6050_Read_All();
    Gx-=RateCalibrateRoll;
    Gy-=RateCalibratePitch;
    Gz-=RateCalibrateYaw;
    
    /*!--- Kalman ---!*/
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, Gx, Roll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, Gy, Pitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    
    /*!--- USART ---!*/
    /*
    recevie_data();
    DesiredRateRoll=0.15*(rx_buffer[1]-1500);
    DesiredRatePitch=0.15*(rx_buffer[2]-1500);
    InputThrottle=rx_buffer[3];
    DesiredRateYaw=0.15*(rx_buffer[4]-1500);
    */
    
    /*!--- Rate Roll_Pitch_Yaw ---!*/
    PID_Angle_Control();
    /*!--- Test pwm ---!*/
    PID_Control();
    /*
    Servo_set_angle(test,1);
    Servo_set_angle(test,2);
    */
    //set_angle();
    //IMU_Update();
    //HAL_Delay(50);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 3359;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1679;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1679;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1679;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
