#include "system.h"

uint32_t system_time;
int SP1=0,SP2=0,SP3=0;//任务指针
uint8_t IF_error;
long system_tick;
int error_register[error_num];
uint8_t Rx[22];
/**
  * @brief  单片机时钟树
  * @param  void
  * @retval void
  * @attention 
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief  系统的各种初始化，赋值
  * @param  void
  * @retval void
  * @attention 
  */
void init_system(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
	MX_USART6_UART_Init();
    MX_CAN2_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    
    SYSTEM_Reset();
    GIMBAL_InitArgument();    
    dbus_uart_init();
    drv_can_init();    
    shoot_init(); 
    Magazine_init();
    REVOLVER_InitArgument();    
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);//开启24V电压
	
    HAL_UART_Receive_DMA(&huart6, Rx, 22);//串口6
    __HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
    
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//pwm开启
    
    system_tick=0;
}



/**
  * @brief  系统开启
  * @param  void
  * @retval void
  * @attention 
  */
void System_Run(void)
{
    __HAL_TIM_ENABLE_IT(System_base_tim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(System_base_tim);
}

/**
  * @brief  系统关闭
  * @param  void
  * @retval void
  * @attention 
  */
void System_Stop(void)
{
    __HAL_TIM_DISABLE_IT(System_base_tim, TIM_IT_UPDATE);


    __HAL_TIM_DISABLE(System_base_tim);
}
/**
  * @brief  时间片任务调度函数
  * @param  void
  * @retval void
  * @attention 放入TIM3中使用，定时器中断优先级应高于其他中断，定时器时间应偏长
  */
void Task_Dispatch_tim(void)
{
    if (__HAL_TIM_GET_FLAG(System_base_tim, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(System_base_tim, TIM_IT_UPDATE) != RESET)
        {
        __HAL_TIM_CLEAR_IT(System_base_tim, TIM_IT_UPDATE);
        gpio_on;
        system_tick++;
        switch(SP1)
        {
            
            case 0:
                if(GIMBAL_InitCtrl())                
                        SP1=1;
                    else                    
                        task_PID_out();                    
            break;
            
            case 1:
                Gimbal_value();
                if (SYSTEM_GetRemoteMode() == RC)//遥控控制模式
                    {
                        GIMBAL_Rc_Ctrl();//遥控模式
                        actGimbal = GIMBAL_NORMAL;
                    }
                    else
                    {
                        GIMBAL_Key_Ctrl();//键盘模式
                    }
                    task_PID_out();
                    
            break;
        }            

        Task_Revolver();
        Shoot_Ctrl();
        Magazine_Ctrl();        
        sent_gimbal_to_down(GIMBAL_GetOffsetAngle());
       gpio_off;
        }
    }    
}

/**
  * @brief  错误处理函数
  * @param  void
  * @retval void
  * @attention 放入while(1)
  */
void Task_error(void)
{
    if(IF_error)
    {
        System_Stop();
        
    }
}

/**********************系统控制判断及保护******************/


//控制模式
eRemoteMode remoteMode = RC;

//系统状态
eSystemState systemState = SYSTEM_STARTING;


/**
  * @brief  系统重置
  * @param  void
  * @retval void
  * @attention 
  */
void SYSTEM_Reset( void )
{
	systemState = SYSTEM_STARTING;
}

/**
  * @brief  失控保护
  * @param  void
  * @retval void
  * @attention 
  */
//void SYSTEM_OutCtrlProtect(void)
//{
//    SYSTEM_Reset();//系统恢复至重启状态
//	REMOTE_vResetData();//遥控数据恢复至默认状态
//	Laser_Off;//激光关
//	CHASSIS_StopMotor();//底盘关
//	GIMBAL_StopMotor();//云台关
//	Magazine_StopCtrl();//舵机停止转动
//	REVOLVER_StopMotor();//拨盘停止转动
//	FRICTION_StopMotor();//摩擦轮关
//	Super_Cap_StopCtrl();//电容关闭充放电
//}



/**
  * @brief  拨杆模式选择
  * @param  void
  * @retval void
  * @attention 键盘或鼠标,可在此自定义模式选择方式,1ms执行一次
  */
void SYSTEM_UpdateRemoteMode( void )
{ 
    if (IF_RC_SW2_UP)
	{
		remoteMode = KEY;
	}
	
	else
	{
		remoteMode = RC;
	}
}


//返回控制模式
eRemoteMode SYSTEM_GetRemoteMode()
{
	return remoteMode;
}

//返回系统状态
eSystemState SYSTEM_GetSystemState()
{
	return systemState;

}
