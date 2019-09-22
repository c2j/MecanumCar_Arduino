#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" /* MPU6050库文件 */
#include "Wire.h"                       /* Ic总线相关库文件 */
#include <SSD1306.h>                    /* OLED显示器库文件 */
#include <PS2X_lib.h>                   /* PS2手柄控制库函数文件 */
#include <DATASCOPE.h>                  /* 这是PC端上位机的库文件 */
#include <PinChangeInt.h>               /* 外部中断 */
#include <FlexiTimer2.h>                /* 定时中断 */
DATASCOPE  data;                   /* 实例化一个 上位机 对象，对象名称为 data */
MPU6050   Mpu6050;                /* 实例化一个 MPU6050 对象，对象名称为 Mpu6050 */
PS2X    ps2x;                   /* create PS2 Controller Class */
/* ////////PS2引脚////////////////// */
#define PS2_DAT 49
#define PS2_CMD 47
#define PS2_SEL 43
#define PS2_CLK 41
/* //////编码器引脚/////////// */
#define ENCODER_A 2               /* A路电机编码器引脚 */
#define ENCODER_B 3               /* B路电机编码器引脚 */
#define ENCODER_C 18              /* C路电机编码器引脚 */
#define ENCODER_D 19              /* D路电机编码器引脚 */
#define DIRECTION_A 51              /* A路电机编码器引脚 */
#define DIRECTION_B 53              /* B路电机编码器引脚 */
#define DIRECTION_C 52              /* C路电机编码器引脚 */
#define DIRECTION_D 50              /* D路电机编码器引脚 */
/* //////OLED显示屏引脚相关设置/////////// */
#define OLED_DC   22
#define OLED_CLK  28
#define OLED_MOSI 26
#define OLED_RESET  24
SSD1306 oled( OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0 );


/* ******************PWM引脚和电机驱动引脚*************************** // */
int AIN1  = 5;                                                                                                                                            /* A电机PWM波 */
int AIN2  = 6;                                                                                                                                            /* A电机PWM波 */
int BIN1  = 8;                                                                                                                                            /* B电机PWM波 */
int BIN2  = 7;                                                                                                                                            /* B电机PWM波 */
int CIN1  = 11;                                                                                                                                           /* C电机PWM波 */
int CIN2  = 12;                                                                                                                                           /* C电机PWM波 */
int DIN1  = 44;                                                                                                                                           /* D电机PWM波 */
int DIN2  = 46;                                                                                                                                           /* D电机PWM波 */
/* ********************************************* // */
int   Transformation;                                                                                                                                 /* APP摇杆-按键控制模式切换标识 */
int   Plug;                                                                                                                                           /* 手柄数据相关标志 */
float   Velocity_KP = 1.05, Velocity_KI = 0.015;                                                                                                    /* PI参数 */
int   Flag_Way  = 1, Flag_Direction;                                                                                                            /* 遥控模式标志与蓝牙窜口通信相关标志 */
int   PS2_Velocity, Battery_Voltage, Bluetooth_Velocity = 330;                                                                                        /* 电池电压采样变量和上位机相关变量 */
float   PWMA, PWMB, PWMC, PWMD, Multiple, Target_A, Target_B, Target_C, Target_D;                                                                       /* 速度控制器倍率参数，目标参数 */
volatile long Velocity_1, Velocity_2, Velocity_3, Velocity_4;                                                                                                 /* 编码器数据 */
float   Velocity_A, Velocity_B, Velocity_C, Velocity_D, Velocity__A, Velocity__B, Velocity__D, Velocity__C, VelocityA, VelocityB, VelocityC, VelocityD; /* 左右轮速度 */
unsigned char PS2_LY, PS2_RX, PS2_LX, PS2_RY;                                                                                                                 /* 手柄遥控参数 */
char    Yuzhi = 40;                                                                                                                                     /* 摇杆角度调整参数 */
float   LY, RX, LX;                                                                                                                                     /* 手柄速度左摇杆Y轴，右摇杆X轴，左摇杆X轴方向取值参数 */
int16_t   ax, ay, az, gx, gy, gz;                                                                                                                         /* MPU6050的三轴加速度和三轴陀螺仪数据 */
static float  Angle1, Angle2, Angle, Gryo;                                                                                                                    /* 用于显示的角度和临时变量 */


/************************************************************************
 * 函数功能：求次方的函数
 * 入口参数：m,n
 * 返回  值：m的n次幂
 **************************************************************************/
uint32_t oled_pow( uint8_t m, uint8_t n )
{
  uint32_t result = 1;
  while ( n-- )
    result *= m;
  return(result);
}


/**************************************************************************
*  函数功能：OLED显示变量函数
*  入口参数：x:x坐标   y:行     num：显示的变量   len ：变量的长度
**************************************************************************/
void OLED_ShowNumber( uint8_t x, uint8_t y, uint32_t num, uint8_t len )
{
  uint8_t t, temp;
  uint8_t enshow = 0;
  for ( t = 0; t < len; t++ )
  {
    temp = (num / oled_pow( 10, len - t - 1 ) ) % 10;
    oled.drawchar( x + 6 * t, y, temp + '0' );
  }
}


/**************************************************************************
*  函数功能：赋值给PWM寄存器 作者：平衡小车之家
*  入口参数：PWM
**************************************************************************/
void Set_PWM( int motora, int motorb, int motorc, int motord )
{
  if ( motora > 0 )
    analogWrite( AIN2, motora + 100 ), analogWrite( AIN1, 0 );      /* 赋值给PWM寄存器根据电机响应速度与机械误差微调, */
  else if ( motora == 0 )
    analogWrite( AIN2, 0 ), analogWrite( AIN1, 0 );
  else if ( motora < 0 )
    analogWrite( AIN1, -motora + 100 ), analogWrite( AIN2, 0 );     /* 高频时电机启动初始值高约为130，低频时电机启动初始值低约为30 */

  if ( motorb > 0 )
    analogWrite( BIN2, motorb + 100 ), analogWrite( BIN1, 0 );      /* 赋值给PWM寄存器根据电机响应速度与机械误差微调, */
  else if ( motorb == 0 )
    analogWrite( BIN2, 0 ), analogWrite( BIN1, 0 );
  else if ( motorb < 0 )
    analogWrite( BIN1, -motorb + 100 ), analogWrite( BIN2, 0 );     /* 高频时电机启动初始值高约为130，低频时电机启动初始值低约为30 */

  if ( motorc > 0 )
    analogWrite( CIN1, motorc + 100 ), analogWrite( CIN2, 0 );      /* 赋值给PWM寄存器根据电机响应速度与机械误差微调, */
  else if ( motorc == 0 )
    analogWrite( CIN2, 0 ), analogWrite( CIN1, 0 );
  else if ( motorc < 0 )
    analogWrite( CIN2, -motorc + 100 ), analogWrite( CIN1, 0 );     /* 高频时电机启动初始值高约为130，低频时电机启动初始值低约为30 */

  if ( motord > 0 )
    analogWrite( DIN1, motord + 100 ), analogWrite( DIN2, 0 );      /* 赋值给PWM寄存器根据电机响应速度与机械误差微调, */
  else if ( motord == 0 )
    analogWrite( DIN1, 0 ), analogWrite( DIN2, 0 );
  else if ( motord < 0 )
    analogWrite( DIN2, -motord + 100 ), analogWrite( DIN1, 0 );     /* 高频时电机启动初始值高约为130，低频时电机启动初始值低约为30 */
}


/**************************************************************************
*  函数功能：增量PI控制器
*  入口参数：编码器测量值，目标速度
*  返回  值：电机PWM
*  根据增量式离散PID公式
*  pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
*  e(k)代表本次偏差
*  e(k-1)代表上一次的偏差  以此类推
*  pwm代表增量输出
*  在我们的速度控制闭环系统里面，只使用PI控制
*  pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/

int Incremental_PI_A( int Encoder, int Target )
{
  float   Bias;
  static float  PWM, Last_bias;
  Bias  = Encoder - Target;                                                                     /* 计算偏差 */
  PWM += Velocity_KP / Multiple * (Bias - Last_bias) + Velocity_KI / Multiple / 1.314 * Bias; /* 增量式PI控制器 */
  if ( PWM > 155 )
    PWM = 155;                                                                              /* 限幅 */
  if ( PWM < -155 )
    PWM = -155;                                                                             /* 限幅 */
  Last_bias = Bias;                                                                               /* 保存上一次偏差 */
  if ( Flag_Way == 1 )
  {
    if ( Flag_Direction == 0 )
    {
      if ( Target > 0 )
        PWM = -2, Last_bias = 0;
      if ( Target < 0 )
        PWM = 2, Last_bias = 0;
    }
  }                                                                                                            /* APP模式停止时，重置相关的变量，反向赋值降低小车惯性移动距离 */
  else if ( Flag_Way == 0 )
  {
    if ( abs( PS2_Velocity ) <= 51 )
    {
      if ( Target > 0 )
        PWM = -2, Last_bias = 0;
      if ( Target < 0 )
        PWM = 2, Last_bias = 0;
    }
  }                                                       /* 手柄停止时，重置相关的变量，反向赋值降低小车惯性移动距离 */
  return(PWM);                                            /* 增量输出 */
}


int Incremental_PI_B( int Encoder, int Target )
{
  float   Bias;
  static float  PWM, Last_bias;
  Bias  = Encoder - Target;                                                                     /* 计算偏差 */
  PWM += Velocity_KP / Multiple * (Bias - Last_bias) + Velocity_KI / Multiple / 1.314 * Bias; /* 增量式PI控制器 */
  if ( PWM > 155 )
    PWM = 155;                                                                              /* 限幅 */
  if ( PWM < -155 )
    PWM = -155;                                                                             /* 限幅 */
  Last_bias = Bias;                                                                               /* 保存上一次偏差 */
  if ( Flag_Way == 1 )
  {
    if ( Flag_Direction == 0 )
    {
      if ( Target > 0 )
        PWM = -2, Last_bias = 0;
      if ( Target < 0 )
        PWM = 2, Last_bias = 0;
    }
  }                                                                                                            /* APP模式停止时，重置相关的变量，反向赋值降低小车惯性移动距离 */
  else if ( Flag_Way == 0 )
  {
    if ( abs( PS2_Velocity ) <= 51 )
    {
      if ( Target > 0 )
        PWM = -2, Last_bias = 0;
      if ( Target < 0 )
        PWM = 2, Last_bias = 0;
    }
  }                                                       /* 手柄停止时，重置相关的变量，反向赋值降低小车惯性移动距离 */
  return(PWM);                                            /* 增量输出 */
}


/**************************************************************************/
int Incremental_PI_C( int Encoder, int Target )
{
  float   Bias;
  static float  PWM, Last_bias;
  Bias  = Encoder - Target;                                                                     /* 计算偏差 */
  PWM += Velocity_KP / Multiple * (Bias - Last_bias) + Velocity_KI / Multiple / 1.314 * Bias; /* 增量式PI控制器 */
  if ( PWM > 155 )
    PWM = 155;                                                                              /* 限幅 */
  if ( PWM < -155 )
    PWM = -155;                                                                             /* 限幅 */
  Last_bias = Bias;                                                                               /* 保存上一次偏差 */
  if ( Flag_Way == 1 )
  {
    if ( Flag_Direction == 0 )
    {
      if ( Target > 0 )
        PWM = -2, Last_bias = 0;
      if ( Target < 0 )
        PWM = 2, Last_bias = 0;
    }
  }                                                                                                            /* APP模式停止时，重置相关的变量，反向赋值降低小车惯性移动距离 */
  else if ( Flag_Way == 0 )
  {
    if ( abs( PS2_Velocity ) <= 51 )
    {
      if ( Target > 0 )
        PWM = -2, Last_bias = 0;
      if ( Target < 0 )
        PWM = 2, Last_bias = 0;
    }
  }                                                       /* 手柄模式停止时，重置相关的变量，反向赋值降低小车惯性移动距离 */
  return(PWM);                                            /* 增量输出 */
}


/**************************************************************************/
int Incremental_PI_D( int Encoder, int Target )
{
  float   Bias;
  static float  PWM, Last_bias;
  Bias  = Encoder - Target;                                                                     /* 计算偏差 */
  PWM += Velocity_KP / Multiple * (Bias - Last_bias) + Velocity_KI / Multiple / 1.314 * Bias; /* 增量式PI控制器 */
  if ( PWM > 155 )
    PWM = 155;                                                                              /* 限幅 */
  if ( PWM < -155 )
    PWM = -155;                                                                             /* 限幅 */
  Last_bias = Bias;                                                                               /* 保存上一次偏差 */
  if ( Flag_Way == 1 )
  {
    if ( Flag_Direction == 0 )
    {
      if ( Target > 0 )
        PWM = -2, Last_bias = 0;
      if ( Target < 0 )
        PWM = 2, Last_bias = 0;
    }
  }                                                                                                            /* APP模式停止时，重置相关的变量，反向赋值降低小车惯性移动距离 */
  else if ( Flag_Way == 0 )
  {
    if ( abs( PS2_Velocity ) <= 51 )
    {
      if ( Target > 0 )
        PWM = -2, Last_bias = 0;
      if ( Target < 0 )
        PWM = 2, Last_bias = 0;
    }
  }                                                       /* 手柄停止时，重置相关的变量，反向赋值降低小车惯性移动距离 */
  return(PWM);                                            /* 增量输出 */
}


void Angular_Speed_Control()
{
  if ( Flag_Direction == 0 )
    PWMA = -Gryo * Bluetooth_Velocity / 3300, PWMB = -Gryo * Bluetooth_Velocity / 3300, PWMC = +Gryo * Bluetooth_Velocity / 3300, PWMD = +Gryo * Bluetooth_Velocity / 3300;
  else if ( Flag_Direction == 1 )
    PWMA = -Gryo * Bluetooth_Velocity / 3300, PWMB = -Gryo * Bluetooth_Velocity / 3300, PWMC = +Gryo * Bluetooth_Velocity / 3300, PWMD = +Gryo * Bluetooth_Velocity / 3300;
  if ( Transformation == 0 )
  {
    if ( Flag_Direction == 5 )
      PWMA = -Gryo * Bluetooth_Velocity / 3300, PWMB = -Gryo * Bluetooth_Velocity / 3300, PWMC = +Gryo * Bluetooth_Velocity / 3300, PWMD = +Gryo * Bluetooth_Velocity / 3300;
    else if ( Flag_Direction == 7 )
      PWMA = -Gryo * Bluetooth_Velocity / 3300, PWMB = -Gryo * Bluetooth_Velocity / 3300, PWMC = +Gryo * Bluetooth_Velocity / 3300, PWMD = +Gryo * Bluetooth_Velocity / 3300;
    else if ( Flag_Direction == 3 )
      PWMA = -Gryo * Bluetooth_Velocity / 3300, PWMB = -Gryo * Bluetooth_Velocity / 3300, PWMC = +Gryo * Bluetooth_Velocity / 3300, PWMD = +Gryo * Bluetooth_Velocity / 3300;
    else if ( Flag_Direction == 8 )
      PWMA = -Gryo * Bluetooth_Velocity / 3000, PWMB = 0, PWMC = +Gryo * Bluetooth_Velocity / 3000, PWMD = 0;
    else if ( Flag_Direction == 4 )
      PWMA = -Gryo * Bluetooth_Velocity / 3000, PWMB = 0, PWMC = +Gryo * Bluetooth_Velocity / 3000, PWMD = 0;
    else if ( Flag_Direction == 2 )
      PWMA = 0, PWMB = -Gryo * Bluetooth_Velocity / 3000, PWMC = 0, PWMD = +Gryo * Bluetooth_Velocity / 3000;
    else if ( Flag_Direction == 6 )
      PWMA = 0, PWMB = -Gryo * Bluetooth_Velocity / 3000, PWMC = 0, PWMD = +Gryo * Bluetooth_Velocity / 3000;
  }
}


/**************************************************************************
 * 函数功能：异常关闭电机
 * 入口参数：电压
 * 返回  值：1：异常  0：正常
 * /**************************************************************************/
unsigned char  Turn_Off()
{
  byte temp;
  if ( Battery_Voltage < 1100 )           /* 电压太低关闭电机 */
  {
    temp = 1;
    digitalWrite( AIN1, LOW );      /* 电机驱动的电平控制 */
    digitalWrite( AIN2, LOW );      /* 电机驱动的电平控制 */
    digitalWrite( BIN1, LOW );      /* 电机驱动的电平控制 */
    digitalWrite( BIN2, LOW );      /* 电机驱动的电平控制 */
    digitalWrite( CIN1, LOW );      /* 电机驱动的电平控制 */
    digitalWrite( CIN2, LOW );      /* 电机驱动的电平控制 */
    digitalWrite( DIN1, LOW );      /* 电机驱动的电平控制 */
    digitalWrite( DIN2, LOW );      /* 电机驱动的电平控制 */
  }else temp = 0;
  return(temp);
}


/*********函数功能：10ms控制函数 核心代码 作者：平衡小车之家*******/
void control()
{
  int     Motora, Motorb, Motorc, Motord, Temp2;                                  /* 临时变量 */
  static float    Voltage_All;                                                            /* 电压采样相关变量 */
  static unsigned char  Position_Count, Voltage_Count;                                          /* 位置控制分频用的变量 */
  sei();                                                                                          /* 全局中断开启 */
  PS2_Velocity = 3.5 * (abs( LY ) + abs( LX ) + abs( RX ) );                                      /* 取摇杆数据为目标速度与摇杆力度和角度相关 */
  Get_RC();                                                                                       /* 遥控函数 */
  if ( Flag_Way == 1 )                                                                            /* 蓝牙遥控赋值 */
  {
    Multiple = 2 * Bluetooth_Velocity / 80;                                                 /* 目标速度大，加快启动加速过程 */
    if ( Bluetooth_Velocity <= 480 )
      Multiple = 8.5;                                                                 /* 目标速度小，延长启动加速过程 */
    Target_A  = VelocityA;                                                            /* 目标速度赋值给PI控制器 */
    Target_B  = VelocityB;                                                            /* 目标速度赋值给PI控制器 */
    Target_C  = VelocityC;                                                            /* 目标速度赋值给PI控制器 */
    Target_D  = VelocityD;                                                            /* 目标速度赋值给PI控制器 */
    Angular_Speed_Control();                                                                /* APP模式陀螺仪调整方向函数 */
  }else if ( Flag_Way == 0 )                                                                      /* 手柄遥控赋值 */
  {
    Multiple = 2 * abs( PS2_Velocity ) / 100;                                               /* 目标速度大，加快启动加速过程 */
    if ( abs( PS2_Velocity ) <= 480 )
      Multiple = 8.5;                                                                 /* 目标速度小，延长启动加速过程 */
    Target_A  = Velocity__A;                                                          /* 目标速度赋值给PI控制器 */
    Target_B  = Velocity__B;                                                          /* 目标速度赋值给PI控制器 */
    Target_C  = Velocity__C;                                                          /* 目标速度赋值给PI控制器 */
    Target_D  = Velocity__D;                                                          /* 目标速度赋值给PI控制器 */
  }
  Motora  = Incremental_PI_A( Target_A, Velocity_A ) + PWMA;                                      /* ===速度PI控制器 */
  Motorb  = Incremental_PI_B( Target_B, Velocity_B ) + PWMB;                                      /* ===速度PI控制器 */
  Motorc  = Incremental_PI_C( Target_C, Velocity_C ) + PWMC;                                      /* ===速度PI控制器 */
  Motord  = Incremental_PI_D( Target_D, Velocity_D ) + PWMD;                                      /* ===速度PI控制器 */
  if ( Turn_Off() == 0 )
    Set_PWM( Motora, Motorb, Motorc, Motord );                                              /* 如果不存在异常，使能电机 */
  Temp2 = analogRead( A0 );                                                                       /* 采集一下电池电压 */
  Voltage_Count++;                                                                                /* 平均值计数器 */
  Voltage_All += Temp2;                                                                           /* 多次采样累积 */
  if ( Voltage_Count == 200 )
    Battery_Voltage = Voltage_All * 0.05371 / 2, Voltage_All = 0, Voltage_Count = 0;        /* 求平均值 */
  Gryo  = -gz / 131;                                                                            /* z轴角速度计算 */
  Angle += Gryo / 100;                                                                          /* Z轴角度计算 */
}


void event()
{
  static int    i, j;
  int     Voltage_Temp;
  static unsigned char  Flag_PID, Receive[10], ReceiveInstruction;
  static float    Data;
  while ( Serial3.available() )
  {
    ReceiveInstruction = Serial3.read();
    if ( ReceiveInstruction == 'J' )
      Transformation = 0;                                                     /* 字符J对应APP摇杆遥控模式标志 */
    if ( ReceiveInstruction == 'K' )
      Transformation = 1;                                                     /* 字符K对应APP按键遥控模式标志 */
    if ( ReceiveInstruction >= 0x41 && ReceiveInstruction <= 0x48 )
      Flag_Direction = ReceiveInstruction - 0x40;                             /* 接收APP指令，ASCLL码对应字符A~H */
    else if ( ReceiveInstruction < 10 )
      Flag_Direction = ReceiveInstruction;                                    /* 字符A~H对应转换为1到8 */
    else if ( ReceiveInstruction == 0X5A )
      Flag_Direction = 0;                                                     /* 字符Z对应转换为0 */
    else Flag_Direction = 9;                                                        /* 非操控指令统一返回参数9，防止蓝牙连接断开导致小车不受控制 */
    if ( ReceiveInstruction == 0x7B )
      Flag_PID = 1;                                                           /* 参数指令起始位 */
    if ( ReceiveInstruction == 0x7D )
      Flag_PID = 2;                                                           /* 参数指令停止位 */
    if ( Flag_PID == 1 )
      Receive[i] = ReceiveInstruction, i++;
    else if ( Flag_PID == 2 )                                                       /* 执行指令 */
    {
      if ( Receive[3] == 0x50 )
      {
        Voltage_Temp = (Battery_Voltage - 1100);                        /* 根据APP的协议对电池电压变量进行处理 */
        if ( Voltage_Temp > 100 )
          Voltage_Temp = 100;
        if ( Voltage_Temp < 0 )
          Voltage_Temp = 0;
        Serial3.print( "{C" );
        Serial3.print( Bluetooth_Velocity );                            /* 速度参数对应APP参数0 */
        Serial3.print( ":" );
        Serial3.print( (int) (Velocity_KP * 100) );                     /* KP参数对应APP参数1 */
        Serial3.print( ":" );
        Serial3.print( (int) (Velocity_KI * 1000) );                    /* KI参数对应APP参数2 */
        Serial3.print( "}$" );
        Serial3.print( Voltage_Temp );
      }
      /*   else  if (Receive[3] == 0x57)    Flash_Send = 1; //掉电保存参数 */
      else if ( Receive[1] != 0x23 )                                          /* 更新PID参数 */
      {
        for ( j = i; j >= 4; j-- )
        {
          Data += (Receive[j - 1] - 48) * pow( 10, i - j );       /* 通讯协议 */
        }
        switch ( Receive[1] )
        {
        case 0x30:  Bluetooth_Velocity  = Data; break;                  /* 目标速度，对应APP参数0 */
        case 0x31:  Velocity_KP   = Data / 100; break;            /* KP参数，对应APP参数1 */
        case 0x32:  Velocity_KI   = Data / 1000; break;           /* KP参数，对应APP参数2 */
        case 0x33:  break;
        case 0x34:  break;                                              /* 9个通道 */
        case 0x35:  break;
        case 0x36:  break;
        case 0x37:  break;
        case 0x38:  break;
        }
      }
      Flag_PID = 0;       i = 0;       j = 0;      Data = 0;                  /* 相关标志位清零 */
    }
  }
}


/***************函数功能：遥控**********/
void Get_RC()
{
  if ( Transformation == 0 )                                                                      /* 进入APP摇杆模式标志 */
  {
    if ( Flag_Direction == 0 )
    {
      VelocityA = 0; VelocityB = 0; VelocityC = 0; VelocityD = 0;
    }                                                                                       /* 停止，速度为0 */
    else if ( Flag_Direction == 1 )                                                         /* 前进 */
    {
      VelocityA = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityB = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityC = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityD = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
    }else if ( Flag_Direction == 5 )                                                        /* 后退 */
    {
      VelocityA = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityB = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityC = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityD = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
    }else if ( Flag_Direction == 7 )                                                        /* 左横移 */
    {
      VelocityA = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityB = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityC = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityD = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
    }else if ( Flag_Direction == 3 )                                                        /* 右横移 */
    {
      VelocityA = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityB = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityC = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityD = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
    }else if ( Flag_Direction == 8 )                                                        /* 左上移动指令 */
    {
      VelocityA = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityB = 0;
      VelocityC = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityD = 0;
    }else if ( Flag_Direction == 2 )                                                        /* 右上移动指令 */
    {
      VelocityA = 0;
      VelocityB = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityC = 0;
      VelocityD = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
    }else if ( Flag_Direction == 6 )                                                        /* 左下移动指令 */
    {
      VelocityA = 0;
      VelocityB = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityC = 0;
      VelocityD = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
    }else if ( Flag_Direction == 4 )                                                        /* 右下移动指令 */
    {
      VelocityA = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityB = 0;
      VelocityC = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityD = 0;
    }else if ( Flag_Direction == 9 )
    {
      VelocityA = 0; VelocityB = 0; VelocityC = 0; VelocityD = 0;
    }
  }
  if ( Transformation == 1 )                                                                      /* 进入APP按键模式标志 */
  {
    if ( Flag_Direction == 0 )
    {
      VelocityA = 0; VelocityB = 0; VelocityC = 0; VelocityD = 0;
    }else if ( Flag_Direction == 1 )                                                        /* 前进指令 */
    {
      VelocityA = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityB = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityC = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityD = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
    }else if ( Flag_Direction == 5 )                                                        /* 后退指令 */
    {
      VelocityA = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityB = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityC = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityD = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
    }else if ( Flag_Direction == 7 )                                                        /* 右自转指令 */
    {
      VelocityA = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityB = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityC = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityD = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
    }else if ( Flag_Direction == 3 )                                                        /* 左自转指令 */
    {
      VelocityA = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityB = -Bluetooth_Velocity;                                          /* 赋予设定的目标速度，轮子后转 */
      VelocityC = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
      VelocityD = Bluetooth_Velocity;                                           /* 赋予设定的目标速度，轮子前转 */
    }else if ( Flag_Direction == 9 )
    {
      VelocityA = 0; VelocityB = 0; VelocityC = 0; VelocityD = 0;
    }
  }
  if ( Flag_Way == 0 )                                                                            /* PS2控制 */
  {
    Velocity__A = 3.5 * (-LX - LY + RX * 1) + Gryo * (abs( LX ) + abs( LY ) ) / 80;     /* 手柄运动控制函数，3.5为设定摇杆速度倍率，可根据需要调整 */
    Velocity__B = 3.5 * (+LX - LY + RX * 1) + Gryo * (abs( LX ) + abs( LY ) ) / 80;     /* 手柄运动控制函数，3.5为设定摇杆速度倍率，可根据需要调整 */
    Velocity__C = -(3.5 * (+LX + LY + RX * 1) - Gryo * (abs( LX ) + abs( LY ) ) / 80);  /* 手柄运动控制函数，3.5为设定摇杆速度倍率，可根据需要调整 */
    Velocity__D = -(3.5 * (-LX + LY + RX * 1) - Gryo * (abs( LX ) + abs( LY ) ) / 80);  /* 手柄运动控制函数，3.5为设定摇杆速度倍率，可根据需要调整 */
    if ( PS2_LY > 250 && PS2_RX > 250 && PS2_LX > 250 )
    {
      Velocity__A = 0;
      Velocity__B = 0;
      Velocity__C = 0;
      Velocity__D = 0;
    }                                                                                       /* 非断电状态直接拔手柄无线模块速度置零，防止小车不受控制 */
  }
}


/********函数功能：OLED显示*********/
void OLED()
{
  oled.clear();
  Velocity_A  = -Velocity_1 / 1.5;    Velocity_1 = 0; /* 读取编码器数据并根据实际接线做调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。 */
  Velocity_B  = Velocity_2 / 1.5;    Velocity_2 = 0;  /* 读取编码器数据并根据实际接线做调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。 */
  Velocity_C  = -Velocity_3 / 1.5;    Velocity_3 = 0; /* 读取编码器数据并根据实际接线做调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。 */
  Velocity_D  = -Velocity_4 / 1.5;    Velocity_4 = 0; /* 读取编码器数据并根据实际接线做调整、然后清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。 */
  oled.drawstring( 00, 0, "VOLTAGEC:" );
  oled.drawstring( 71, 0, "." );
  oled.drawstring( 93, 0, "V" );
  OLED_ShowNumber( 58, 0, Battery_Voltage / 100, 2 );     /* 显示电池电量 */
  OLED_ShowNumber( 81, 0, Battery_Voltage % 100, 2 );     /* 显示电池电量 */
  if ( Velocity_A < 0 )
  {
    oled.drawstring( 00, 01, "-:" );
    OLED_ShowNumber( 06, 01, -Velocity_A, 5 );      /* 显示轮子实时速度 */
  }else if ( Velocity_A > 0 )
  {
    oled.drawstring( 00, 01, "+:" );
    OLED_ShowNumber( 06, 01, Velocity_A, 5 );       /* 显示轮子实时速度 */
  }
  if ( Velocity_B < 0 )
  {
    oled.drawstring( 70, 01, "-:" );
    OLED_ShowNumber( 76, 01, -Velocity_B, 5 );      /* 显示轮子实时速度 */
  }else if ( Velocity_B > 0 )
  {
    oled.drawstring( 70, 01, "+:" );
    OLED_ShowNumber( 76, 01, Velocity_B, 5 );       /* 显示轮子实时速度 */
  }
  if ( Velocity_C < 0 )
  {
    oled.drawstring( 00, 02, "-:" );
    OLED_ShowNumber( 06, 02, -Velocity_C, 5 );      /* 显示轮子实时速度 */
  }else if ( Velocity_C > 0 )
  {
    oled.drawstring( 00, 02, "+:" );
    OLED_ShowNumber( 06, 02, Velocity_C, 5 );       /* 显示轮子实时速度 */
  }
  if ( Velocity_D < 0 )
  {
    oled.drawstring( 70, 02, "-:" );
    OLED_ShowNumber( 76, 02, -Velocity_D, 5 );      /* 显示轮子实时速度 */
  }else if ( Velocity_D > 0 )
  {
    oled.drawstring( 70, 02, "+:" );
    OLED_ShowNumber( 76, 02, Velocity_D, 5 );       /* 显示轮子实时速度 */
  }
  if ( Flag_Way == 1 )
  {
    oled.drawstring( 00, 3, "Target-Velocity:" );   /* 显示APP设定的目标速度 */
    OLED_ShowNumber( 100, 3, Bluetooth_Velocity, 4 );
  }                                                       /* 显示APP设定的目标速度 */
  if ( Flag_Way == 0 )
  {
    oled.drawstring( 00, 3, "Target-Velocity:" );   /* 显示手柄设定的目标速度 */
    OLED_ShowNumber( 100, 3, abs( PS2_Velocity ), 4 );
  }                                                       /* 显示手柄设定的目标速度 */
  oled.drawstring( 00, 04, "Mode-select:" );
  if ( Flag_Way == 1 )
  {
    oled.drawstring( 95, 04, "APP" );
  }
  if ( Flag_Way == 0 )
  {
    oled.drawstring( 95, 04, "PS2" );
  }
  oled.drawstring( 00, 05, "LY:" ),                       /* 手柄数据显示 */
  OLED_ShowNumber( 18, 05, abs( PS2_LY ), 3 );            /* 手柄数据显示 */
  oled.drawstring( 42, 05, "LX:" ),                       /* 手柄数据显示 */
  OLED_ShowNumber( 60, 05, abs( PS2_LX ), 3 );            /* 手柄数据显示 */
  oled.drawstring( 84, 05, "RX:" ),                       /* 手柄数据显示 */
  OLED_ShowNumber( 100, 05, abs( PS2_RX ), 3 );           /* 手柄数据显示 */

  if ( Angle < 0 )
  {
    oled.drawstring( 00, 06, "AngleZ:-" );          /* MPU Z轴翻转角度数据显示 */
    OLED_ShowNumber( 49, 06, -Angle, 3 );           /* MPU Z轴翻转角度数据显示 */
  }else if ( Angle >= 0 )
  {
    oled.drawstring( 00, 06, "AngleZ:+" );          /* MPU Z轴翻转角度数据显示 */
    OLED_ShowNumber( 49, 06, Angle, 3 );            /* MPU Z轴翻转角度数据显示 */
  }
  if ( Gryo < 0 )
  {
    oled.drawstring( 00, 07, "Gryo :-" );           /* MPU z轴角速度数据显示 */
    OLED_ShowNumber( 46, 07, -Gryo, 3 );            /* MPU z轴角速度数据显示 */
    oled.drawstring( 66, 07, "." );                 /* MPU z轴角速度数据显示 */
    OLED_ShowNumber( 76, 07, -100 * Gryo, 2 );      /* MUP z轴角速度数据显示 */
  }else if ( Gryo >= 0 )
  {
    oled.drawstring( 00, 07, "Gryo :+" );           /* MPU z轴角速度数据显示 */
    OLED_ShowNumber( 46, 07, Gryo, 3 );             /* MPU z轴角速度数据显示 */
    oled.drawstring( 66, 07, "." );                 /* MPU z轴角速度数据显示 */
    OLED_ShowNumber( 76, 07, 100 * Gryo, 2 );       /* MPUz轴角速度数据显示 */
  }
  oled.display();
}


/***********函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家************/
void setup()
{
  char  error;
  int fff = 1;
  oled.ssd1306_init( SSD1306_SWITCHCAPVCC );      /* 显示器初始化 */
  oled.clear();                                   /* clears the screen and buffer */
  TCCR1B  = (TCCR1B & 0xF8) | fff;                /* 调整计数器分频，频率调高至31.374KHZ */
  TCCR3B  = (TCCR3B & 0xF8) | fff;                /* 调整计数器分频，频率调高至31.374KHZ */
  TCCR4B  = (TCCR4B & 0xF8) | fff;                /* 调整计数器分频，频率调高至31.374KHZ */
  TCCR5B  = (TCCR5B & 0xF8) | fff;                /* 调整计数器分频，频率调高至31.374KHZ */
  pinMode( AIN1, OUTPUT );
  pinMode( BIN1, OUTPUT );
  pinMode( CIN1, OUTPUT );
  pinMode( DIN1, OUTPUT );
  pinMode( AIN2, OUTPUT );
  pinMode( BIN2, OUTPUT );
  pinMode( CIN2, OUTPUT );
  pinMode( DIN2, OUTPUT );
  pinMode( ENCODER_A, INPUT );
  pinMode( ENCODER_B, INPUT );
  pinMode( ENCODER_C, INPUT );
  pinMode( ENCODER_D, INPUT );
  pinMode( DIRECTION_A, INPUT );
  pinMode( DIRECTION_B, INPUT );
  pinMode( DIRECTION_C, INPUT );
  pinMode( DIRECTION_D, INPUT );
  pinMode( A0, INPUT );
  delay( 300 );                                                                           /* 延时等待初始化完成 */
  FlexiTimer2::set( 10, control );                                                        /* 10毫秒定时中断函数 */
  FlexiTimer2::start();                                                                   /* 中断使能 */
  attachInterrupt( 0, READ_ENCODER_A, CHANGE );                                           /* 开启外部中断 编码器接口A */
  attachInterrupt( 1, READ_ENCODER_B, CHANGE );                                           /* 开启外部中断 编码器接口B */
  attachInterrupt( 5, READ_ENCODER_C, CHANGE );                                           /* 开启外部中断 编码器接口C */
  attachInterrupt( 4, READ_ENCODER_D, CHANGE );                                           /* 开启外部中断 编码器接口D */
  /* Serial3.begin(9600);//开启串口 */
  Serial.begin( 9600 );                                                                   /* 开启串口 */
  error = ps2x.config_gamepad( PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false );        /* PS2无线手柄控制初始化 */
  while ( select() )
  {
  }                                                                                       /* 检测是否插入手柄，进入模式选择 */
  Wire.begin();                                                                           /* 加入 IIC 总线 */
  delay( 1500 );                                                                          /* 延时等待初始化完成 */
  Mpu6050.initialize();                                                                   /* 初始化MPU6050 */
  delay( 50 );
}


/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_A()
{
  if ( digitalRead( DIRECTION_A ) == LOW )        /* 如果是下降沿触发的中断 */
  {
    if ( digitalRead( ENCODER_A ) == LOW )
      Velocity_1--;                   /* 根据另外一相电平判定方向 */
    else Velocity_1++;
  }else  {                                        /* 如果是上升沿触发的中断 */
    if ( digitalRead( ENCODER_A ) == LOW )
      Velocity_1++;                   /* 根据另外一相电平判定方向 */
    else Velocity_1--;
  }
}


/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_B()
{
  if ( digitalRead( DIRECTION_B ) == LOW )        /* 如果是下降沿触发的中断 */
  {
    if ( digitalRead( ENCODER_B ) == LOW )
      Velocity_2++;                   /* 根据另外一相电平判定方向 */
    else Velocity_2--;
  }else  {                                        /* 如果是上升沿触发的中断 */
    if ( digitalRead( ENCODER_B ) == LOW )
      Velocity_2--;                   /* 根据另外一相电平判定方向 */
    else Velocity_2++;
  }
}


/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_C()
{
  if ( digitalRead( DIRECTION_C ) == LOW )        /* 如果是下降沿触发的中断 */
  {
    if ( digitalRead( ENCODER_C ) == LOW )
      Velocity_3++;                   /* 根据另外一相电平判定方向 */
    else Velocity_3--;
  }else  {                                        /* 如果是上升沿触发的中断 */
    if ( digitalRead( ENCODER_C ) == LOW )
      Velocity_3--;                   /* 根据另外一相电平判定方向 */
    else Velocity_3++;
  }
}


/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_D()
{
  if ( digitalRead( DIRECTION_D ) == LOW )        /* 如果是下降沿触发的中断 */
  {
    if ( digitalRead( ENCODER_D ) == LOW )
      Velocity_4++;                   /* 根据另外一相电平判定方向 */
    else Velocity_4--;
  }else  {                                        /* 如果是上升沿触发的中断 */
    if ( digitalRead( ENCODER_D ) == LOW )
      Velocity_4--;                   /* 根据另外一相电平判定方向 */
    else Velocity_4++;
  }
}


void event_pi()
{
  static int    i, j;
  int     Voltage_Temp;
  static unsigned char  Flag_PID, Receive[10], ReceiveInstruction;
  static float    Data;
  while ( Serial.available() )
  {
    ReceiveInstruction = Serial.read();

    if ( ReceiveInstruction == 'J' )
      Transformation = 0;                                                     /* 字符J对应APP摇杆遥控模式标志 */

    if ( ReceiveInstruction == 'K' )
      Transformation = 1;                                                     /* 字符K对应APP按键遥控模式标志 */

    if ( ReceiveInstruction >= 0x41 && ReceiveInstruction <= 0x48 )
      Flag_Direction = ReceiveInstruction - 0x40;                             /* 接收APP指令，ASCLL码对应字符A~H */
    else if ( ReceiveInstruction < 10 )
      Flag_Direction = ReceiveInstruction;                                    /* 字符A~H对应转换为1到8 */
    else if ( ReceiveInstruction == 0X5A )
      Flag_Direction = 0;                                                     /* 字符Z对应转换为0 */
    else Flag_Direction = 9;                                                        /* 非操控指令统一返回参数9，防止蓝牙连接断开导致小车不受控制 */

    if ( ReceiveInstruction == 0x7B )
      Flag_PID = 1;                                                           /* 参数指令起始位 */

    if ( ReceiveInstruction == 0x7D )
      Flag_PID = 2;                                                           /* 参数指令停止位 */

    Serial.print( "Flag_PID:" );
    Serial.println( Flag_PID );

    if ( Flag_PID == 1 )
      Receive[i] = ReceiveInstruction, i++;
    else if ( Flag_PID == 2 )                                                       /* 执行指令 */
    {
      Serial.println( "Branch 1" );
      if ( Receive[3] == 0x50 )
      {
        Serial.print( "Branch 2: " );
        Voltage_Temp = (Battery_Voltage - 1100);                        /* 根据APP的协议对电池电压变量进行处理 */
        if ( Voltage_Temp > 100 )
          Voltage_Temp = 100;
        if ( Voltage_Temp < 0 )
          Voltage_Temp = 0;
        //Serial.println( Voltage_Temp );
        Serial.print( "{C" );
        Serial.print( Bluetooth_Velocity );                             /* 速度参数对应APP参数0 */
        Serial.print( ":" );
        Serial.print( (int) (Velocity_KP * 100) );                      /* KP参数对应APP参数1 */
        Serial.print( ":" );
        Serial.print( (int) (Velocity_KI * 1000) );                     /* KI参数对应APP参数2 */
        Serial.print( "}$" );
        Serial.print( Voltage_Temp );
        Serial.println();
      }
      /*   else  if (Receive[3] == 0x57)    Flash_Send = 1; //掉电保存参数 */
      else if ( Receive[1] != 0x23 )                                          /* 更新PID参数 */
      {
        Serial.println( "Branch 3" );
        for ( j = i; j >= 4; j-- )
        {
          Data += (Receive[j - 1] - 48) * pow( 10, i - j );       /* 通讯协议 */
        }
        switch ( Receive[1] )
        {
        case 0x30:  Bluetooth_Velocity  = Data; break;                  /* 目标速度，对应APP参数0 */
        case 0x31:  Velocity_KP   = Data / 100; break;            /* KP参数，对应APP参数1 */
        case 0x32:  Velocity_KI   = Data / 1000; break;           /* KP参数，对应APP参数2 */
        case 0x33:  break;
        case 0x34:  break;                                              /* 9个通道 */
        case 0x35:  break;
        case 0x36:  break;
        case 0x37:  break;
        case 0x38:  break;
        }
      }


      Flag_PID = 0;       i = 0;       j = 0;      Data = 0; /* 相关标志位清零 */
    }

    Serial.print( "Flag_Way:" );
    Serial.print( Flag_Way );    
    Serial.print( ", Battery_Voltage:" );
    Serial.print( Battery_Voltage );
    Serial.print( ", Transformation:" );
    Serial.print( Transformation );
    Serial.print( ", Flag_Direction:" );
    Serial.print( Flag_Direction );
    Serial.print( ", Bluetooth_Velocity:" );
    Serial.print( Bluetooth_Velocity );
    Serial.print( ", Velocity_KP:" );
    Serial.print( Velocity_KP );
    Serial.print( ", Velocity_KI:" );
    Serial.print( Velocity_KI );
    Serial.println();
    Serial.print( "Receive:" );
    Serial.print( Receive[1] );
    Serial.print( ", " );
    Serial.print( Receive[2] );
    Serial.print( ", " );
    Serial.print( Receive[3] );
    Serial.print( ", " );
    Serial.print( Receive[4] );
    Serial.print( ", " );
    Serial.print( Receive[5] );
    Serial.print( ", " );
    Serial.print( Receive[6] );
    Serial.print( ", " );
    Serial.print( Receive[7] );
    Serial.print( ", " );
    Serial.print( Receive[8] );
    Serial.print( ", " );
    Serial.print( Receive[9] );
    Serial.println();
  }
}


/* **********主函数*************** // */
void loop()
{
  event_pi();                                             /* 默认APP模式，蓝牙控制 */
  OLED();                                                 /* 显示器显示参数 */
  Mpu6050.getMotion6( &ax, &ay, &az, &gx, &gy, &gz );     /* 获取MPU6050陀螺仪和加速度计的数据 */
  if ( Plug == 0 )
    ps2x.read_gamepad( false, 0 );                  /* read controller and set large motor to spin at 'vibrate' speed */
  PS2_LY  = ps2x.Analog( PSS_LY );                        /* 手柄左摇杆Y轴数据读取 */
  PS2_RX  = ps2x.Analog( PSS_RX );                        /* 手柄右摇杆X轴数据读取 */
  PS2_LX  = ps2x.Analog( PSS_LX );                        /* 手柄左摇杆X轴数据读取 */
  PS2_RY  = ps2x.Analog( PSS_RY );                        /* 手柄右摇杆Y轴数据读取 */
  LY  = PS2_LY - 128;                                 /* 计算偏差 */
  LX  = PS2_LX - 128;                                 /* 计算偏差 */
  RX  = PS2_RX - 128;                                 /* 计算偏差 */
  if ( LY > -Yuzhi && LY < Yuzhi )
    LY = 0;                                         /* 小角度设为死区 防止抖动出现异常 */
  if ( LX > -Yuzhi && LX < Yuzhi )
    LX = 0;                                         /* 小角度设为死区 防止抖动出现异常 */
  if ( RX > -Yuzhi && RX < Yuzhi )
    RX = 0;                                         /* 小角度设为死区 防止抖动出现异常 */
  if ( PS2_RY > 133 )
  {
    Flag_Way = 0;
  }                                                       /* 手柄右摇杆下拨，进入手柄模式 */
  if ( PS2_RY < 120 )
  {
    Flag_Way = 1;
  }                                                       /* 手柄右摇杆上拨，进入APP模式 */
  if ( PS2_RY == 255 && PS2_RX == 255 && PS2_LY == 255 && PS2_LX == 255 )
  {
    Flag_Way = 1; Plug = 1;
  }                                                       /* 拔掉手柄接收模块，进入APP模式 */
  /* if(Serial3.available()<=0){Plug=0;}//APP停止指令，允许进入手柄模式 */
  if ( Serial.available() <= 0 )
  {
    Plug = 0;
  }                                                       /* APP停止指令，允许进入手柄模式 */
}


/* **************************启动时检测是否插入手柄接收模块********** // */
unsigned char  select( void )
{
  static unsigned char flag = 1;                          /* 循环标志 */
  ps2x.read_gamepad( false, 0 );                          /* read controller and set large motor to spin at 'vibrate' speed */
  PS2_LY  = ps2x.Analog( PSS_LY );                        /* 手柄左摇杆Y轴数据读取 */
  PS2_RX  = ps2x.Analog( PSS_RX );                        /* 手柄右摇杆X轴数据读取 */
  PS2_LX  = ps2x.Analog( PSS_LX );                        /* 手柄左摇杆X轴数据读取 */
  PS2_RY  = ps2x.Analog( PSS_RY );                        /* 手柄右摇杆Y轴数据读取 */
  if ( PS2_RY == 255 && PS2_RX == 255 && PS2_LY == 255 && PS2_LX == 255 )
  {
    Flag_Way = 1; Plug = 1; flag = 0;
  }                                                       /*不插手柄接收模块，禁止进入手柄模式，直接进入APP模式 */
  /* （不插手柄但是又运行函数ps2x.read_gamepad(false, 0);的话会对APP模式造成干扰） */
  else flag = 0;                                          /* 跳出循环标志 */
  return(flag);
}
