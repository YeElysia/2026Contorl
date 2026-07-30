#ifndef TTL_STEPPER_H
#define TTL_STEPPER_H
#include <Arduino.h>

#define ABS(x) ((x) > 0 ? (x) : -(x))
#define MAXSPEED 100000
#define MAXACCELERATION 100000
#define BAUDRATE 115200

// // CRC-8校验表（已提供）
// const unsigned char crc8Table[256] = {
// 0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
// 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
// 0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
// 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
// 0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
// 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
// 0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
// 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
// 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
// 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
// 0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
// 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
// 0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
// 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
// 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
// 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
// 0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
// 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
// 0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
// 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
// 0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
// 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
// 0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
// 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
// 0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
// 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
// 0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
// 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
// 0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
// 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
// 0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
// 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
// };
// unsigned char calculateCRC8(unsigned char *p, unsigned char len);

typedef enum
{
   S_VER = 0,    /* 读取固件版本和对应的硬件版本 */
   S_RL = 1,     /* 读取读取相电阻和相电感 */
   S_PID = 2,    /* 读取PID参数 */
   S_VBUS = 3,   /* 读取总线电压 */
   S_CPHA = 5,   /* 读取相电流 */
   S_ENCL = 7,   /* 读取经过线性化校准后的编码器值 */
   S_TPOS = 8,   /* 读取电机目标位置角度 */
   S_VEL = 9,    /* 读取电机实时转速 */
   S_CPOS = 10,  /* 读取电机实时位置角度 */
   S_PERR = 11,  /* 读取电机位置误差角度 */
   S_FLAG = 13,  /* 读取使能/到位/堵转状态标志位 */
   S_Conf = 14,  /* 读取驱动参数 */
   S_State = 15, /* 读取系统状态参数 */
   S_ORG = 16,   /* 读取正在回零/回零失败状态标志位 */
} SysParams_t;

// 通讯协议
class TTL_Protocol
{
public:
   uint32_t baudrate; // 串口通信的波特率
   // HardwareSerial *serial; //串口
   Stream *serial;
   TTL_Protocol(HardwareSerial *serial, uint32_t baudrate);
   void init(HardwareSerial *serial, uint32_t baudrate); // 初始化
   void emptyCache();
   void synrun();                 // 触发多机同步运动
   uint8_t error_address_catch(); // 判断命令返回，仅确认是否发出错误命令，返回错误命令步进地址
   /*以下是商家提供支持arduino 步进串口通讯函数 */

   void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);                                                                                                                                     // 将当前位置清零
   void Emm_V5_Reset_Clog_Pro(uint8_t addr);                                                                                                                                           // 解除堵转保护
   void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);                                                                                                                           // 读取参数
   void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);                                                                                                            // 发送命令修改开环/闭环控制模式
   void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);                                                                                                                         // 电机使能控制
   void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);                                                                                            // 速度模式控制
   void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);                                                                    // 位置模式控制
   void Emm_V5_Stop_Now(uint8_t addr, bool snF);                                                                                                                                       // 让电机立即停止运动
   void Emm_V5_Synchronous_motion(uint8_t addr);                                                                                                                                       // 触发多机同步开始运动
   void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);                                                                                                                                   // 设置单圈回零的零点位置
   void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); // 修改回零参数
   void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);                                                                                                          // 发送命令触发回零
   void Emm_V5_Origin_Interrupt(uint8_t addr);                                                                                                                                         // 强制中断并退出回零
   void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);                                                                                                                         // 返回数据接收函数
private:
};

class TTL_Stepper
{
public:
   TTL_Protocol *protocol; // 舵机串口通信协议
   uint8_t addr;
   float convert_K;        // 旋转转位移系数转一圈移动多少0.1毫米
   uint16_t substep;       // 细分步数
   uint32_t baudrate;      // 串口通信的波特率
   uint16_t Speed;         // 速度(0~5000RPM)

   uint32_t ptime;         // 运动开始时间
   float previousPosition; // 运动开始的位置0.1mm
   float currentPosition;  // 当前绝对位置0.1mm
   float delta;            // 运动需要的位移,用以更新实时位置
   double p_speed;         //运动开始速度rpm
   double currentSpeed;    //当前速度rpm
   uint8_t Acceleration;   // 加速度(0~255)
   uint8_t Func_Code;      // 命令码
   uint8_t recDate[256];       // 返回值接收区
   uint8_t Date_Index;     // 返回值接收区指针
   bool Ask_State;         // 查询状态标志，已经发出查询命令True,未False.
   bool CW;                // cw为正方向0，反方向1
   bool conditionNotMet;   // 条件不满足
   bool command_check;     // 发送的上一个指令是否发送了错误指令，发送了错误指令0，发送的正确指令1
   bool enstate;           // 使能状态，True为打开电机，False为关闭
   bool onPos_state;       // 到位标志位，True到位，False无
   bool locked_state;      // 堵转标志位，True堵转,False无
   bool loPro_state;       // 堵转保护标志位，True堵转保护，False无
   bool snF;               // 多机同步标志，Ture
   // 构造器函数
   TTL_Stepper();
   TTL_Stepper(uint8_t addr, TTL_Protocol *protocol);
   void state_update(); // 状态更新（使能状态、到位状态、堵转、堵转保护）
   void update_state(); // 状态更新（使能状态、到位状态、堵转、堵转保护）更好
   void init();         // 初始化
   void init(uint8_t addr, TTL_Protocol *protocol);
   void set(uint16_t Speed, uint8_t Acceleration, bool CW, float convert_K, uint16_t substep);
   // 命令在有限次数内获得正确应答返回true，否则返回false。
   bool runToNewPosition(float x);
   bool runToNewPosition(float x, uint16_t vel, uint8_t acc);
   bool setAngle(float angle);   // 旋转
   bool setAngle(float angle, uint16_t vel, uint8_t acc);
   float Calculate_DeltaPos();   // 计算当前位移,更新速度
   float Calculate_CurrentPos(); // 计算当前绝对位置
   void update_CurrentPos();     // 更新当前位置；
   void runToOrigin();           // 回零
   void wait();
   bool enable();                  // 电机使能
   int Emm_V5_Origin_Read_state(); // 读取原点回零状态
   bool wrong_command_catch();     // 指令正确错误判断
   void Multi_syn_RunToOrigin();   // 多机同步回零
   void recDate_Clear();           // 清空接收区何查询状态标志
private:
   static constexpr uint8_t COMMAND_RETRY_COUNT = 3;
   bool sendPositionCommand(
      float target,
      uint16_t velocity,
      uint8_t acceleration,
      bool angle);
};
class ARM_WHOLE_STEPPER
{
public:
   TTL_Stepper *Stepper_SHUZHI;
   TTL_Stepper *Stepper_SHUIPIN;
   TTL_Stepper *aim_Stepper; // 获得的返回值指向的步进
   uint8_t *recData;
   uint8_t Data_Index;
   bool enable;
   TTL_Protocol *protocol; // 舵机串口通信协议
   ARM_WHOLE_STEPPER(TTL_Stepper *Stepper_SHUZHI, TTL_Stepper *Stepper_SHUIPIN, TTL_Protocol *protocol);
   void Stepper_State_Update();
   void Stepper_onPos_Update();
};
#endif
