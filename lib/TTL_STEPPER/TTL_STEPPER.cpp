#include "TTL_STEPPER.h"
// CRC_8校验
//  unsigned char calculateCRC8(unsigned char *p, unsigned char len)
//  {
//      unsigned char i = 0, crc8 = p[0];

//     for(i = 1; i < len; i++)
//     {
//         crc8 = crc8Table[crc8 ^ (*(p + i))];
//     }

//     return crc8;
// }

/**
 * @brief    创建空步进通讯对象
 * @param
 * @param
 * @retval
 */
TTL_Stepper::TTL_Stepper()
{
}

/**
 * @brief    创建步进通讯对象
 * @param    addr :电机地址
 * @param    protocol:串口通讯协议
 * @retval
 */
TTL_Stepper::TTL_Stepper(uint8_t addr, TTL_Protocol *protocol)
{
    this->addr = addr;
    this->Speed = 0;        // 默认设置为零，防止损坏
    this->Acceleration = 1; // 默认设置为1；
    this->CW = 1;
    this->currentPosition = 0;
    this->previousPosition = 0;
    this->protocol = protocol;
}

/**
 * @brief    设置电机运动速度、加速度、正方向、位移转换系数、细分步数
 * @param    Speed :速度
 * @param    Acceleration :加速度
 * @param    CW :正方向与CW方向一致为0,否则为1
 * @param    convert_K :转换系数
 * @param    substep :细分步数
 * @retval
 */
void TTL_Stepper::set(uint16_t Speed, uint8_t Acceleration, bool CW, float convert_K, uint16_t substep)
{
    this->Speed = Speed;               // 默认设置为零，防止损坏
    this->Acceleration = Acceleration; // 默认设置为1；
    this->CW = CW;
    this->convert_K = convert_K;
    this->substep = substep;
}

/**
 * @brief    位置控制，默认不加减速（加速度为0），速度为1000rpm，绝对运动,非多机同步
 * @param    x:绝对位置 单位0.1mm
 * @retval
 */
void TTL_Stepper::runToNewPosition(float x)
{
    // 位置模式：（16细分下发送3200个脉冲电机转一圈），绝对运动
    uint32_t clk = x * substep * 200 * 1.0 / convert_K;
    do
    {
        // currentPosition = previousPosition + Calculate_DeltaPos(); // 更新当前绝对位置和速度
        // previousPosition = currentPosition;                        // 更新运动开始位置
        // p_speed=currentSpeed;                                      // 更新运动开始速度
        protocol->Emm_V5_Pos_Control(this->addr, CW, Speed, Acceleration, clk, 1, 0);
        //ptime = millis(); // 用于后续更新位移量
    } while (!wrong_command_catch());
    //delta = x - previousPosition; // 更新目标位移量
}
/**
 * @brief    位置控制，默认不加减速（加速度为0），速度为1000rpm，绝对运动,非多机同步
 * @param    x:绝对位置 单位0.1mm
 * @retval
 */
void TTL_Stepper::runToNewPosition(float x,uint16_t vel, uint8_t acc)
{
    // 位置模式：速度1000RPM，加速度240，脉冲数（16细分下发送3200个脉冲电机转一圈），绝对运动
    uint32_t volatile clk = x * substep * 200 * 1.0 / convert_K;
    do
    {
        // currentPosition = previousPosition + Calculate_DeltaPos(); // 更新当前绝对位置和速度
        // previousPosition = currentPosition;                        // 更新运动开始位置
        // p_speed=currentSpeed;                                      // 更新运动开始速度
        protocol->Emm_V5_Pos_Control(this->addr, CW, vel, acc, clk, 1, 0);
        //ptime = millis(); // 用于后续更新位移量
    } while (!wrong_command_catch());
    //delta = x - previousPosition; // 更新目标位移量
}
/**
 * @brief    设置角度
 * @param    angle:绝对位置角度 单位0.1度
 * @retval
 */
void TTL_Stepper::setAngle(float angle)
{
    angle=(angle > 3450)? 3450:((angle <0)?0:angle);//限位
    // 位置模式：速度1000RPM，加速度240，脉冲数（16细分下发送3200个脉冲电机转一圈），绝对运动
    uint32_t clk = angle * substep * 200 * 1.0 / convert_K;
    do
    {
        // currentPosition = previousPosition + Calculate_DeltaPos(); // 更新当前绝对位置和速度
        // previousPosition = currentPosition;                        // 更新运动开始位置
        // p_speed=currentSpeed;                                      // 更新运动开始速度
        protocol->Emm_V5_Pos_Control(this->addr, CW, Speed, Acceleration, clk, 1, 0);
        //ptime = millis(); // 用于后续更新位移量
    } while (!wrong_command_catch());
    //delta = x - previousPosition; // 更新目标位移量
}
/**
 * @brief    设置角度
 * @param    angle:绝对位置角度 单位0.1度
 * @retval
 */
void TTL_Stepper::setAngle(float angle,uint16_t vel, uint8_t acc)
{
    angle=(angle > 3450)? 3450:((angle <0)?0:angle);//限位
    // 位置模式：速度1000RPM，加速度240，脉冲数（16细分下发送3200个脉冲电机转一圈），绝对运动
    uint32_t clk = angle * substep * 200 * 1.0 / convert_K;
    do
    {
        // currentPosition = previousPosition + Calculate_DeltaPos(); // 更新当前绝对位置和速度
        // previousPosition = currentPosition;                        // 更新运动开始位置
        // p_speed=currentSpeed;                                      // 更新运动开始速度
        protocol->Emm_V5_Pos_Control(this->addr, CW, vel, acc, clk, 1, 0);
        //ptime = millis(); // 用于后续更新位移量
    } while (!wrong_command_catch());
    //delta = x - previousPosition; // 更新目标位移量
}
/**
 * @brief    命令回零（向上）,采用多圈无限位碰撞回零，非多机同步
 * @param
 * @retval
 */
void TTL_Stepper::runToOrigin()
{
    do
    {
        protocol->Emm_V5_Origin_Trigger_Return(this->addr, 2, 0); // 发送命令触发回零,多圈无限位碰撞回零
    } while (!wrong_command_catch());
    currentPosition = previousPosition = 0;
}

/**
 * @brief    等待到位
 * @param
 * @retval
 */
void TTL_Stepper::wait()
{
    onPos_state = false; // 先认为不到位
    do
    {
        update_state(); // 更新状态
    } while (!onPos_state);
    currentSpeed=p_speed=0;
}
/**
 * @brief    计算当前增加的位置量,并更新速度
 * @param
 * @retval   位移量
 */
float TTL_Stepper::Calculate_DeltaPos()
{
    double total_pulse = delta * substep * 200 * 1.0 / convert_K; // 计算总脉冲值
    uint32_t delta_time = millis() - ptime; // 计算运动时间
    double v_max = Speed * substep * 200.0 / 60000.0; //最大速度 脉冲数每毫秒
    double v0 = p_speed * substep * 200.0 / 60000.0; // 运动开始速度，脉冲数每毫秒
    double s = 0.0;
    double a = 0.0;
    // 根据 delta 确定速度方向
    if (delta < 0) {
        v_max = -v_max;
    }
    if (this->Acceleration == 0) // 直接启动
    {
        // 使用最大速度计算位移
        s = v_max * delta_time;
        // 限制不超过总位移
        if ((v_max > 0 && s > total_pulse) || (v_max < 0 && s < total_pulse)) {
            s = total_pulse;
            currentSpeed = 0;
        } else {
            currentSpeed = v_max * 60000.0 / (substep * 200.0); // 更新当前速度，转换回 RPM
        }
    }
    else
    {
        // 计算实际加速度（脉冲/毫秒²），根据 delta 确定方向
        a = (20.0 / (256.0 - this->Acceleration)) * (substep * 200.0 / 60000.0);
        a = (delta > 0) ? a : -a;
        // 计算从初始速度到目标速度所需的时间
        double t_accel = (v_max - v0) / a;
        // 计算从目标速度减速到 0 所需的距离,减速度与加速度大小相等方向相反
        double s_decel = (v_max * v_max) / (2 * -a);
        // 计算加速段位移
        double s_accel = v0 * t_accel + 0.5 * a * t_accel * t_accel;
        // 计算实际需要的最小位移
        double s_min_total = s_accel + s_decel;
        // 梯形加减速（有匀速段）
        if (fabs(total_pulse) >= fabs(s_min_total))
        {
            // 计算匀速段位移
            double s_const = total_pulse - s_min_total;
            // 避免除零错误
            double t_const = (v_max != 0) ? s_const / v_max : 0;
            double t_decel = (0 - v_max) / -a; // 从 v_max 减速到 0 的时间
            double t_total = t_accel + t_const + t_decel;
            if (delta_time <= t_accel)
            {
                // 加速段：s = v0 * t + 0.5 * a * t²
                s = v0 * delta_time + 0.5 * a * delta_time * delta_time;
                currentSpeed = (v0 + a * delta_time) * 60000.0 / (substep * 200.0); // 更新当前速度，转换回 RPM
            }
            else if (delta_time <= t_accel + t_const)
            {
                // 匀速段：s = s_acc + v_max*(t - t_acc)
                s = s_accel + v_max * (delta_time - t_accel);
                currentSpeed = v_max * 60000.0 / (substep * 200.0); // 更新当前速度，转换回 RPM
            }
            else if (delta_time <= t_total)
            {
                // 减速段
                double t_decel_phase = delta_time - (t_accel + t_const);
                s = s_accel + v_max * t_const +
                    v_max * t_decel_phase + 0.5 * (-a) * t_decel_phase * t_decel_phase;
                currentSpeed = (v_max + (-a) * t_decel_phase) * 60000.0 / (substep * 200.0); // 更新当前速度，转换回 RPM
            }
            else
            {
                s = total_pulse; // 运动结束
                currentSpeed = 0;
            }
        }
        // 三角形加减速（无匀速段）
        else
        {
            // 重新计算峰值速度，考虑初始速度和方向
            double v_peak = sqrt(0.5*v0 * v0 +  a * total_pulse);  // 考虑初速度对峰值速度的影响
            // 给 v_peak 加上方向
            v_peak = (a > 0) ? v_peak : -v_peak;
            double t_acc = (v_peak - v0) / a; // 加速时间，从 v0 加速到 v_peak
            double t_dec = (0 - v_peak) / -a; // 从 v_peak 减速到 0 的时间
            double t_total =  t_acc + t_dec; // 总时间
            if (delta_time <= t_acc) {
                // 加速段
                double t_acc_phase = delta_time - t_acc;
                s = v0 * t_acc_phase + 0.5 * a * t_acc_phase * t_acc_phase;
                currentSpeed = (v0 + a * t_acc_phase) * 60000.0 / (substep * 200.0);
            } else if (delta_time <= t_total) {
                // 减速段
                double t_dec_phase = delta_time -  t_acc;
                s = v0 *t_acc + 0.5 * a * t_acc * t_acc +
                    v_peak * t_dec_phase + 0.5 * (-a) * t_dec_phase * t_dec_phase;
                currentSpeed = (v_peak + (-a) * t_dec_phase) * 60000.0 / (substep * 200.0);
            } else {
                s = total_pulse; // 运动结束
                currentSpeed = 0;
            }
        }
    }
      return s * convert_K * 1.0 / (substep * 200.0); // 返回位移量
}
/**
 * @brief    计算当前位置，
 * @param
 * @retval
 */
float TTL_Stepper::Calculate_CurrentPos() // 计算当前绝对位置
{
    currentPosition = previousPosition + Calculate_DeltaPos();
    return currentPosition;
}
/**
 * @brief    更新当前位置，
 * @param
 * @retval
 */
void TTL_Stepper::update_CurrentPos()
{
    int frame_header = 0;
    int frame_length = 8;
    protocol->Emm_V5_Read_Sys_Params(this->addr, S_CPOS);
    Date_Index = 0; // 清空缓冲区
    this->protocol->Emm_V5_Receive_Data(recDate, &Date_Index);
    while ((Date_Index - frame_header) >= frame_length)
    {
        if (recDate[frame_header] == this->addr &&
                recDate[frame_header + frame_length - 1] == 0x6B &&
                recDate[frame_header + 1] == 0x36)
        {
            // 解析符号（0x00正数，0x01负数）
            int sign = (recDate[frame_header + 2] == 0x00) ? 1 : -1;

            // 组合32位位置值（大端序解析）
            uint32_t positionValue =
                    (static_cast<uint32_t>(recDate[frame_header + 3]) << 24) |
                    (static_cast<uint32_t>(recDate[frame_header + 4]) << 16) |
                    (static_cast<uint32_t>(recDate[frame_header + 5]) << 8) |
                    static_cast<uint32_t>(recDate[frame_header + 6]);

            // 计算实际位置（注意除以65536）
            currentPosition = sign * static_cast<double>(positionValue) * this->convert_K / 65536.0;
            // 判断正负
            currentPosition = CW ? -currentPosition : currentPosition;
            // 移动帧头准备处理下一帧
            frame_header += frame_length;
        }
        else
        {
            frame_header++;
        }
    }
    Date_Index = 0;
}
/**
 * @brief    更新使能/到位/堵转状态标志位，执行完即更新
 * @param
 * @retval
 */
void TTL_Stepper::update_state()
{
    int frame_header = 0;
    int frame_length = 4;
    protocol->Emm_V5_Read_Sys_Params(this->addr, S_FLAG);
    Date_Index = 0; // 清空缓冲区
    this->protocol->Emm_V5_Receive_Data(recDate, &Date_Index);
    while ((Date_Index - frame_header) >= frame_length)
    {
        if (recDate[frame_header] == this->addr &&
                recDate[frame_header + frame_length - 1] == 0x6B &&
                recDate[frame_header + 1] == 0x3A)
        {
            enstate = (recDate[2] & 0x01) != 0;
            onPos_state = (recDate[2] & 0x02) != 0;
            locked_state = (recDate[2] & 0x04) != 0;
            loPro_state = (recDate[2] & 0x08) != 0;

            // 移动帧头准备处理下一帧
            frame_header += frame_length;
        }
        else
        {
            frame_header++;
        }
    }
    Date_Index = 0;
}
/**
 * @brief    发出多机回零（向上）命令,采用多圈无限位碰撞回零，
 * @param
 * @retval
 */
void TTL_Stepper::Multi_syn_RunToOrigin()
{
    do
    {
        protocol->Emm_V5_Origin_Trigger_Return(this->addr, 2, 1); // 发送命令触发回零,多圈无限位碰撞回零
    } while (!wrong_command_catch());
}

/**
 * @brief    当前电机初始化,设置上电自动使能，更新运动计算基准时间
 * @param
 * @retval
 */
void TTL_Stepper::init()
{
    ptime = millis();
    // this->state_update();//更新到位状态
    //  this->enable();//电机使能
    //  this->Emm_V5_Reset_CurPos_To_Zero(this->addr); // 将当前位置清零
}

/**
 * @brief    指定电机初始化
 * @param
 * @retval
 */
void TTL_Stepper::init(uint8_t addr, TTL_Protocol *protocol)
{
    this->addr = addr;         // 设定舵机的ID号
    this->protocol = protocol; // 舵机的通信协议
    this->Date_Index = 0;
    this->currentPosition = 0;
    this->currentSpeed = 0;
    this->delta = 0;
    init(); // 初始化舵机
}
/**
 * @brief    当前电机使能
 * @param
 * @retval
 */
bool TTL_Stepper::enable()
{
    do
    {
        protocol->Emm_V5_En_Control(this->addr, 1, 0); // 电机使能控制,此处选择默认不开启多机同步
        return 1;                                      // 偷懒，本应该根据返回值判断是否使能成功
    } while (!wrong_command_catch());
}

/**
 * @brief    更新使能/到位/堵转状态标志位
 * @param
 * @retval
 */
void TTL_Stepper::state_update()
{
    if (Ask_State)
    {
        while (protocol->serial->available())
        {
            recDate[Date_Index++] = protocol->serial->read();
            if (Date_Index > 3)
            {
                if (recDate[3] == 0x6B && recDate[1] == 0x3A && recDate[0] == addr)
                {
                    enstate = (recDate[2] & 0x01) != 0;
                    onPos_state = (recDate[2] & 0x02) != 0;
                    locked_state = (recDate[2] & 0x04) != 0;
                    loPro_state = (recDate[2] & 0x08) != 0;
                    recDate_Clear();
                }
                else
                {
                    recDate[0] = recDate[1];
                    recDate[1] = recDate[2];
                    recDate[2] = recDate[3];
                    Date_Index--; // 滑动
                }
            }
        }
    }
    else
    {
        protocol->emptyCache();
        protocol->Emm_V5_Read_Sys_Params(addr, S_FLAG);
        Ask_State = true;
    }
}
/**
 * @brief    清除查询状态和缓冲区
 * @param
 * @retval
 */
void TTL_Stepper::recDate_Clear()
{
    Ask_State = false;
    Date_Index = 0;
}
/**
 * @brief   读取原点回零状态
 * @param
 * @retval   0未完成1已完成
 */
int TTL_Stepper::Emm_V5_Origin_Read_state()
{
    uint8_t cmd[3] = {0};
    uint8_t *rxCmd, *rxCount;
    protocol->emptyCache();
    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x3B; // 功能码
    cmd[2] = 0x6B; // 校验字节

    // 发送命令
    protocol->serial->write(cmd, 3);
    delay(500);
    protocol->Emm_V5_Receive_Data(rxCmd, rxCount); // 返回数据接收函数
    for (int i = 0; i <= *rxCount - 4; i++)
    {
        if (rxCmd[i] == this->addr)
        {
            if (rxCmd[i + 1] == 0x3B && rxCmd[i + 3] == 0x6B)
            {
                if (rxCmd[i + 2] == 0x03)
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            }
        }
    }
    return 0;
}

/**
 * @brief    错误命令判断，通过步进返回值判断之前是否发出了错误命令，8/6新增没有接收到返回值时的判断
 * @param
 * @retval   0：发出了错误命令；1发出的是正确命令
 */
bool TTL_Stepper::wrong_command_catch()
{
    int erroraddr = protocol->error_address_catch();
    if (erroraddr == addr || erroraddr == 254)
    {
        command_check = 0;
        return 0;
    }
    else
    {
        command_check = 1;
        return 1;
    }
}
// Protocol函数
/**
 * @brief    将当前位置清零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x0A; // 功能码
    cmd[2] = 0x6D; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    serial->write(cmd, 4);
}

/**
 * @brief    解除堵转保护
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x0E; // 功能码
    cmd[2] = 0x52; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    serial->write(cmd, 4);
}

/**
 * @brief    读取系统参数
 * @param    addr  ：电机地址
 * @param    s     ：系统参数类型
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[i] = addr;
    ++i; // 地址

    switch (s) // 功能码
    {
    case S_VER:
        cmd[i] = 0x1F;
        ++i;
        break;
    case S_RL:
        cmd[i] = 0x20;
        ++i;
        break;
    case S_PID:
        cmd[i] = 0x21;
        ++i;
        break;
    case S_VBUS:
        cmd[i] = 0x24;
        ++i;
        break;
    case S_CPHA:
        cmd[i] = 0x27;
        ++i;
        break;
    case S_ENCL:
        cmd[i] = 0x31;
        ++i;
        break;
    case S_TPOS:
        cmd[i] = 0x33;
        ++i;
        break;
    case S_VEL:
        cmd[i] = 0x35;
        ++i;
        break;
    case S_CPOS:
        cmd[i] = 0x36;
        ++i;
        break;
    case S_PERR:
        cmd[i] = 0x37;
        ++i;
        break;
    case S_FLAG:
        cmd[i] = 0x3A;
        ++i;
        break;
    case S_ORG:
        cmd[i] = 0x3B;
        ++i;
        break;
    case S_Conf:
        cmd[i] = 0x42;
        ++i;
        cmd[i] = 0x6C;
        ++i;
        break;
    case S_State:
        cmd[i] = 0x43;
        ++i;
        cmd[i] = 0x7A;
        ++i;
        break;
    default:
        break;
    }

    cmd[i] = 0x6B;
    ++i; // 校验字节

    // 发送命令
    serial->write(cmd, i);
}

/**
 * @brief    修改开环/闭环控制模式
 * @param    addr     ：电机地址
 * @param    svF      ：是否存储标志，false为不存储，true为存储
 * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;      // 地址
    cmd[1] = 0x46;      // 功能码
    cmd[2] = 0x69;      // 辅助码
    cmd[3] = svF;       // 是否存储标志，false为不存储，true为存储
    cmd[4] = ctrl_mode; // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
    cmd[5] = 0x6B;      // 校验字节

    // 发送命令
    serial->write(cmd, 6);
}

/**
 * @brief    使能信号控制
 * @param    addr  ：电机地址
 * @param    state ：使能状态     ，true为使能电机，false为关闭电机
 * @param    snF   ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;           // 地址
    cmd[1] = 0xF3;           // 功能码
    cmd[2] = 0xAB;           // 辅助码
    cmd[3] = (uint8_t)state; // 使能状态
    cmd[4] = snF;            // 多机同步运动标志
    cmd[5] = 0x6B;           // calculateCRC8(cmd, 5);                       // 校验字节
    // 发送命令
    serial->write(cmd, 6);
}

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向       ，0为CW，其余值为CCW
 * @param    vel ：速度       ，范围0 - 5000RPM
 * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
*/
void TTL_Protocol::Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                // 校验字节

    // 发送命令
    serial->write(cmd, 13);
}

/**
 * @brief    立即停止（所有控制模式都通用）
 * @param    addr  ：电机地址
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0xFE; // 功能码
    cmd[2] = 0x98; // 辅助码
    cmd[3] = snF;  // 多机同步运动标志
    cmd[4] = 0x6B; // 校验字节

    // 发送命令
    serial->write(cmd, 5);
}

/**
 * @brief    多机同步运动
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0xFF; // 功能码
    cmd[2] = 0x66; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    serial->write(cmd, 4);
}

/**
 * @brief    设置单圈回零的零点位置
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x93; // 功能码
    cmd[2] = 0x88; // 辅助码
    cmd[3] = svF;  // 是否存储标志，false为不存储，true为存储
    cmd[4] = 0x6B; // 校验字节

    // 发送命令
    serial->write(cmd, 5);
}

/**
 * @brief    修改回零参数
 * @param    addr  ：电机地址
 * @param    svF   ：是否存储标志，false为不存储，true为存储
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    o_dir  ：回零方向，0为CW，其余值为CCW
 * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
 * @param    o_tm   ：回零超时时间，单位：毫秒
 * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param    potF   ：上电自动触发回零，false为不使能，true为使能
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};

    // 装载命令
    cmd[0] = addr;                    // 地址
    cmd[1] = 0x4C;                    // 功能码
    cmd[2] = 0xAE;                    // 辅助码
    cmd[3] = svF;                     // 是否存储标志，false为不存储，true为存储
    cmd[4] = o_mode;                  // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[5] = o_dir;                   // 回零方向
    cmd[6] = (uint8_t)(o_vel >> 8);   // 回零速度(RPM)高8位字节
    cmd[7] = (uint8_t)(o_vel >> 0);   // 回零速度(RPM)低8位字节
    cmd[8] = (uint8_t)(o_tm >> 24);   // 回零超时时间(bit24 - bit31)
    cmd[9] = (uint8_t)(o_tm >> 16);   // 回零超时时间(bit16 - bit23)
    cmd[10] = (uint8_t)(o_tm >> 8);   // 回零超时时间(bit8  - bit15)
    cmd[11] = (uint8_t)(o_tm >> 0);   // 回零超时时间(bit0  - bit7 )
    cmd[12] = (uint8_t)(sl_vel >> 8); // 无限位碰撞回零检测转速(RPM)高8位字节
    cmd[13] = (uint8_t)(sl_vel >> 0); // 无限位碰撞回零检测转速(RPM)低8位字节
    cmd[14] = (uint8_t)(sl_ma >> 8);  // 无限位碰撞回零检测电流(Ma)高8位字节
    cmd[15] = (uint8_t)(sl_ma >> 0);  // 无限位碰撞回零检测电流(Ma)低8位字节
    cmd[16] = (uint8_t)(sl_ms >> 8);  // 无限位碰撞回零检测时间(Ms)高8位字节
    cmd[17] = (uint8_t)(sl_ms >> 0);  // 无限位碰撞回零检测时间(Ms)低8位字节
    cmd[18] = potF;                   // 上电自动触发回零，false为不使能，true为使能
    cmd[19] = 0x6B;                   // 校验字节

    // 发送命令
    serial->write(cmd, 20);
}

/**
 * @brief    触发回零
 * @param    addr   ：电机地址
 * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
 * @param    snF   ：多机同步标志，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;   // 地址
    cmd[1] = 0x9A;   // 功能码
    cmd[2] = o_mode; // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[3] = snF;    // 多机同步运动标志，false为不启用，true为启用
    cmd[4] = 0x6B;   // calculateCRC8(cmd, 5); calculateCRC8(cmd,4);                       // 校验字节

    // 发送命令
    serial->write(cmd, 5);
}

/**
 * @brief    强制中断并退出回零
 * @param    addr  ：电机地址
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void TTL_Protocol::Emm_V5_Origin_Interrupt(uint8_t addr)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr; // 地址
    cmd[1] = 0x9C; // 功能码
    cmd[2] = 0x48; // 辅助码
    cmd[3] = 0x6B; // 校验字节

    // 发送命令
    serial->write(cmd, 4);
}

/**
 * @brief    接收数据
 * @param    rxCmd   : 接收到的数据缓存在该数组
 * @param    rxCount : 接收到的数据长度
 * @retval   无
 */
void TTL_Protocol::Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount)
{
    const int MAX_BUFFER_SIZE = 256; // 定义最大缓冲区大小
    int i = 0;
    unsigned long lTime;  // 上一时刻的时间
    unsigned long cTime;  // 当前时刻的时间
    bool end_rec = false; // 是否接收到了帧尾
    // 记录当前的时间
    lTime = cTime = millis();

    // 开始接收数据
    while (1)
    {
        if (serial->available() > 0) // 串口有数据进来
        {
            if (i < MAX_BUFFER_SIZE) // 防止数组溢出
            {
                rxCmd[i++] = serial->read(); // 接收数据
                if (rxCmd[i - 1] == 0x6B)
                {
                    end_rec = true;
                }
                lTime = millis(); // 更新上一时刻的时间
            }
            else
            {
                // 缓冲区溢出，清空缓冲区
                while (serial->read() > 0);
                break;
            }
        }
        else // 串口没有数据
        {
            cTime = millis(); // 获取当前时刻的时间

            if ((int)(cTime - lTime) > 50 || end_rec) // 50毫秒内串口没有数据进来或已经接收到帧尾，就判定一帧数据接收结束
            {
                *rxCount = i; // 数据长度

                break; // 退出while(1)循环
            }
        }
    }
}

/**
 * @brief    串口通讯协议对象构造器函数
 * @param    serial:串口
 * @param    baudrate:波特率
 * @retval   无
 */
TTL_Protocol::TTL_Protocol(HardwareSerial *serial, uint32_t baudrate)
{
    this->serial = serial;
    this->baudrate = baudrate;
    // 初始化波特率
    serial->begin(baudrate);
}

/**
 * @brief    建立串口通讯协议初始化
 * @param    serial:串口
 * @param    baudrate:波特率
 * @retval   无
 */
void TTL_Protocol::init(HardwareSerial *serial, uint32_t baudrate)
{
    this->baudrate = baudrate;
    serial->begin(baudrate);
    this->serial = serial;
}

/**
 * @brief    清口串口
 * @param
 * @retval   无
 */
void TTL_Protocol::emptyCache()
{
    // 清空UART接收缓冲区
    while (serial->read() > 0)
        ;
}

/**
 * @brief    触发多机同步，（发送了多机通讯命令的）全部
 * @param
 * @retval   无
 */
void TTL_Protocol::synrun()
{
    do
    {
        Emm_V5_Synchronous_motion(0); // 触发多机同步开始运动
    } while (error_address_catch() == 255);
}

/**
 * @brief    仅确认是否发出错误命令，返回一个错误命令步进地址，否则返回255，添加条件不满足的判断，8/6添加未收到返回值的判断
 * @param
 * @retval   错误命令步进地址
 */
uint8_t TTL_Protocol::error_address_catch()
{
    uint8_t *rxCmd;
    uint8_t rxCount;
    this->Emm_V5_Receive_Data(rxCmd, &rxCount);
    for(int i=0;i<=rxCount-4;i++){
        if(rxCmd[i+3]!=0x6B){
            continue;
        }
        if(rxCmd[i+1]==0x00 && rxCmd[i+2]==0xEE){
            return rxCmd[0];
        }
        else if(rxCmd[i+1]==0xFD && rxCmd[i+2]==0xE2){
            return rxCmd[0];
        }
        else if(rxCmd[i+2]==0x02){//正确命令
            return 255;
        }
    }
    return 254;//没有接收到返回
    // uint8_t *rxCmd;
    // int i = -1;
    // unsigned long lTime;  // 上一时刻的时间
    // unsigned long cTime;  // 当前时刻的时间
    // lTime=cTime=millis();
    // while (serial->available() <= 3)
    // {
    //   cTime=millis();
    //   if((int)(cTime - lTime) > 5)//五毫秒无返回默认发出正确指令
    //   {
    //     return 255;
    //   }
    // }; // 等到返回至少四个数据
    // do
    // {
    //   i++;
    //   rxCmd[i] = serial->read();
    // } while (rxCmd[i] != 0x6B);
    // if (i == 3)
    // {
    //   if (rxCmd[1] == 0x00 && rxCmd[2] == 0xEE)
    //   {
    //     return rxCmd[0];
    //   }
    //   else if (rxCmd[1] == 0xFD && rxCmd[2] == 0xE2)
    //   { // 条件不满足
    //     return rxCmd[0];
    //   }
    //   else
    //   {
    //     return 255;
    //   }
    // }
    // else
    // {
    //   return 255;
    // }
}
ARM_WHOLE_STEPPER::ARM_WHOLE_STEPPER(TTL_Stepper *Stepper_SHUZHI, TTL_Stepper *Stepper_SHUIPIN, TTL_Protocol *protocol)
{
    this->Stepper_SHUIPIN = Stepper_SHUIPIN;
    this->Stepper_SHUZHI = Stepper_SHUZHI;
    this->protocol = protocol;
    this->enable = false;
    this->Data_Index = 0;
}

/**
 * @brief    更新电机状况，到位、正确命令、错误命令、条件不满足
 * @param
 * @retval   错误命令步进地址
 */
void ARM_WHOLE_STEPPER::Stepper_State_Update()
{
    if (!this->enable)
        return;
    while (protocol->serial->available())
    {
        Data_Index++;
        recData[Data_Index] = protocol->serial->read();
        if (Data_Index == 3 && recData[3] == 0x6B)
        {
            if (recData[0] == Stepper_SHUZHI->addr)
            {
                aim_Stepper = Stepper_SHUZHI;
            }
            else if (recData[0] == Stepper_SHUIPIN->addr)
            {
                aim_Stepper = Stepper_SHUIPIN;
            }
            else
            {
                recData[0] = recData[1];
                recData[1] = recData[2];
                recData[2] = recData[3];
                Data_Index = 2; // 滑动
                continue;
            }
            if (recData[1] == 0xFD && recData[2] == 0x9F)
            {
                aim_Stepper->onPos_state = true;
            }
            else if (recData[1] == Stepper_SHUZHI->Func_Code)
            {
                if (recData[2] == 0x02)
                {
                    aim_Stepper->command_check = true;
                }
                else if (recData[2] == 0xE2)
                {
                    aim_Stepper->conditionNotMet = true;
                }
            }
            else if (recData[1] == 0x00 && recData[2] == 0xEE)
            {
                aim_Stepper->command_check = false;
            }
        }
    }
}