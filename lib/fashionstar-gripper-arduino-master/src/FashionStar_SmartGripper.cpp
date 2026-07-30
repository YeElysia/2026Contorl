/**
 * Fashion Star 智能机械爪
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/11/14
 */

#include "FashionStar_SmartGripper.h"

/**
  * @brief     家具
  * @param    
  * @retval   
  */
FSGP_Gripper::FSGP_Gripper(){
    
}
/**
  * @brief     夹具
  * @param    
  * @retval   
  */
FSGP_Gripper::FSGP_Gripper(FSUS_Servo *servo, float angleGripperOpen, float angleGripperClose)
{   
    // 参数赋值
    this->servo = servo;                          // 舵机指针初始化
    this->angleGripperOpen = angleGripperOpen;    // 爪子开启的角度
    this->angleGripperClose = angleGripperClose;  // 爪子闭合的角度
    // 默认参数设置
    this->maxPower = FSGP_DEFAULT_POWER_MW;
    this->interval = FSGP_DEFAULT_INTERVAL;
    this->curPower = 0;
    // 设置状态位
    this->status = FSGP_STATUS_OPEN;
}

/**
  * @brief     夹具
  * @param    
  * @retval   
  */
FSGP_Gripper::FSGP_Gripper(FSUS_Servo *servo, float angleGripperOpen, float angleGripperClose,float angleGripperMax)
{   
    // 参数赋值
    this->servo = servo;                          // 舵机指针初始化
    this->angleGripperOpen = angleGripperOpen;    // 爪子开启的角度
    this->angleGripperClose = angleGripperClose;  // 爪子闭合的角度
    this->angleGripperMax = angleGripperMax;      // 爪子最大的角度 
    // 默认参数设置
    this->maxPower = FSGP_DEFAULT_POWER_MW;
    this->interval = FSGP_DEFAULT_INTERVAL;
    this->curPower = 0;
    // 设置状态位
    this->status = FSGP_STATUS_OPEN;
}

/**
  * @brief     初始化舵机
  * @param    
  * @retval   
  */
bool FSGP_Gripper::init()
{
    // 舵机机初始化
    this->servo->init();
    if(!this->servo->isOnline){
        return false; // 舵机不在线
    }
    // 爪子开启
    this->servo->setAngle(this->angleGripperOpen);  // 设置舵机的角度
    this->servo->wait();
    // this->open();
    
    return true;
}

bool FSGP_Gripper::init(FSUS_Servo *servo, float angleGripperOpen, float angleGripperClose){
    // 参数赋值
    this->servo = servo;                          // 舵机指针初始化
    this->angleGripperOpen = angleGripperOpen;    // 爪子开启的角度
    this->angleGripperClose = angleGripperClose;  // 爪子闭合的角度
    // 默认参数设置
    this->maxPower = FSGP_DEFAULT_POWER_MW;
    this->interval = FSGP_DEFAULT_INTERVAL;
    this->curPower = 0;
    // 设置状态位
    this->status = FSGP_STATUS_OPEN;
    
    this->init();
}


/**
  * @brief     // 设置电机的最大功率
  * @param    
  * @retval   
  */
void FSGP_Gripper::setMaxPower(uint16_t power)
{
    this->maxPower = power;
}

/**
  * @brief     查询舵机原始角度
  * @param    
  * @retval   
  */
void FSGP_Gripper::setAngle(float angle, uint16_t interval, uint16_t power){
    ptime=millis();
    this->servo->setAngle(angle, interval, power);
}


/**
  * @brief     爪子打开，必定到位
  * @param    
  * @retval   
  */
void FSGP_Gripper::open()
{
    ptime=millis();
    // 设置状态位
    this->status = FSGP_STATUS_OPEN;
    // 设置舵机角度
    this->servo->setAngle(this->angleGripperOpen, this->interval, 0);
    // 等待舵机旋转到位
    this->servo->wait();
}
/**
  * @brief     爪子打开，无等待
  * @param    
  * @retval   
  */
void FSGP_Gripper::Unfold()
{
    ptime=millis();
    // 设置状态位
    this->status = FSGP_STATUS_OPEN;
    // 设置舵机角度
    this->servo->setAngle(this->angleGripperOpen, this->interval, 0);
}

/**
  * @brief     爪子张到最大，没有到位等待
  * @param    
  * @retval   
  */
void FSGP_Gripper::openMax()
{
    ptime=millis();
    // 设置状态位
    this->status = FSGP_STATUS_OPEN;
    // 设置舵机角度
    this->servo->setAngle(this->angleGripperMax, this->interval, 0);
}

/**
  * @brief     爪子关闭，没有到位等待
  * @param    
  * @retval   
  */
void FSGP_Gripper::close()
{
    ptime=millis();
    this->status = FSGP_STATUS_CLOSE;
    this->servo->setAngle(this->angleGripperClose, this->interval, this->maxPower);
}

/**
  * @brief     到位等待
  * @param    
  * @retval   
  */
void FSGP_Gripper::wait(){
  this->servo->wait();   
}

/**
  * @brief     重设角度
  * @param    
  * @retval   
  */
void FSGP_Gripper::set(float angleGripperOpen, float angleGripperClose,float angleGripperMax){
    this->angleGripperOpen = angleGripperOpen;    // 爪子开启的角度
    this->angleGripperClose = angleGripperClose;  // 爪子闭合的角度
    this->angleGripperMax=angleGripperMax;        //爪子最大的角度
}

/**
  * @brief     查询当前的功率
  * @param    
  * @retval   
  */
uint16_t FSGP_Gripper::queryPower()
{
    return this->servo->queryPower();
}

/**
  * @brief     查询当前夹爪的状态
  * @param    
  * @retval   
  */
uint8_t FSGP_Gripper::getStatus()
{
    return this->status;
}

/**
  * @brief     更新当前的状态
  * @param    
  * @retval   
  */
uint8_t FSGP_Gripper::updateStatus()
{
    // TODO
}
/**
  * @brief     判断爪子是否卡死
  * @param    
  * @retval   
  */
bool FSGP_Gripper::isFrozen()
{
    ctime=millis();
    if(ctime-ptime>this->interval+250){
      return true;
    }else{
      return false;
    }
}