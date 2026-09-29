// 串口屏调试：轮询 get 指令读取控件值，控制电机
// n0=升降位置  n1=伸缩位置  n2=旋转角度
// n3=储料盘    n4=手爪    n5=手爪旋转
#include <Arduino.h>
#include "pins.h"
#include "Storage.h"
#include "Arm.h"
#include "TTL_STEPPER.h"

HardwareSerial Serial_DEBUG(DEBUG_RX, DEBUG_TX);
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);

// 步进电机
HardwareSerial Serial_Stepper(Stepper_RX, Stepper_TX);
TTL_Protocol Stepper_protocol(&Serial_Stepper, STEPPER_BAUDRATE);
TTL_Stepper armStepper(7, &Stepper_protocol);
TTL_Stepper gripperStepper(6, &Stepper_protocol);

HardwareSerial Serial_ArmBase(ArmBaseStepper_Rx, ArmBaseStepper_Tx);
TTL_Protocol ArmBase_protocol(&Serial_ArmBase, STEPPER_BAUDRATE);
TTL_Stepper armBaseStepper(5, &ArmBase_protocol);

// 控件映射: n0=储料盘 n7=手爪 n8=伸缩 n9=升降 n10=旋转
int lastVal[11] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

void setup()
{
    delay(1000);
    Serial_DEBUG.begin(115200);
    Serial_TJCHMI.begin(TJCHMI_BAUDRATE);

    // 舵机
    Serial_SERVO.begin(SERVO_BAUDRATE);
    protocol.init(&Serial_SERVO, SERVO_BAUDRATE);
    Storage_Init();
    Arm_Init();

    // 步进
    armStepper.init();
    armStepper.set(250, 255, 0, 120, 16);
    gripperStepper.init();
    gripperStepper.set(150, 254, 1, 1131, 16);
    armBaseStepper.init();
    armBaseStepper.set(50, 250, 1, 900, 16);

    Serial_TJCHMI.print("page debug\xff\xff\xff");
    Serial_DEBUG.println("=== Polling screen values ===");
}

int parseResponse()
{
    if (Serial_TJCHMI.available() < 8)
        return -1;

    unsigned char buf[8];
    unsigned char h = Serial_TJCHMI.peek();

    if (h == 0x71)
    {
        Serial_TJCHMI.readBytes(buf, 8);
        if (buf[5] == 0xFF && buf[6] == 0xFF && buf[7] == 0xFF)
        {
            // 方法1：使用无符号类型累加再转换
            uint32_t uval = (uint32_t)buf[1] |
                            ((uint32_t)buf[2] << 8) |
                            ((uint32_t)buf[3] << 16) |
                            ((uint32_t)buf[4] << 24);
            int val = (int)uval; // 符号由最高位决定

            // 或方法2：直接使用int，但明确使用无符号转换
            // int val = (int)((uint32_t)buf[1] | ((uint32_t)buf[2] << 8) |
            //                ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24));

            while (Serial_TJCHMI.available())
                Serial_TJCHMI.read();
            return val;
        }
    }
    else
    {
        while (Serial_TJCHMI.available())
            Serial_TJCHMI.read();
    }
    return -1;
}

void pollControl(int idx)
{
    char cmd[32];
    sprintf(cmd, "get n%d.val\xff\xff\xff", idx);
    Serial_TJCHMI.print(cmd);

    delay(30); // 等屏幕回复

    int val = parseResponse();
    if (val >= 0 && val != lastVal[idx])
    {
        lastVal[idx] = val;
        Serial_DEBUG.print("n");
        Serial_DEBUG.print(idx);
        Serial_DEBUG.print(".val=");
        Serial_DEBUG.println(val);

        switch (idx)
        {
        case 0:
            if (val == 1)
            {
                storageServo.setAngle(-90);
                delay(500);
            }
            else if (val == 2)
            {
                storageServo.setAngle(0);
                delay(500);
            }
            else if (val == 3)
            {
                storageServo.setAngle(90);
                delay(500);
            }
            break;
        case 7:
            gripperServo.setAngle(val);
            break;
        case 8:
            gripperStepper.runToNewPosition(val);
            delay(500);
            break;
        case 9:
            armStepper.runToNewPosition(val);
            delay(500);
            break;
        case 10:
            armBaseStepper.setAngle(val);
            delay(500);
            break;
        }
    }
}

void loop()
{
    // n0, n7, n8, n9, n10
    static int pollList[] = {0, 7, 8, 9, 10};
    static int pi = 0;
    static unsigned long lastPoll = 0;

    if (millis() - lastPoll > 200)
    {
        lastPoll = millis();
        pollControl(pollList[pi]);
        pi = (pi + 1) % 5;
    }
}
