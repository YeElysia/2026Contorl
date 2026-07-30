/********************************************
//使用mega2560读取hwt101数据的例程
//  tips: 使用mega2560的硬件串口三与hwt101进行通讯
//  参考文章： https://zhuanlan.zhihu.com/p/367614669
//  联系方式： 知乎 “Poao”
//                    2021/5/11  —— by Poao
**********************************************/

/************ HWT101- uart3 *************/
#include <Wire.h>
#include <JY901.h>

float yaw;   // 当前角度数据  
float Gyro;  // 当前角加速度  


void setup() {

  /*--------------硬件串口初始化----------------*/
  Serial.begin(115200);

  //uart3 ---  to hwt101 
  Serial3.begin(115200);
  
}

void loop() {

  /***  由于hwt101只能输出z轴角度，角加速度，故已将无用部分删去  ***/
  // 串口3接收到数据后，进行数据的读取与存储。
   while (Serial3.available()) 
  {
    JY901.CopeSerialData(Serial3.read()); //Call JY901 data cope function
   }
   
  // 储存数据
  Gyro = (float)JY901.stcGyro.w[2]/32768*2000;
  yaw = (float)JY901.stcAngle.Angle[2]/32768*180;

  // 串口打印数据
  Serial.print("Gyro:");Serial.println(Gyro);
  Serial.print("Angle:");Serial.println(yaw);

  delay(100);     // 100ms 进行一次数据读取
}
