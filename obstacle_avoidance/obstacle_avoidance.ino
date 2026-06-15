#include <RPLidar.h>
#include "MecanumDriver.h" 

// 创建激光雷达对象
RPLidar lidar;
// 创建麦克纳姆轮驱动器对象 (引脚分配: 左前, 右前, 左后, 右后... 根据具体驱动板定义)
MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);
// 用于存储激光雷达数据的数组（0到359度共360个值），初始清零
float distances[360] = { 0 };

void setup() {
  Serial.begin(115200);  // 开启用于调试的串口
  lidar.begin(Serial2);  // 将激光雷达连接到串口2（引脚19、20）
  mecanum.begin();       // 启动麦克纳姆轮驱动器
}

void loop() {
  // 1. 获取并更新雷达数据
  if (IS_OK(lidar.waitPoint())) {                                // 等到一个新的扫描点
    float distance = lidar.getCurrentPoint().distance / 1000.0;  // 获取距离值，单位转换为米(m)
    int angle = lidar.getCurrentPoint().angle;                   // 获取角度值（整数，四舍五入）
    bool startBit = lidar.getCurrentPoint().startBit;            // 每进入一次新的360度扫描圈时为true
    
    if (angle >= 0 && angle < 360) {                             // 确保角度值在[0, 359]合法范围内
      distances[angle] = distance;                               // 将距离值存储到对应角度的数组中
    }

    // 2. 一圈扫描结束，处理数据并执行避障控制
    if (startBit) {             
      float distance_min = 10;  // 用于存储最近障碍物距离 (初始设为一个较大的值，如10m)
      int angle_min = 0;        // 用于存储最近障碍物对应角度
      
      // 遍历寻找[0, 359]范围内最近障碍物距离及其对应角度
      for (int angle = 0; angle < 360; angle++) {
        float distance = distances[angle];
        // 激光雷达的最小量程通常为0.15m，>=0.15的才是有效数据（过滤掉雷达自身的遮挡死角）
        if (distance >= 0.15) {           
          if (distance < distance_min) {  // 如果发现更近的障碍物
            distance_min = distance;      // 更新最近距离
            angle_min = angle;            // 更新对应角度
          }
        }
      }

      // 打印最近障碍物的信息用于调试
      Serial.printf("angle: %3d -> distance: %4.2f m\r\n", angle_min, distance_min);

      // 3. 八向避障运动控制逻辑
      // 设定安全距离为 0.5m
      if (distance_min > 0.5) {
        // 周围0.5m内没有障碍物，停止运动 (或者这里可以改为你的巡航/前进代码)
        mecanum.driveAllMotor(0, 0, 0, 0); 
      } else {
        // 当0.5m内有障碍物时，将360度分为8个扇区(每个45度)，根据障碍物所在扇区，向相反方向逃逸
        // 这里的逻辑公式是: [ -22.5 + 45*(n-1), -22.5 + 45*n )，使得0度、45度、90度等正好在扇区正中心
        
        // 扇区1：正前方有障碍 (中心0度: 337.5°~22.5°) -> 向正后方逃逸
        if (angle_min >= -22.5 + 360 || angle_min < -22.5 + 45 * 1) 
          mecanum.driveAllMotor(100, 100, 100, 100);
          
        // 扇区2：右前方有障碍 (中心45度: 22.5°~67.5°) -> 向左后方逃逸
        else if (angle_min >= -22.5 + 45 * 1 && angle_min < -22.5 + 45 * 2) 
          mecanum.driveAllMotor(100, 0, 0, 100);
          
        // 扇区3：正右方有障碍 (中心90度: 67.5°~112.5°) -> 向正左方平移逃逸
        else if (angle_min >= -22.5 + 45 * 2 && angle_min < -22.5 + 45 * 3) 
          mecanum.driveAllMotor(100, -100, -100, 100);
          
        // 扇区4：右后方有障碍 (中心135度: 112.5°~157.5°) -> 向左前方逃逸
        else if (angle_min >= -22.5 + 45 * 3 && angle_min < -22.5 + 45 * 4) 
          mecanum.driveAllMotor(0, -100, -100, 0);
          
        // 扇区5：正后方有障碍 (中心180度: 157.5°~202.5°) -> 向正前方逃逸
        else if (angle_min >= -22.5 + 45 * 4 && angle_min < -22.5 + 45 * 5) 
          mecanum.driveAllMotor(-100, -100, -100, -100);
          
        // 扇区6：左后方有障碍 (中心225度: 202.5°~247.5°) -> 向右前方逃逸
        else if (angle_min >= -22.5 + 45 * 5 && angle_min < -22.5 + 45 * 6) 
          mecanum.driveAllMotor(-100, 0, 0, -100);
          
        // 扇区7：正左方有障碍 (中心270度: 247.5°~292.5°) -> 向正右方平移逃逸
        else if (angle_min >= -22.5 + 45 * 6 && angle_min < -22.5 + 45 * 7) 
          mecanum.driveAllMotor(-100, 100, 100, -100);
          
        // 扇区8：左前方有障碍 (中心315度: 292.5°~337.5°) -> 向右后方逃逸
        else if (angle_min >= -22.5 + 45 * 7 && angle_min < -22.5 + 45 * 8) 
          mecanum.driveAllMotor(0, 100, 100, 0);
      }
    }
  } else {
    // 4. 异常处理：如果没有获取到有效雷达数据，尝试重新连接并启动激光雷达
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      lidar.startScan(); // 重启扫描
      delay(1000);       // 给雷达一点启动时间
    }
  }
}
