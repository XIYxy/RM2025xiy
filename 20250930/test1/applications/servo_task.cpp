#include "cmsis_os.h"
#include "io/servo/servo.hpp"

// C板
sp::Servo servo(&htim1, TIM_CHANNEL_1, 168e6f, 180.0f);  // 开发板最上面的PWM端口, 180度舵机

// 达妙
// sp::Servo servo(&htim1, TIM_CHANNEL_3, 240e6f, 180.0f); // 开发板最上面的PWM端口, 180度舵机

extern "C" void servo_task()
{
  servo.start();  // 启动 PWM 输出

  float angle = 0.0f;

  // 从 0° 每秒加 45° 到 180°
  for (angle = 0.0f; angle <= 180.0f; angle += 45.0f) {
    servo.set(angle);  // 设置舵机角度
    osDelay(1000);     // 延时 1 秒
  }

  // 最后停在 180°，保持不动
  while (true) {
    osDelay(1000);  // 空循环，防止任务退出
  }
}