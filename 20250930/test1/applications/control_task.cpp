#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
#include "io/buzzer/buzzer_alert.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

// C板
sp::DBus remote(&huart3);
// 引用蜂鸣器
extern sp::Buzzer buzzer;

// 实例化 CAN2
sp::CAN can2(&hcan2);

// 定义四个 PID 控制器（每个轮子一个）
sp::PID pid_lf(0.001f, 0.5f, 0.0f, 0.0f, 3.0f, 1.0f, 0.8f, false, true);
sp::PID pid_rf(0.001f, 0.5f, 0.0f, 0.0f, 3.0f, 1.0f, 0.8f, false, true);
sp::PID pid_rr(0.001f, 0.5f, 0.0f, 0.0f, 3.0f, 1.0f, 0.8f, false, true);
sp::PID pid_lr(0.001f, 0.5f, 0.0f, 0.0f, 3.0f, 1.0f, 0.8f, false, true);

// 达妙
// sp::DBus remote(&huart5, false);

// 定义四个 M3508 电机
sp::RM_Motor motor_rf(1, sp::RM_Motors::M3508, 3591.0f / 187.0f);
sp::RM_Motor motor_lf(2, sp::RM_Motors::M3508, 3591.0f / 187.0f);
sp::RM_Motor motor_lr(3, sp::RM_Motors::M3508, 3591.0f / 187.0f);
sp::RM_Motor motor_rr(4, sp::RM_Motors::M3508, 3591.0f / 187.0f);

// 麦克纳姆底盘，轮半径 0.077m，前后/左右半宽各 0.185m
sp::Mecanum mecanum(0.077f, 0.185f, 0.165f, false, false, true, true);  // 电机方向是否反转

// 控制效率相关的参数
const float T_rated = 3.0f;                                    // N·m
const float omega_rated = 469.0f * 2.0f * 3.1415926f / 60.0f;  // rad/s
const float eta = 0.7f;                                        // 系统效率
const float P_max_motor = T_rated * omega_rated / eta;         // W
const float P_max_total = 4.0f * P_max_motor;                  // 四轮总功率

extern "C" void control_task(void * argument)
{
  can2.config();
  can2.start();

  // 初始请求遥控器
  remote.request();

  float cmd_lf = 0.0f, cmd_lr = 0.0f, cmd_rf = 0.0f, cmd_rr = 0.0f;

  while (true) {
    // 默认速度
    float vx = 0.0f;  // 前后
    float vy = 0.0f;  // 左右
    float wz = 0.0f;  // 旋转
    float fb_lf, fb_lr, fb_rf, fb_rr;
    float power_total;

    switch (remote.sw_r) {
      case sp::DBusSwitchMode::DOWN:
        // 下档：所有电机失能
        vx = vy = wz = 0.0f;
        // 计算各轮速度
        mecanum.calc(vx, vy, wz);
        // 将速度赋值给电机
        motor_lf.cmd(mecanum.speed_lf);
        motor_lr.cmd(mecanum.speed_lr);
        motor_rf.cmd(mecanum.speed_rf);
        motor_rr.cmd(mecanum.speed_rr);

        break;

      case sp::DBusSwitchMode::MID:
        // // 中档：根据左摇杆和右摇杆控制底盘
        // // 左摇杆纵向控制前后 (ch_lv)
        vx = remote.ch_lv * 1.0f;  // 假设最大 0.5 m/s，可根据需要调节

        // 左摇杆横向控制左右 (ch_lv)
        vy = -remote.ch_lh * 1.0f;  // 假设最大 0.5 m/s

        // 右摇杆纵向控制旋转 (ch_rv)
        if (remote.ch_rv > 0.1f) {
          wz = 1.0f;  // 右摇杆前推动 → 逆时针旋转
        }
        else if (remote.ch_rv < -0.1f) {
          wz = -1.0f;  // 右摇杆后推动 → 顺时针旋转
        }
        else {
          wz = 0.0f;
        }

        // vx = 0.25f;
        // vy = 0.0f;
        // wz = 0.0f;

        // 计算各轮速度
        mecanum.calc(vx, vy, wz);

        // 功率限制
        power_total = 0.0f;
        power_total += fabs(mecanum.speed_lf * T_rated / eta);
        power_total += fabs(mecanum.speed_lr * T_rated / eta);
        power_total += fabs(mecanum.speed_rf * T_rated / eta);
        power_total += fabs(mecanum.speed_rr * T_rated / eta);

        if (power_total > P_max_total) {
          float scale = P_max_total / power_total;
          mecanum.speed_lf *= scale;
          mecanum.speed_lr *= scale;
          mecanum.speed_rf *= scale;
          mecanum.speed_rr *= scale;
        }

        // PID
        fb_lf = motor_lf.speed;
        fb_lr = motor_lr.speed;
        fb_rf = motor_rf.speed;
        fb_rr = motor_rr.speed;

        pid_lf.calc(mecanum.speed_lf, fb_lf);
        cmd_lf = pid_lf.out;
        pid_lr.calc(mecanum.speed_lr, fb_lr);
        cmd_lr = pid_lr.out;
        pid_rf.calc(mecanum.speed_rf, fb_rf);
        cmd_rf = pid_rf.out;
        pid_rr.calc(mecanum.speed_rr, fb_rr);
        cmd_rr = pid_rr.out;

        motor_lf.cmd(cmd_lf);
        motor_lr.cmd(cmd_lr);
        motor_rf.cmd(cmd_rf);
        motor_rr.cmd(cmd_rr);

        // 将速度赋值给电机
        // motor_lf.cmd(mecanum.speed_lf);
        // motor_lr.cmd(mecanum.speed_lr);
        // motor_rf.cmd(mecanum.speed_rf);
        // motor_rr.cmd(mecanum.speed_rr);

        break;

      case sp::DBusSwitchMode::UP:
        vx = vy = wz = 0.0f;  // 电机停
        // 将速度赋值给电机
        motor_lf.cmd(mecanum.speed_lf);
        motor_lr.cmd(mecanum.speed_lr);
        motor_rf.cmd(mecanum.speed_rf);
        motor_rr.cmd(mecanum.speed_rr);

        {
          float ch = remote.ch_lv;  // -1.0 ~ +1.0
          if (fabs(ch) > 0.1f) {    // 中间小范围不触发，避免抖动
            // 把 [-1,1] 映射到 [0,7]
            int index = (int)((ch + 1.0f) / 2.0f * 7.0f);
            if (index < 0) index = 0;
            if (index > 7) index = 7;

            // Do Re Mi Fa Sol La Si Do
            static const float freqs[8] = {262, 294, 330, 349, 392, 440, 494, 523};

            float freq = freqs[index];
            buzzer.set(freq, 0.2f);  // 0.2秒时长
            buzzer.start();
          }
          else {
            buzzer.stop();
          }
        }
        break;

      default:
        vx = vy = wz = 0.0f;
        // 计算各轮速度
        mecanum.calc(vx, vy, wz);
        // 将速度赋值给电机
        motor_lf.cmd(mecanum.speed_lf);
        motor_lr.cmd(mecanum.speed_lr);
        motor_rf.cmd(mecanum.speed_rf);
        motor_rr.cmd(mecanum.speed_rr);
        break;
    }

    // // 写入 CAN 数据
    // motor_lf.write(can2.tx_data);
    // motor_lr.write(can2.tx_data);
    // motor_rf.write(can2.tx_data);
    // motor_rr.write(can2.tx_data);

    // // 发送 CAN 数据
    // can2.send(motor_lf.tx_id);
    // can2.send(motor_lr.tx_id);
    // can2.send(motor_rf.tx_id);
    // can2.send(motor_rr.tx_id);

    osDelay(10);  // 100 Hz 控制循环
  }
}

// UART 接收中断
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();
  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

// UART 错误中断
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}

// CAN 接收中断
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan2) {
      can2.recv();
      // 更新电机状态
      if (can2.rx_id == motor_lf.rx_id) motor_lf.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == motor_lr.rx_id) motor_lr.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == motor_rf.rx_id) motor_rf.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == motor_rr.rx_id) motor_rr.read(can2.rx_data, stamp_ms);
    }
  }
}
