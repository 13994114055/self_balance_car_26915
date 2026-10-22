#include <stdio.h>
#include "balance_control.h"

int main(void)
{
    Balance_t bal;
    BalanceParam_t p = {
        .stand_kp = 10.0f, .stand_kd = 1.0f,
        .speed_kp = 2.0f,  .speed_ki = 0.0f,
        .turn_kp  = 0.5f,  .turn_kd  = 0.0f,
    };
    Balance_Init(&bal, &p);

    puts("=== A 直立P: 前倾+5, 期望 balance_out > 0 ===");
    bal.pitch_target = 0; bal.status.pitch = 5; bal.gyro_pitch_v = 0; bal.turn_out = 0;
    Balance_UpdateFast(&bal);
    printf("balance_out=%f  left=%f  right=%f\n",
           bal.balance_out, bal.left_duty, bal.right_duty);

    puts("=== B 直立D: 快速前倒+50, 期望 balance_out > 0 ===");
    bal.status.pitch = 0; bal.gyro_pitch_v = 50;
    Balance_UpdateFast(&bal);
    printf("balance_out=%f\n", bal.balance_out);

    puts("=== C 速度环: set=1 now=0, 期望 pitch_target > 0 ===");
    bal.speed_set = 1; bal.speed_now_left = 0; bal.speed_now_right = 0;
    Balance_UpdateSlow(&bal);
    printf("pitch_target=%f\n", bal.pitch_target);

    puts("=== D 转向: turn_set=+10, 期望 left > right ===");
    bal.status.yaw = 0; bal.turn_set = 10;
    Balance_UpdateSlow(&bal);
    bal.pitch_target = 0; bal.status.pitch = 0; bal.gyro_pitch_v = 0;
    Balance_UpdateFast(&bal);
    printf("turn_out=%f  left=%f  right=%f\n",
           bal.turn_out, bal.left_duty, bal.right_duty);

    return 0;
}