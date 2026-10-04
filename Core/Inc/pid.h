#ifndef __PID_H
#define __PID_H

typedef enum {
    PID_OK = 0,
    PID_ERR_NULL,
    PID_ERR_DT,
} PID_Status_t;

typedef  struct {
    float kp;
    float ki;
    float kd;
    float max;
    float min;
    float dt;
    float error_last;
    float error_i;      //积分使用的e,死区不清零
    float error_pd;     //pd使用的e, 死区时清零
    float e_sum;
    float i_max;        //用来通过限制e_sum来限制最终积分项输出,
                        //如果直接限制积分输出项则e_sum 本身还在无限涨。
                        //误差反向时，巨大的 e_sum 要很久才泄放 → 过冲/恢复慢。
    float e_stable;
}PID_t;

/**
 * @brief  初始化一个 PID 控制器实例，把配置一次性写入实例。
 * @param  pid : 指向 PID 实例的指针（由调用者分配，本函数只填充内容）
 * @param  kp  : 比例增益        [单位: 输出/误差]
 * @param  ki  : 积分增益        [单位: 输出/(误差·秒)]
 * @param  kd  : 微分增益        [单位: 输出/(误差/秒)]
 * @param  max : 输出上限        [须与执行器匹配，如电机占空比 1.0]
 * @param  min : 输出下限        [如 -1.0]
 * @param  dt  : 采样时间间隔    [单位: 秒, 如 1kHz 对应 0.001f]
 * @param  i_max : 积分限幅      [用来通过限制e_sum来限制最终积分项输出,如果直接限制积分输出项则e_sum 本身还在无限涨。误差反向时，巨大的 e_sum 要很久才泄放 → 过冲/恢复慢。]
 * @param e_stable : 死区阈值    [|误差|<该值时 P/D 项归零、积分继续累积；填 0 关闭死区]
 * @retval PID_OK 成功；PID_ERR_NULL / PID_ERR_DT 等失败
 * @note   配置仅在初始化时设定；PID_Calculate 运行期间不再传增益或频率。
 *         本模块由直立环/速度环/转向环复用——每个环一个独立实例，
 *         各自用不同的采样周期初始化（直立 1ms、速度/转向 20ms）。
 */
PID_Status_t PID_Init(PID_t *pid, float kp, float ki, float kd, float max, float min, float dt, float i_max, float e_stable);

/**
 * @brief  根据目标值与当前值计算一次 PID 输出。
 * @param  pid     : 指向 PID 实例的指针
 * @param  target  : 目标值（与误差同单位）
 * @param  current : 当前值（与误差同单位）
 * @retval PID 输出，已夹在 [min, max] 内（含死区与抗积分饱和处理）
 * @note   调用前须先执行 PID_Init；本函数不修改配置。
 */
float PID_Calculate(PID_t *pid, float target, float current);

#endif