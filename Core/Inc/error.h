//
// Created by user on 2026/10/3.
//

#ifndef __ERROR_H
#define __ERROR_H

typedef enum {
    ERR_NONE = 0,
    ERR_NULL_PTR,     /* 传入了空指针 */
    ERR_BAD_DT,       /* 采样周期非法 (<=0) */
    ERR_PID_INIT,     /* PID 初始化失败 */
} ErrorCode_t;


void error(ErrorCode_t errorCode) ;

#endif //_ERROR_H
