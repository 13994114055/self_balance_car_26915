#ifndef __MAHONY_AHRS_H
#define __MAHONY_AHRS_H

typedef struct {
    float q[4];
    float kp ;
    float ki ;
    float ax ;
    float ay ;
    float az ;
    float integralFB[3] ;

}MahonyAHRS_t ;

typedef struct {
    float yaw ;
    float pitch ;
    float roll ;
}EulerAngles_t ;

void MahonyAHRS_Init(MahonyAHRS_t *mah , float kp , float ki ) ;

void MahonyAHRS_Update(MahonyAHRS_t *mah) ;

void MahonyAHRS_GetEulerAngles(MahonyAHRS_t *mah , EulerAngles_t *e);

#endif