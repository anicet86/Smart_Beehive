/*
 * pid.h
 *
 *  Created on: Nov 13, 2025
 *      Author: kemazhu
 */

#ifndef PID_H_
#define PID_H_
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint; //
    float prev_error;
    float integral;
} PID_Controller_t;

float PID_Compute(PID_Controller_t *pid, float measured_value);

#endif /* PID_H_ */
