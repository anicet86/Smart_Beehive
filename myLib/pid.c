/*
 * pid.c
 *
 *  Created on: Nov 13, 2025
 *      Author: kemazhu
 */

#include "pid.h"
#include "dht22.h"
#include "timer3.h"


#include "pid.h"

float PID_Compute(PID_Controller_t *pid, float measured)
{
    float error =  measured - pid->setpoint ; //float error = pid->setpoint - measured;
    pid->integral += error;
    //float derivative = error - pid->prev_error; //  ancien code
    if (pid->integral > 100.0f) pid->integral = 100.0f; // avec anti windup
    if (pid->integral < -100.0f) pid->integral = -100.0f; //  avec anti windup

    float derivative = error - pid->prev_error;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    // Saturation optionnelle (ex: 0 à 100%)
    if (output > 100.0f) output = 100.0f;
    if (output < 0.0f) output = 0.0f;

    pid->prev_error = error;
    return output;
}
