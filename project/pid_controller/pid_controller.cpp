/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   // set Kd
   this->Kd = Kd;
   // set Kp
   this->Kp = Kp;
   // set Ki
   this->Ki = Ki;
   
   // set output_lim_min
   this->output_lim_min = output_lim_min;
   // set output_lim_max
   this->output_lim_max = output_lim_max;
   
   // set cte_previous
   this->cte_previous = 0;
   // set I_Value
   this->I_Value = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
   if(abs(delta_time) < MIN_DELTA_TIME) return;
   double P_Value = this->Kp * cte;
   this->I_Value += this->Ki * cte * this->delta_time;
   double D_Value = this->Kd * (cte - this->cte_previous) / this->delta_time;
   this->action_value = P_Value + this->I_Value + D_Value;
   this->cte_previous = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double control_value = this->action_value;
   if (control_value < this->output_lim_min) control_value = this->output_lim_min;
   if (control_value > this->output_lim_max) control_value = this->output_lim_max;
   return control_value;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   this->delta_time = new_delta_time;
   return this->delta_time;
}