#include "PID.h"
#include "iostream"
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    
    this->best_Kp = Kp;
    this->best_Ki = Ki;
    this->best_Kd = Kd;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    total_error += cte*cte;
}

double PID::TotalError() {
     return (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);
}

bool PID::should_restart_twiddle_iteration() {
    if (!is_twiddling) {
        return false;
    }
    if (dp[0] + dp[1] +dp[2] < tolerace) { // tolerance
        return false;
    }
    if (iteration_for_twiddle % 4000 == 0) {
        return true;
    }
    return false;
}

void PID::twiddle() {
    if (best_error == 0.0) {
        best_error = total_error;
    }
    if (total_error <= best_error)
    {
        best_error = total_error;
        dp[twiddling_index] *= 1.1;
        best_Kp = Kp;
        best_Ki = Ki;
        best_Kd = Kd;
        twiddling_index = (twiddling_index + 1) % 3;
        updateIndex(twiddling_index, dp[twiddling_index]);
        twiddle_state = 0;
    } else {
        if (twiddle_state == 0) {
            updateIndex(twiddling_index, dp[twiddling_index] * (-2));
            twiddle_state = 1;
        } else if (twiddle_state == 1) {
            dp[twiddling_index] *= 0.9;
            twiddle_state = 0;
            Kp = best_Kp;
            Ki = best_Ki;
            Kd = best_Kd;
            twiddling_index = (twiddling_index + 1) % 3;
            updateIndex(twiddling_index, dp[twiddling_index]);
        }
    }
    total_error = 0.0;
}

void PID::updateIndex(int index, double value){
    switch (index) {
        case 0:
            Kp += value;
            break;
        case 1:
            Ki += value;
            break;
        case 2:
            Kd += value;
            break;
        default:
            break;
    }
}
