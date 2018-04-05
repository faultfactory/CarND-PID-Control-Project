#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // set P gain
    this->Kp = Kp;

    // set I gain
    this->Ki = Ki;

    // set D gain
    this->Kd = Kd;
    }

void PID::UpdateError(double error) {
    //Add current error to i_error term to accumulate
    this->i_error+=error;

    //differential error calculated using p_error 
    this->d_error = error - this->p_error;
    
    // Cross track error is 
    this->p_error = error;
}

double PID::TotalError() {
    // This function title is awful.
    return Kp*p_error + Ki*i_error + Kd*d_error;

}

void PID::PrintError() {
    // Prints each terms contribution so i can drag and drop 
    std::cout<<"PID,"<<(Kp*p_error)<<","<<(Ki*i_error)<<","<<(Kd*d_error)<<std::endl;
}