#include "Car_control.h"

#include "math.h"
#include <map>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <random>
using namespace std;
/**
* Created by: TRAN Anh Tuan 2019/09/06
*/

/* =====================================================================
 * Class Low-Pass Filter
 * =====================================================================
 */
// Constructor
LowPassFilter::LowPassFilter(double _input, double _Ts, double _dt) : input(_input), Ts(_Ts), dt(_dt), output(0) {};

//Setter
void LowPassFilter::set_Ts(double _Ts) {
    this->Ts = _Ts;
}
void LowPassFilter::set_dt(double _dt) {
    this->dt = _dt;
}
void LowPassFilter::set_input(double _input) {
    this->input = _input;
}
void LowPassFilter::set_init(double _output) {
    this->output = _output;
}
void LowPassFilter::calculate() {
    this->output = output + dt * (-1/Ts*output + 1/Ts*input);
}

//Getter
double LowPassFilter::get_output() const {
    return output;
}

// Print
void LowPassFilter::print() const {
    std::cout << "Signal output = "<<std::fixed<<std::setprecision(2)<<output<<std::endl;
}

/* =====================================================================
 * Class Derivative Filter
 * =====================================================================
 */
// Constructor
DerivativeFilter::DerivativeFilter(double _input, double _Ts, double _dt) : input(_input), Ts(_Ts), dt(_dt), output(0.0) {};

//Setter
void DerivativeFilter::set_Ts(double _Ts) {
    this->Ts = _Ts;
}
void DerivativeFilter::set_dt(double _dt) {
    this->dt = _dt;
}
void DerivativeFilter::set_input(double _input) {
    this->input = _input;
}

void DerivativeFilter::calculate(double &x) {
    this->output = -1/Ts * x + 1/Ts * input;	// Calculate derivative
    x += dt * (-1/Ts * x + 1/Ts * input);		// Update x
}

//Getter
double DerivativeFilter::get_output() const {
    return output;
}

// Print
void DerivativeFilter::print() const {
    std::cout << "Signal output = "<<std::fixed<<std::setprecision(2)<<output<<std::endl;
}


/* =====================================================================
 * Class PID
 * =====================================================================
 */
// Constructor
PID::PID (double _kp, double _ki, double _kd, double _error_integration_max, double _error_integration_min) : Kp(_kp), Ki(_ki), Kd(_kd), error(0.0), error_integration(0.0), error_derivative(0.0), error_integration_max(_error_integration_max), error_integration_min(_error_integration_min) {};

// Getter
double PID::get_Kp() const {
    return Kp;
};
double PID::get_Ki() const {
    return Ki;
};
double PID::get_Kd() const {
    return Kd;
};
double PID::get_error() const {
    return error;
};
double PID::get_error_integration() const {
    return error_integration;
};
double PID::get_error_derivative() const {
    return error_derivative;
};

// Reset
void PID::reset_error_integration() {
    this->error_integration=0.0;
}

// Setter
void PID::set_Kp(double _kp) {
    this->Kp = _kp;
};
void PID::set_Ki(double _ki) {
    this->Ki = _ki;
};
void PID::set_Kd(double _kd) {
    this->Kd = _kd;
};
void PID::set_Kp_variant (double Kp_max, double Kp_min, double error_small, double error_large) {
    // Variable Kp gain: Kp = a * speed_error + b;
    // Kp = Kp_max when speed_error is small
    // Kp = Kp_min when speed_error is large
	double abs_error = fabs(error);
    if (abs_error >= error_large) {
        this->Kp = Kp_min;
    }
    else if (abs_error  <= error_small) {
        this->Kp = Kp_max;
    }
    else {
        this->Kp =  Kp_min * (abs_error - error_small) / (error_large - error_small) + Kp_max * (error_large - abs_error) / (error_large - error_small);
    }
}

// Constant Kp
double PID::calculate(double output, double setpoint, double &x, double dt, int flag) {

    error = setpoint - output;

    DerivativeFilter error_dot(error);
    error_dot.calculate(x);
    error_derivative = error_dot.get_output();

    // Integrator
    error_integration += flag * error * dt;
	
    //Anti-windup
    error_integration = min(max(error_integration,error_integration_min),error_integration_max);

    return Kp * error + Ki*error_integration + Kd * error_derivative;
}

// Variable Kp
double PID::calculate_Kp_variant(double output, double setpoint, double &x, double Kp_max, double Kp_min, double error_small, double error_large, double dt, int flag) {

    error = setpoint - output;

    // Calculate Kp
    set_Kp_variant(Kp_max, Kp_min, error_small, error_large);

    DerivativeFilter error_dot(error);
    error_dot.calculate(x);
    error_derivative = error_dot.get_output();

    // Integrator
    error_integration += flag * error * dt;
    error_integration = min(max(error_integration,error_integration_min),error_integration_max);	//Anti-windup

    // PID calculation
    return Kp * error + Ki*error_integration + Kd * error_derivative;
}

/* =====================================================================
 * Create relay signal for system identification
 * =====================================================================
 */

void additive_relay_signal(double &speed_relay, double magnitude, int &sign, int frequency, int &loop_relay) {
	loop_relay += 1;
	if (loop_relay % frequency == 0) { 
		sign = - sign; 
		loop_relay = 0;
		speed_relay = magnitude;
        // fprintf(stdout,"===============================\n");
		// fprintf(stdout,"Relay frequency = %d[ms]\n",frequency*10);
        // fprintf(stdout,"Relay signal = %.1f [PWM]\n",magnitude );

	}
	
}




/* =====================================================================
 * Lane keeping State Feedback controller
 * Created by: H. Okuda
 * =====================================================================
 */
StateFBControlParameter::StateFBControlParameter()
{
    PFLLook = 10;
    PFSpeed = 10;
    PFKspeed = -2.0;
    PFKangle = -1.0;

    SFBK1 = 1000;
    SFBK2 = 100;
    SFBK3 = 1000;
    SFBK4 = 50;
    
        // SFBK1 = 19.7;
    // SFBK2 = 641.5;
    // SFBK3 = 960.9;
    // SFBK4 = 21.7;

    SFBLLook = 0.6;
};

void StateFBControlParameter::ReadFromMap(std::map<std::string, double> param)
{
    TryGetVal(param, "Speed", PFSpeed);
    TryGetVal(param, "PFkspeed", PFKspeed);
    TryGetVal(param, "PFkangle", PFKangle);
    TryGetVal(param, "PFLLook", PFLLook);

    TryGetVal(param, "k1", SFBK1);
    TryGetVal(param, "k2", SFBK2);
    TryGetVal(param, "k3", SFBK3);
    TryGetVal(param, "k4", SFBK4);

    TryGetVal(param, "LLook", SFBLLook);
};

void StateFBControlParameter::TryGetVal(std::map<std::string, double> &param, std::string key, double &val)
{
    if (param.count(key) == 1)
    {
        val = param[key];
    }
    else {

    }
};

