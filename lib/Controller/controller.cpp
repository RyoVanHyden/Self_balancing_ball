#include "controller.h"

// Integral ------------------------------------------------------------

Integral::Integral(float step_size, String name){
    this->step_size = step_size;

    this->name = name;
    this->sum = 0.0;
    this->output = 0.0;
}

Integral::Integral(){
    this->step_size = 0.0;
    this->name = "";
    this->sum = 0.0;
    this->output = 0.0;
}

float Integral::computeOutput(float input){
    output = sum + input*step_size;
    return output;
}

void Integral::updateSum(){
    sum = output;
}

void Integral::resetSum(){
    sum = 0;
    output = 0;
}

// Derivative ----------------------------------------------------------

Derivative::Derivative(float step_size, String name){
    this->name = name;
    this->step_size = step_size;
    this->output = 0.0;
    this->last_input = 0.0;
}

Derivative::Derivative(){
    this->name = " ";
    this->step_size = 0.0;
    this->output = 0.0;
    this->last_input = 0.0;
}

float Derivative::computeOutput(float input){

    output = (input - last_input)/step_size;
    last_input = input;

    return output;
}

void Derivative::resetDerivative(){
    last_input = 0;
    output = 0;
}

// Controller ----------------------------------------------------------

Controller::Controller(String name, bool P, bool I, bool D){
    this->name = name;
    this->P = P;
    this->I = I;
    this->D = D;
    this->integral = Integral(0,"integral");
    this->derivative = Derivative(0,"derivative");
    this->Kc = 0;
    this->Ti = 0;
    this->Td = 0;
}

void Controller::setProportional(float Kc){
    if (P) this->Kc = Kc;
}

void Controller::setIntegral(float Ti){
    if(I) this->Ti = Ti;
}

void Controller::setDerivative(float Td){
    if(D) this->Td = Td;
}

void Controller::resetController(){
    if (I) integral.resetSum();
    if (D) derivative.resetDerivative();
}

float Controller::computeOutput(float input){
    float p = 0.0;
    float i = 0.0;
    float d = 0.0;

    if (P) p = input*Kc;
    if (I) i = integral.computeOutput(input);
    if (D) d = derivative.computeOutput(input);

    float output = p + i + d;

    //Anti-windup
    if (output > upper_limit) {
        output = upper_limit;
    } else if (output < lower_limit){
        output = lower_limit;
    } else {
        if (I) integral.updateSum();
    }

    return (p + i + d);
}