#include "controller.h"

Integral::Integral(float step_size, float upper_limit, float lower_limit, String name){
    this->step_size = step_size;
    this->upper_limit = upper_limit;
    this->lower_limit = lower_limit;
    this->name = name;
    this->sum = 0;
    this->output = 0;
}

Integral::Integral(){
    this->step_size = 0;
    this->upper_limit = 0;
    this->lower_limit = 0;
    this->name = "";
    this->sum = 0;
    this->output = 0;
}

float Integral::computeOutput(float input){

    output = sum + input*step_size;

    if (output > upper_limit){
        output = upper_limit;
    } else if (output < lower_limit){
        output = lower_limit;
    }

    sum = output;
    return output;
}


Derivative::Derivative(float step_size, String name){
    this->name = name;
    this->step_size = step_size;
    this->output = 0;
    this->last_input = 0;
}

Derivative::Derivative(){
    this->name = " ";
    this->step_size = 0;
    this->output = 0;
    this->last_input = 0;
}

float Derivative::computeOutput(float input){

    output = (input - last_input)/step_size;
    last_input = input;

    return output;
}

Controller::Controller(String name, bool P, bool I, bool D){
    this->name = name;
    this->P = P;
    this->I = I;
    this->D = D;
    this->integral = Integral(0,0,0,"integral");
    this->derivative = Derivative(0,"derivative");
    this->Kc = 0;
    this->Ti = 0;
    this->Td = 0;

}