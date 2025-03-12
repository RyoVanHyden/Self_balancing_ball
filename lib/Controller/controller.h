#include <WString.h>
class Controller {

private:
    
    bool P, I, D;
    Derivative derivative;
    Integral integral;
    float Kc, Ti, Td;
    float lower_limit, upper_limit;

public:
    Controller(String name, bool P, bool I, bool D, float upper_limit, float lower_limit);
    String name;

    void setProportional(float Kc);
    void setIntegral(float Ti);
    void setDerivative(float Td);
    
    float computeOutput(float input);
    void resetController();

};

class Integral{
    
private:
    float step_size;
    float sum, output;   


public:
    String name;
    Integral(float step_size, String name);
    Integral();
    float computeOutput(float input);
    void updateSum();
    void resetSum();

};

class Derivative{

private:
    float step_size, last_input;
    float output;

public:
    String name;
    Derivative(float step_size, String name);
    Derivative();
    float computeOutput(float input);
    void setStepSize(float step_size);
    void resetDerivative();

};