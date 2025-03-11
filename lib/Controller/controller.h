#include <WString.h>
class Controller {

private:
    
    bool P, I, D;
    Derivative derivative;
    Integral integral;
    float Kc, Ti, Td;

public:
    Controller(String name, bool P, bool I, bool D);
    String name;

    void setPGain(float Kc);
    void setIGain(float Ti);
    void setDGain(float Td);
    
    float computeOutput(float input);
    void resetController();

};

class Integral{
    
private:
    float step_size, upper_limit, lower_limit;
    float sum, output;   


public:
    String name;
    Integral(float step_size, float upper_limit, float lower_limit, String name);
    Integral();
    float computeOutput(float input);
    void setLimits(float upper_limit, float lower_limit);

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

};