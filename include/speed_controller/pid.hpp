#include <stdint.h>

class Pid
{
private:
    float target_value;
    float integral;
    float p_factor;
    float i_factor;
    float d_factor;
    float previous_error;
public:
    Pid(float _pf, float _if, float _df);
    void set_proportion_factor(float);
    void set_integral_factor(float);
    void set_differentation_factor(float);
    void set_target_value(float);
    void set_target_value_with_default(float, float);
    int8_t calculate(float, float);
    float get_target_value() const;
};
