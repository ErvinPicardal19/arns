#ifndef ARNS_HARDWARE_WHEEL_HPP
#define ARNS_HARDWARE_WHEEL_HPP

#include <string>

class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0; /*command velocity*/
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Wheel();

    Wheel(const std::string &wheel_name, int counts_per_rev);

    
    void setup(const std::string &wheel_name, int counts_per_rev);

    double calc_enc_angle();



};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP