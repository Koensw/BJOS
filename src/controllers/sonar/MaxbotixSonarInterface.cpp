#include "controllers/sonar/MaxbotixSonarInterface.h"

#include "libs/log.h" 
#include "libs/i2c.h"

#include <iostream>

using namespace bjos;

bool MaxbotixSonarInterface::isActive(){
    //TODO: implement check
    return true;
}

double MaxbotixSonarInterface::getDistance(){
    //read distance
    unsigned int total = 0;
    unsigned short data;
    int ret_val;
    ret_val = I2C::read_word(_address, 0xE1, data);
    if(ret_val < 0) return MAXBOTIX_ERROR_RANGE;
    
    total = data;
    double range = MAXBOTIX_MAX_RANGE;
    if(total != 0) range = total/100.0;
    return range;
}

bool MaxbotixSonarInterface::readDistance(){    
    //request for a distance read in cm
    int ret_val = I2C::write(_address, 0, 0x51);
    return (ret_val >= 0);
}

bool MaxbotixSonarInterface::globalReadDistance(){
    //request for a global distance read in cm
    int ret_val = I2C::write(0, 0, 0x51);
    return (ret_val >= 0);
}
