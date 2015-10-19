#include "controllers/sonar/DevantechSonarInterface.h"
 
#include "geometry.h"
#include "i2c.h"

#include <iostream>

using namespace bjos;

bool DevantechSonarInterface::isActive(){
    if(!I2C::isStarted()) return false;
    unsigned char data;
    int ret_val = I2C::read(_address, 0, data, true);
    if(ret_val < 0) return false;
    //check for correct software revision
    return (data == 11);
}

double DevantechSonarInterface::getDistance(){
    //read distance
    unsigned int total = 0;
    unsigned char data;
    int ret_val;
    ret_val = I2C::read(_address, 2, data);
    if(ret_val < 0) return DEVANTECH_ERROR_RANGE;
    
    total = data;
    ret_val = I2C::read(_address, 3, data);
    if(ret_val < 0) return DEVANTECH_ERROR_RANGE;
    
    total <<= 8;
    total |= data;
    double range = DEVANTECH_MAX_RANGE;
    if(total != 0) range = total/100.0;
    return range;
}

bool DevantechSonarInterface::readDistance(){    
    //request for a distance read in cm
    int ret_val = I2C::write(_address, 0, 0x51);
    return (ret_val >= 0);
}

bool DevantechSonarInterface::globalReadDistance(){
    //request for a global distance read in cm
    int ret_val = I2C::write(0, 0, 0x51);
    return (ret_val >= 0);
}
