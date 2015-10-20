#ifndef _BLUEJAY_DEVANTECH_SONAR_INTERFACE_H_
#define _BLUEJAY_DEVANTECH_SONAR_INTERFACE_H_

/* 
 * Devantech Sonar interface that works with the real sonars
 */

#include "SonarInterface.h"

namespace bjos{
    /* special devantech range values */
    const int DEVANTECH_FOV = 1.04719755;
    const int DEVANTECH_MIN_RANGE = 0.03;
    const int DEVANTECH_MAX_RANGE = 5;

    const int DEVANTECH_ERROR_RANGE = 10;

    //TODO: not threadsafe currently
    class DevantechSonarInterface : public SonarInterface{
    public:
        DevantechSonarInterface(unsigned char addr): _address(addr) {}
        
        /* Check if still active */
        bool isActive();
        
        /* Get the distance after read */
        double getDistance();

        /* Read from sonar  */
        //FIXME: this should actually have been static but that is impossible for virtual members...
        bool readDistance();
        bool globalReadDistance();
        
        /* Getter for sonar info */
        double getFieldOfView(){
            return DEVANTECH_FOV;
        }
        double getMinRange(){
            return DEVANTECH_MIN_RANGE;
        }
        double getMaxRange(){
            return DEVANTECH_MAX_RANGE;
        }
    private:
        unsigned char _address;
    };
}

#endif
