#ifndef _BLUEJAY_MAXBOTIX_SONAR_INTERFACE_H_
#define _BLUEJAY_MAXBOTIX_SONAR_INTERFACE_H_

/* 
 * Maxbotix Sonar interface that works with the real sonars
 */

#include "SonarInterface.h"

namespace bjos{
    /* special devantech range values */
    const int MAXBOTIX_FOV = 1.04719755;
    const int MAXBOTIX_MIN_RANGE = 0.2;
    const int MAXBOTIX_MAX_RANGE = 7.5;

    const int MAXBOTIX_ERROR_RANGE = 10;

    //TODO: not threadsafe currently
    class MaxbotixSonarInterface : public SonarInterface{
    public:
        MaxbotixSonarInterface(unsigned char addr): _address(addr) {}
        
        /* Check if still active */
        bool isActive();
        
        /* Get the distance after read */
        double getDistance();

        /* Read from sonar  */
        bool readDistance();
        bool globalReadDistance();
        
        /* Getter for sonar info */
        double getFieldOfView(){
            return MAXBOTIX_FOV;
        }
        double getMinRange(){
            return MAXBOTIX_MIN_RANGE;
        }
        double getMaxRange(){
            return MAXBOTIX_MAX_RANGE;
        }
    private:
        unsigned char _address;
    };
}

#endif
