#ifndef _BLUEJAY_PHYSICAL_SONAR_INTERFACE_H_
#define _BLUEJAY_PHYSICAL_SONAR_INTERFACE_H_

/* 
 * Base class for sonar interfaces that use real physical sonars
 */

//TODO: not threadsafe currently
namespace bjos{
    class SonarInterface{
    public:
        /* Check if still active */
        virtual bool isActive() = 0;
        
        /* Get the distance after read */
        virtual double getDistance() = 0;

        /* Read from sonar  */
        //FIXME: this should actually have been static but that is impossible for virtual members...
        virtual bool readDistance() = 0;
        virtual bool globalReadDistance() = 0;
        
        /* Getters for sonar info */
        virtual double getFieldOfView() = 0;
        virtual double getMinRange() = 0;
        virtual double getMaxRange() = 0;
        
        /* Virtual destructor */
        virtual ~SonarInterface() {}
    };
}


#endif