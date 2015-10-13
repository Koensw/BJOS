#ifndef _BLUEJAY_BJOS_PROCESS
#define _BLUEJAY_BJOS_PROCESS

#include <atomic>

/*
 * Utility class for making a process signal control available
 */

namespace bjos{
    class Process{
    public:
        /* Get the BJOS signal handler to call it if another signal handler is defined */
        static void (*getSignalHandler())(int);
        /* Install the signal handler */
        static void installSignalHandler();
        
        /* Sets the active state of the process to false */
        static void stop();
        
        /* Return if the process is and the BJOS are active */
        static bool isActive();
    private:
        static void signal_handler(int);
        static std::atomic_bool _active;
        
        Process();
        ~Process();
    };
}
#endif