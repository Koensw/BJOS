#ifndef _BLUEJAY_BJOS_ERROR
#define _BLUEJAY_BJOS_ERROR

#include <exception>
#include <string>

/*
 * Error clases to throw if severe errors occur (replace fatal_error)
 */

namespace bjos{
    class Controller;
    
    /* Error initializing an instance */
    class ControllerInitializationError : public std::exception{
    public:
        ControllerInitializationError(Controller *controller, std::string message);
        
        const char *what(){
            return _message.c_str();
        }
        
        const char *getControllerName(){
            return _controller_name.c_str();
        }
    private:
        std::string _message;
        
        std::string _controller_name;
    };
    
    class BJOSError : public std::exception{
    public:
        BJOSError(std::string message): _message(message) { }
        
        const char *what(){
            return _message.c_str();
        }
    private:
        std::string _message;
    };
}
    
#endif