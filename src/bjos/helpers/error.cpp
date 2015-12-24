#include "bjos/helpers/error.h"

#include "bjos/controller.h"

using namespace bjos;

ControllerInitializationError::ControllerInitializationError(Controller *controller, std::string message): _message(message) {
    _controller_name = controller->getControllerName();
}