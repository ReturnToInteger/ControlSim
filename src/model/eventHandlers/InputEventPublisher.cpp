#include "InputEventPublisher.h"
#include "InputEventHandler.h"

// Prevent 
void model::InputEventPublisher::_notify(std::string const& src, InputEvent const& e) {
	for (auto it = _eventHandlers.begin(); it != _eventHandlers.end(); ) {
		if (*it) {
			(*it)->handleInputEvent(src, e);
			++it;
		}
		else {
			std::cout << "Event not found." << std::endl;
			it = _eventHandlers.erase(it);  // erase returns the next iterator
		}
	}
}
