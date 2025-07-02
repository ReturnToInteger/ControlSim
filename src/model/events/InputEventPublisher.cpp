#include "InputEventPublisher.h"
#include "InputEventHandler.h"


void model::events::InputEventPublisher::_notify(std::string const& src, model::events::InputEvent const& e) {
	for (auto it = _eventHandlers.begin(); it != _eventHandlers.end(); ) {
		if (*it) {
			(*it)->handleInputEvent(src, e);
			++it;
		}
		else {
			std::cout << "Event not found.\n";
			it = _eventHandlers.erase(it);  // erase returns the next iterator
		}
	}
}
