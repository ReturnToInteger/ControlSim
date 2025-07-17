#include "InputEventPublisher.h"
#include "InputEventHandler.h"


void model::events::InputEventPublisher::notify(std::string const& src, model::events::InputEvent const& e) {
	for (auto it = m_eventHandlers.begin(); it != m_eventHandlers.end(); ) {
		if (*it) {
			(*it)->handleInputEvent(src, e);
			++it;
		}
		else {
			std::cout << "Event not found.\n";
			it = m_eventHandlers.erase(it);  // erase returns the next iterator
		}
	}
}
