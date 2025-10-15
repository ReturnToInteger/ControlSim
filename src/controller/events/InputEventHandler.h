#pragma once
#include <string>
#include "InputEventPublisher.h"

namespace controller {
	namespace events {
		using controller::events::InputEvent;
		class InputEventHandler {
		public:
			~InputEventHandler() = default;
			virtual void handleInputEvent(std::string const& src, controller::events::InputEvent const& e) = 0;
		protected:
			InputEventHandler() = default;
		};
	}
}