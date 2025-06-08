#pragma once
#include <string>
#include "InputEventPublisher.h"

namespace model {
	namespace events {
		using model::events::InputEvent;
		class InputEventHandler {
		public:
			~InputEventHandler() = default;
			virtual void handleInputEvent(std::string const& src, model::events::InputEvent const& e) = 0;
		protected:
			InputEventHandler() = default;
		};
	}
}