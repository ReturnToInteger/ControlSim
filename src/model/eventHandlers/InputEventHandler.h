#pragma once
#include <string>
#include "InputEventPublisher.h"

namespace model {
	using model::InputEvent;
	class InputEventHandler {
	public:
		~InputEventHandler() = default;
		virtual void handleInputEvent(std::string const& src, model::InputEvent const& e) = 0;
	protected:
		InputEventHandler() = default;
	};
}