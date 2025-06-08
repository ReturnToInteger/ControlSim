#pragma once
#include <string>
#include <variant>
#include <unordered_set>
#include <iostream>

namespace model {
	namespace events {
		struct PressedEsc {};
		struct PressedLShift {};
		struct Scrolled { int delta; };
		struct ClickedAt { double x, y; };
		struct None {};
		struct Resized {};
		struct LostFocus {};
		struct GainedFocus {};
		using InputEvent = std::variant<PressedEsc, PressedLShift, Scrolled, ClickedAt, Resized, LostFocus, GainedFocus, None>;

		class InputEventHandler;

		class InputEventPublisher {
		public:
			virtual ~InputEventPublisher() = default;
			virtual void attach(model::events::InputEventHandler* handler) { _eventHandlers.emplace(handler); }
			virtual void detach(model::events::InputEventHandler* handler) { _eventHandlers.erase(handler); }
		protected:
			InputEventPublisher() : _eventHandlers({}) {}; // Prevent instantiation
			virtual void _notify(std::string const& src, InputEvent const& e);
			std::unordered_set<model::events::InputEventHandler*> _eventHandlers;


		};
	}
}