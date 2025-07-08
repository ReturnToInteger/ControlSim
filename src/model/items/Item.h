#pragma once
#include "model/utils/Point.h"
namespace model {
	class Item
	{
	public:
		Item() = default;
		Item(Item const&) = default;
		Item& operator=(Item const&) = default;
		Item(Item&& other) = default;
		Item& operator=(Item&& other) = default;
		virtual ~Item() = default;
		virtual model::Point getPosition() const = 0;
	protected:

	};
}