#pragma once
#include "Point.h"
namespace model {
	class Item
	{
	public:
		Item() = default;
		Item(const Item&) = default;
		Item& operator=(const Item&) = default;
		Item(Item&& other) = default;
		Item& operator=(Item&& other) = default;
		virtual ~Item() = default;
		virtual Point getPosition() const = 0;
	protected:

	};
}