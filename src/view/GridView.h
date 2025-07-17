#pragma once
#include "ItemView.h"
#include <sfml/Graphics.hpp>
namespace view {
	class GridView : public ItemView {
	public:
		GridView(double const& cellSize) : m_cellSizePtr(&cellSize) {}
		// Inherited via ItemView
		void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
		sf::Vector2f getPosition() const override;
	private: 
		const double* m_cellSizePtr;
	};
}