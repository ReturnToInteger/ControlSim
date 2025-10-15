#pragma once 

#include <SFML/Graphics.hpp>
#include <vector>
#include <unordered_map>
#include <deque>


#include "model/utils/Point.h"
#include "model/items/Cone.h"
#include "model/items/Path.h"
#include "VehicleView.h"
#include "ConeView.h"
#include "PathView.h"
#include "controller/events/InputEventPublisher.h"
#include "model/utils/IDrawableVehicle.h"


namespace model {
	class Item;
}

namespace view {
	struct DefaultAppViewConfig {
		static constexpr unsigned int width = 1600;
		static constexpr unsigned int height = 900;
		static constexpr int frameRate = 144;
		static constexpr double zoom = 1.0 / 8.0;
	};
	class AppView : public controller::events::InputEventPublisher {
	public:
		AppView();
		AppView(model::IDrawableVehicle const& vehicle, std::vector<model::Cone> const& map);
		~AppView() = default;
		AppView(AppView const&) = delete;
		void init();
		void render();
		void pollEvents();
		bool isOpen() const;
		double getFrameTime() const;
		void close();
		void setVehicle(model::IDrawableVehicle const& vehicle);
		void setCones(std::vector<model::Cone> const& cones);
		void setPath(model::Path const& path);
		void setPath(std::deque<model::Path> const& pathVector);
		void setConeDetectedFlag(const std::unordered_set<const model::Cone*>& detectedCones);
		void setGridSize(double const& cellSize) { m_cellSize = cellSize; }
		double getWidth() const { return m_videoWidth; }
		double getHeight() const { return m_videoHeight; }
		model::Point getClickGlobalPos() const { return m_clickGlobalPos; }
		void zoom(double factor);
		void rotate(double delta);

		static constexpr double defaultGridZoomLimit = 10.0;

	private:
		void setupGrid(double limit);
		void drawGrid();
		controller::events::InputEvent translateEventToInput(sf::Event event);
		sf::RenderWindow m_window;
		unsigned int m_videoWidth;
		unsigned int m_videoHeight;
		int m_frameRate;
		double m_zoom;
		double m_lastX;
		double m_gridZoomLimit;

		model::Point m_startPos;
		model::Point m_clickGlobalPos;
		double m_cellSize;

		VehicleView m_vehicleView;
		std::unordered_map<const model::Item*, ItemView*> m_itemViewTable;
		std::vector<ConeView> m_coneViews;

		std::vector<PathView> m_pathViews;
		sf::View m_view;
		sf::VertexArray m_gridLines;
		//std::vector<sf::Drawable> m_drawables;

	};
} // namespace view 