#pragma once 

#include <SFML/Graphics.hpp>
#include <vector>
#include <unordered_map>
#include <deque>

#include "model/utils/Point.h"
#include "model/items/Cone.h"
#include "VehicleView.h"
#include "ConeView.h"
#include "PathView.h"
#include "model/items/Path.h"
#include "model/eventHandlers/InputEventPublisher.h"

namespace model {
	class Vehicle;
	class Item;
}

namespace view {
	class AppView : public model::InputEventPublisher {
	public:
		AppView();
		AppView(model::Vehicle const& vehicle, std::vector<model::Cone> const& map);
		~AppView() = default;
		AppView(AppView const&) = delete;
		void init();
		void render();
		void pollEvents();
		bool isOpen() const;
		double getFrameTime() const;
		void close();
		void setVehicle(model::Vehicle const& vehicle);
		void setCones(std::vector<model::Cone> const& cones);
		void setPath(model::Path const& path);
		void setPath(std::deque<model::Path> const& pathVector);
		void setConeDetectedFlag(std::vector<const model::Cone*> detectedCones);
		void setGridSize(double const& cellSize) { _cellSize = cellSize; }
		double getWidth() const { return _videoWidth; }
		double getHeight() const { return _videoHeight; }
		model::Point getClickGlobalPos() const { return _clickGlobalPos; }

	private:
		void _setupGrid();
		void _drawGrid();
		model::InputEvent _translateToInput(sf::Event e);
		sf::RenderWindow _window;
		int _videoWidth;
		int _videoHeight;
		int _frameRate;
		double _zoom;
		model::Point _startPos;
		model::Point _clickGlobalPos;
		double _cellSize;

		VehicleView _vehicleView;
		std::unordered_map<const model::Item*, ItemView*> _itemViewTable;
		std::vector<ConeView> _coneViews;

		std::vector<PathView> _pathViews;
		sf::View _view;
		sf::VertexArray _gridLines;
		//std::vector<sf::Drawable> _drawables;

	};
} // namespace view 