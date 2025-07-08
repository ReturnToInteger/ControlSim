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
#include "model/events/InputEventPublisher.h"
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
	class AppView : public model::events::InputEventPublisher {
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
		void setGridSize(double const& cellSize) { _cellSize = cellSize; }
		double getWidth() const { return _videoWidth; }
		double getHeight() const { return _videoHeight; }
		model::Point getClickGlobalPos() const { return _clickGlobalPos; }
		void zoom(double factor);
		void rotate(double delta);

		static constexpr double defaultGridZoomLimit = 10.0;

	private:
		void _setupGrid(double limit);
		void _drawGrid();
		model::events::InputEvent _translateEventToInput(sf::Event event);
		sf::RenderWindow _window;
		unsigned int _videoWidth;
		unsigned int _videoHeight;
		int _frameRate;
		double _zoom;
		double _lastX;
		double _gridZoomLimit;

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