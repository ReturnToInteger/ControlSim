#include "AppView.h" 
#include <cassert>
#include "model/items/Item.h"
#include "model/utils/IDrawableVehicle.h"
#include "VehicleView.h"

//#include "imgui.h"
//#include "imgui-sfml.h"
#ifdef ENABLE_DEBUG_DRAW
#include "DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW


namespace view {
	AppView::AppView()
		: _videoWidth(DefaultAppViewConfig::width),
		_videoHeight(DefaultAppViewConfig::height),
		_frameRate(DefaultAppViewConfig::frameRate),
		_zoom(DefaultAppViewConfig::zoom),
		_lastX(-1),
		_gridZoomLimit(defaultGridZoomLimit),
		_cellSize(0)
	{
	}

	AppView::AppView(model::IDrawableVehicle const& vehicle, std::vector<model::Cone> const& map)
		: _videoWidth(DefaultAppViewConfig::width),
		_videoHeight(DefaultAppViewConfig::height),
		_frameRate(DefaultAppViewConfig::frameRate),
		_zoom(DefaultAppViewConfig::zoom),
		_lastX(-1),
		_vehicleView(vehicle, sf::Color::Red),
		_gridZoomLimit(defaultGridZoomLimit),
		_cellSize(0)
	{
		_coneViews.reserve(map.size());
		for (const auto& cone : map) {
			_coneViews.emplace_back(cone);
			_itemViewTable.emplace(&cone, &_coneViews.back());
		}
	}

	void AppView::init()
	{
		_window.create(sf::VideoMode(_videoWidth, _videoHeight), "Vehicle Simulation");
		_window.setFramerateLimit(_frameRate);
		_view.setSize(static_cast<float>(_videoWidth), static_cast<float>(_videoHeight));
		_view.setCenter(_vehicleView.getPosition());
		_view.zoom(static_cast<float>(_zoom));
		_window.setView(_view);
	}

	void AppView::render()
	{
		_setupGrid(_gridZoomLimit);
		_window.clear(sf::Color::Black);
		_view.setCenter(_vehicleView.getPosition());
		_window.setView(_view);
		//Drawing
		_drawGrid();
		#ifdef ENABLE_DEBUG_DRAW
		_window.draw(view::DebugDraw::instance());
		#endif // ENABLE_DEBUG_DRAW

		for (auto const& coneView : _coneViews) {
			_window.draw(coneView);
		}
		for (auto& path : _pathViews) {
			_window.draw(path);
		}
		_window.draw(_vehicleView);
		//if (_drawables) {
		//	for (auto const& drawable : _drawables) {
		//		_window.draw(drawable);
		//	}
		//}

		_window.display();
	}

	void AppView::pollEvents()
	{
		sf::Event event;
		while (_window.pollEvent(event)){
			if (event.type == sf::Event::Closed) 
			{
				_window.close();
			}
			else if (event.type == sf::Event::Resized)
			{
				// update the view to the new size of the window
				auto resized = event.size;
				_videoWidth = resized.width;
				_videoHeight = resized.height;
				_view.setSize(static_cast<float>(resized.width), static_cast<float>(resized.height));
				_view.setCenter(_vehicleView.getPosition());
				_view.zoom(static_cast<float>(_zoom));
				
				_notify("AppView", model::events::Resized{});
			} 
			else
			{
				model::events::InputEvent e = _translateEventToInput(event);
				if (!std::holds_alternative<model::events::None>(e))
				{
					_notify("AppView", e);
				}
			}

		}

		// Poll keys not in event
		if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)) 
		{
			sf::Vector2i pixelPos = sf::Mouse::getPosition(_window);
			if (_lastX < 0) 
			{
				_lastX = pixelPos.x;
			}

			_notify("AppView", model::events::RightClickDown{ .lastX= _lastX,.currentX= (double)pixelPos.x });
			_lastX = pixelPos.x;
		}
		else 
		{
			_lastX = -1;
		}
	}

	bool AppView::isOpen() const
	{
		return _window.isOpen();
	}

	double AppView::getFrameTime() const
	{
		return 1.0/(double) _frameRate;
	}

	void AppView::close()
	{
		_window.close();
	}

	void AppView::setVehicle(model::IDrawableVehicle const& vehicle)
	{
		_vehicleView = view::VehicleView(vehicle, sf::Color::Red);
		_view.setCenter(_vehicleView.getPosition());
	}

	void AppView::setCones(std::vector<model::Cone> const& cones)
	{
		_coneViews.clear();
		_coneViews.reserve(cones.size());
		for (auto const& cone : cones) {
			_coneViews.emplace_back(cone);	
			_itemViewTable.emplace(&cone, &_coneViews.back());
		}
	}

	void AppView::setPath(model::Path const& path)
	{
		_pathViews = std::vector<PathView>{ PathView(path) };

	}

	void AppView::setPath(std::deque<model::Path> const& pathVector)
	{
		// TODO: Delete from itemViewTable
		_pathViews.clear();
		if (pathVector.empty()) {
			_pathViews.emplace_back();
			return;
		}
		for (auto const& path : pathVector) {
			_pathViews.emplace_back(path);
			//_itemViewTable.emplace(&path, _pathViews.back());
		}
		_pathViews.back().setColor(sf::Color::Green);

	}

	void AppView::setConeDetectedFlag(const std::unordered_set<const model::Cone*>& detectedCones)
	{
		for (auto& coneView : _coneViews) {
			coneView.isDetected = false;
		}
		for (auto const& item : detectedCones) {
			_itemViewTable[item]->isDetected = true;
		}
	}

	void AppView::zoom(double factor)
	{
		_zoom = _zoom * factor;
		//std::cout << delta << "\n";
		_view.zoom(static_cast<float>(factor));

	}

	void AppView::rotate(double delta)
	{
		_view.rotate(static_cast<float>(delta));
	}

	void AppView::_setupGrid(double limit)
	{
		sf::Vector2f topLeft(_window.mapPixelToCoords({ 0,0 }));
		sf::Vector2f bottomRight(_window.mapPixelToCoords(sf::Vector2i( (int)_videoWidth,(int)_videoHeight )));
		if (_cellSize / _zoom < limit) {
			_gridLines.clear();
			return;
		}
		double cellSize = _cellSize;

		int startX = int(std::floor(topLeft.x / cellSize));
		int endX = int((std::floor(bottomRight.x / cellSize) + 1));
		int startY = int(std::floor(topLeft.y / cellSize));
		int endY = int((std::floor(bottomRight.y / cellSize) + 1));

		sf::VertexArray lines(sf::Lines);

		sf::Color lineColor(80, 80, 80); // subtle gray, TO DO: gridView class


		// vertical lines
		for (int x = startX; x <= endX; ++x) {
			auto px = static_cast<float>(x * cellSize);
			lines.append(sf::Vertex(sf::Vector2f(px, static_cast<float>(startY * cellSize)), lineColor));
			lines.append(sf::Vertex(sf::Vector2f(px, static_cast<float>(endY * cellSize)), lineColor));
		}

		// horizontal lines
		for (int y = startY; y <= endY; ++y) {
			auto py = static_cast<float>(y * cellSize);
			lines.append(sf::Vertex(sf::Vector2f(static_cast<float>(startX * cellSize), py), lineColor));
			lines.append(sf::Vertex(sf::Vector2f(static_cast<float>(endX * cellSize), py), lineColor));
		}
		_gridLines = lines;
	}

	void AppView::_drawGrid()
	{
		_window.draw(_gridLines);
	}

	model::events::InputEvent view::AppView::_translateEventToInput(sf::Event event)
	{

		if (event.type == sf::Event::KeyPressed) 
		{
			if (event.key.code == sf::Keyboard::Escape)
			{
				return model::events::PressedEsc();
			}
				//_window.close();
			if (event.key.code == sf::Keyboard::LShift) {
				//// Wait until released
				//while (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)) {
				//	// Do nothing
				//}
				return model::events::PressedLShift();
			}
		}
		if (event.type == sf::Event::MouseButtonPressed)
		{
			if (event.mouseButton.button == sf::Mouse::Left) {
				sf::Vector2i pixelPos = sf::Mouse::getPosition(_window);

				sf::Vector2f worldPos = _window.mapPixelToCoords(pixelPos);

				//std::cout << "Map coords (on button press):\n";
				//std::cout << "map x: " << worldPos.x << "\n";
				//std::cout << "map y: " << worldPos.y << "\n";
				//_clickGlobalPos = model::Point(worldPos.x, worldPos.y);
				return model::events::ClickedAt{ .x = (double)worldPos.x, .y = (double)worldPos.y };
			}
		}
		if (event.type == sf::Event::MouseWheelMoved)
		{
			int delta =event.mouseWheel.delta;
			return model::events::Scrolled{ delta };
		}
		if (event.type == sf::Event::Resized) {
			return model::events::Resized();
		}		
		if (event.type == sf::Event::LostFocus) {
			return model::events::LostFocus();
		}
		if (event.type == sf::Event::GainedFocus) {
			return model::events::GainedFocus();
		}
		return model::events::None();
	}

//	void AppView::addDrawable(sf::Drawable& drawable)
//	{
//		_drawables.emplace_back(drawable);
//	}
//
//	void AppView::clearDrawables()
//	{
//		_drawables.clear();
//	}
//
}

