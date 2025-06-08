#include "AppView.h" 
#include <cassert>
#include "model/items/Item.h"

namespace view {
	AppView::AppView()
		: _videoWidth(1600),
		_videoHeight(900),
		_frameRate(144),
		_zoom(1.0/45.0)
	{
	}

	AppView::AppView(model::Vehicle const& vehicle, std::vector<model::Cone> const& map)
		: _videoWidth(1600),
		_videoHeight(900),
		_frameRate(144),
		_zoom(1.0 / 45.0), 
		_vehicleView(vehicle)
	{
		_coneViews.reserve(map.size());
		for (auto& cone : map) {
			_coneViews.emplace_back(ConeView(cone));
			_itemViewTable.emplace(&cone, &_coneViews.back());
		}

	}

	void AppView::init()
	{
		_window.create(sf::VideoMode(_videoWidth, _videoHeight), "Vehicle Simulation");
		_window.setFramerateLimit(_frameRate);
		_view.setSize(_videoWidth, _videoHeight);
		_view.setCenter(_vehicleView.getPosition());
		_view.zoom(_zoom);
		_window.setView(_view);
	}

	void AppView::render()
	{
		_setupGrid();
		_window.clear(sf::Color::Black);
		_view.setCenter(_vehicleView.getPosition());
		_window.setView(_view);
		//Drawing
		_drawGrid();
		_window.draw(_vehicleView);
		for (auto const& coneView : _coneViews) {
			_window.draw(coneView);
		}
		for (auto& path : _pathViews) {
			_window.draw(path);
		}
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
				_window.close(); 
			else if (event.type == sf::Event::Resized)
			{
				// update the view to the new size of the window
				auto resized = event.size;
				_videoWidth = resized.width;
				_videoHeight = resized.height;
				_view.setSize(resized.width, resized.height);
				_view.setCenter(_vehicleView.getPosition());
				_view.zoom(_zoom);
				
				_notify("AppView", model::events::Resized{});
			} 
			else
			{
				model::events::InputEvent e = _translateToInput(event);
				if (!std::holds_alternative<model::events::None>(e))
					_notify("AppView", e);
			}


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

	void AppView::setVehicle(model::Vehicle const& vehicle)
	{
		_vehicleView = VehicleView(vehicle);
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
			_pathViews.push_back(PathView());
			return;
		}
		for (auto const& path : pathVector) {
			_pathViews.emplace_back(path);
			//_itemViewTable.emplace(&path, _pathViews.back());
		}
		_pathViews.back().setColor(sf::Color::Green);

	}

	void AppView::setConeDetectedFlag(std::vector<const model::Cone*> detectedCones)
	{
		for (auto& coneView : _coneViews) {
			coneView.isDetected = false;
		}
		for (auto const& item : detectedCones) {
			_itemViewTable[item]->isDetected = true;
		}
	}

	void AppView::_setupGrid()
	{
		sf::Vector2f topLeft(_window.mapPixelToCoords({ 0,0 }));
		sf::Vector2f bottomRight(_window.mapPixelToCoords({ _videoWidth,_videoHeight }));
		if (_cellSize / _zoom < 9.0) {
			_gridLines.clear();
			return;
		}
		double cellSize = _cellSize;

		int startX = int(std::floor(topLeft.x / cellSize));
		int endX = int((std::floor(bottomRight.x / cellSize) + 1));
		int startY = int(std::floor(topLeft.y / cellSize));
		int endY = int((std::floor(bottomRight.y / cellSize) + 1));

		sf::VertexArray lines(sf::Lines);

		sf::Color lineColor(80, 80, 80); // subtle gray


		// vertical lines
		for (int x = startX; x <= endX; ++x) {
			float px = x * cellSize;
			lines.append(sf::Vertex(sf::Vector2f(px, startY * cellSize), lineColor));
			lines.append(sf::Vertex(sf::Vector2f(px, endY * cellSize), lineColor));
		}

		// horizontal lines
		for (int y = startY; y <= endY; ++y) {
			float py = y * cellSize;
			lines.append(sf::Vertex(sf::Vector2f(startX * cellSize, py), lineColor));
			lines.append(sf::Vertex(sf::Vector2f(endX * cellSize, py), lineColor));
		}
		_gridLines = lines;
	}

	void AppView::_drawGrid()
	{
		_window.draw(_gridLines);
	}

	model::events::InputEvent AppView::_translateToInput(sf::Event event)
	{

		if (event.type == sf::Event::KeyPressed) {
			if (event.key.code == sf::Keyboard::Escape)
				return model::events::PressedEsc();
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

				//std::cout << "Map coords (on button press):" << std::endl;
				//std::cout << "map x: " << worldPos.x << std::endl;
				//std::cout << "map y: " << worldPos.y << std::endl;
				//_clickGlobalPos = model::Point(worldPos.x, worldPos.y);
				return model::events::ClickedAt{ (double)worldPos.x, (double)worldPos.y };
			}
		}
		if (event.type == sf::Event::MouseWheelMoved)
		{
			int delta = event.mouseWheel.delta;
			//double zoom = 1 - delta * 0.25;
			//_zoom = _zoom * zoom;
			////std::cout << delta << std::endl;
			//_view.zoom(1 - delta * 0.25);
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

