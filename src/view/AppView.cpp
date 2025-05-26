#include "AppView.h" 
#include <cassert>
 
namespace view {
	AppView::AppView()
		: _videoWidth(1600),
		_videoHeight(900),
		_frameRate(144),
		_zoom(1.0/25.0)
	{
	}

	view::AppView::AppView(const model::Vehicle& vehicle, const std::vector<model::Cone>& map)
		: _videoWidth(1600),
		_videoHeight(900),
		_frameRate(144),
		_zoom(1.0 / 25.0), 
		_vehicleView(vehicle)
	{
		_coneViews.reserve(map.size());
		for (auto& cone : map) {
			_coneViews.emplace_back(ConeView(cone));
			_coneViewTable.emplace(&cone, &_coneViews.back());
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

	void view::AppView::render()
	{

		_window.clear(sf::Color::Black);
		_view.setCenter(_vehicleView.getPosition());
		_window.setView(_view);
		//Drawing
		_window.draw(_vehicleView);
		for (const auto& coneView : _coneViews) {
			_window.draw(coneView);
		}
		_window.draw(_pathView);

		//if (_drawables) {
		//	for (const auto& drawable : _drawables) {
		//		_window.draw(drawable);
		//	}
		//}

		_window.display();
	}
	//This should be in the controller
	void AppView::pollEvents()
	{
		sf::Event event;
		while (_window.pollEvent(event)){
			if (event.type == sf::Event::Closed)
				_window.close();

			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Escape)
					_window.close();
				if (event.key.code == sf::Keyboard::LShift) {
					// Wait until released
					while (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)) {
						// Do nothing
					}
				}
			}
			if (event.mouseButton.button == sf::Mouse::Left)
			{
				std::cout << "the right button was pressed" << std::endl;
				std::cout << "mouse x: " << event.mouseButton.x << std::endl;
				std::cout << "mouse y: " << event.mouseButton.y << std::endl;
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

	void view::AppView::setVehicle(const model::Vehicle& vehicle)
	{
		_vehicleView = VehicleView(vehicle);
		_view.setCenter(_vehicleView.getPosition());
	}

	void view::AppView::setCones(const std::vector<model::Cone>& cones)
	{
		_coneViews.clear();
		_coneViews.reserve(cones.size());
		for (const auto& cone : cones) {
			_coneViews.emplace_back(ConeView(cone));	
			_coneViewTable.emplace(&cone, &_coneViews.back());
		}
	}

	void view::AppView::setPath(std::vector<model::Pose> path)
	{
		_pathView = PathView(path);
	}

	void view::AppView::setConesDetectedFlag(std::vector<const model::Cone*> detectedCones)
	{
		for (auto& coneView : _coneViews) {
			coneView.isDetected = false;
		}
		for (const auto& cone : detectedCones) {
			_coneViewTable[cone]->isDetected = true;
		}
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