#include "AppView.h" 
 
namespace view {
	AppView::AppView()
		: _videoWidth(1600),
		_videoHeight(900),
		_frameRate(144),
		_zoom(1.0/25.0)
	{
	}

	AppView::AppView(model::Vehicle& vehicle, std::vector<model::Cone>& cones) : AppView()
	{
		_vehicleView = VehicleView(&vehicle);
		_coneViews.reserve(cones.size());
		for (auto& cone : cones) {
			_coneViews.emplace_back(ConeView(&cone));
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

		//Drawing
		_window.draw(_vehicleView);
		for (const auto& coneView : _coneViews) {
			_window.draw(coneView);
		}

		_window.display();
	}

	void AppView::pollEvents()
	{
		sf::Event event;
		while (_window.pollEvent(event)){
			if (event.type == sf::Event::Closed)
				_window.close();

			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Escape)
					_window.close();
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

	void AppView::setVehicle(model::Vehicle* vehicle)
	{
		_vehicleView = VehicleView(vehicle);
		_view.setCenter(_vehicleView.getPosition());
	}

	void AppView::setCones(std::vector<model::Cone>& cones)
	{
		_coneViews.clear();
		_coneViews.reserve(cones.size());
		for (auto& cone : cones) {
			_coneViews.emplace_back(ConeView(&cone));
		}
	}

}