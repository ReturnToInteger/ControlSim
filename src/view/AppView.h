#pragma once 

#include <SFML/Graphics.hpp>
#include "src/model/Point.h"
#include "VehicleView.h"
#include "ConeView.h"

 
namespace view { 
class AppView { 
public: 
    AppView(); 
	AppView(model::Vehicle& vehicle, std::vector<model::Cone>& cones);
    ~AppView()=default; 
    AppView(const AppView&) = delete;
	void init();
	void render();
	void pollEvents();
	bool isOpen() const;
	double getFrameTime() const;
	void close();
	void setVehicle(model::Vehicle* vehicle);
	void setCones(std::vector<model::Cone>& cones);

private:
    sf::RenderWindow _window;
	int _videoWidth;
	int _videoHeight;
	int _frameRate;
	double _zoom;
	model::Point startPos;

	VehicleView _vehicleView;
	std::vector<ConeView> _coneViews;
	sf::View _view;

};
} // namespace view 
