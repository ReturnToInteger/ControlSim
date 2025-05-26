#pragma once 

#include <SFML/Graphics.hpp>
#include "src/model/Point.h"
#include "VehicleView.h"
#include "ConeView.h"
#include "PathView.h"
#include <thread>

 
namespace view { 
class AppView { 
public: 
    AppView(); 
	AppView(const model::Vehicle & vehicle, const std::vector<model::Cone>& map);
    ~AppView()=default; 
    AppView(const AppView&) = delete;
	void init();
	void render();
	void pollEvents();
	bool isOpen() const;
	double getFrameTime() const;
	void close();
	void setVehicle(const model::Vehicle & vehicle);
	void setCones(const std::vector<model::Cone>& cones);
	void setPath(std::vector<model::Pose> path);
	void setConesDetectedFlag(std::vector<const model::Cone*> detectedCones);
	double getWidth() const { return _videoWidth; }
	double getHeight() const { return _videoHeight; }

private:
    sf::RenderWindow _window;
	int _videoWidth;
	int _videoHeight;
	int _frameRate;
	double _zoom;
	model::Point startPos;

	VehicleView _vehicleView;
	std::unordered_map<const model::Cone*,ConeView*> _coneViewTable;
	std::vector<ConeView> _coneViews;

	PathView _pathView;
	sf::View _view;
	//std::vector<sf::Drawable> _drawables;

};
} // namespace view 
