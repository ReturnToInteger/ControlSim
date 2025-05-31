#pragma once 

#include <SFML/Graphics.hpp>
#include "model/utils/Point.h"
#include "model/items/Item.h"
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
	void setConeDetectedFlag(std::vector<const model::Cone*> detectedCones);
	void setGridSize(const double& cellSize) { _cellSize = cellSize; }
	double getWidth() const { return _videoWidth; }
	double getHeight() const { return _videoHeight; }
	model::Point getClickGlobalPos() const { return _clickGlobalPos; }

private:
	void _setupGrid();
	void _drawGrid();
    sf::RenderWindow _window;
	int _videoWidth;
	int _videoHeight;
	int _frameRate;
	double _zoom;
	model::Point _startPos;
	model::Point _clickGlobalPos;
	double _cellSize;

	VehicleView _vehicleView;
	std::unordered_map<const model::Item*,ItemView*> _itemViewTable;
	std::vector<ConeView> _coneViews;

	PathView _pathView;
	sf::View _view;
	sf::VertexArray _gridLines;
	//std::vector<sf::Drawable> _drawables;

};
} // namespace view 
