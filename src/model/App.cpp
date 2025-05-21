#include "App.h"
#include <cassert>

model::App::App(std::unique_ptr<Vehicle> vehicle,std::unique_ptr<model::IMapReader> mapReader, std::unique_ptr<view::AppView> view)
{
	assert(mapReader != nullptr && "Provide valid arguments.");
	assert(view!= nullptr && "Provide valid arguments.");
	assert(vehicle != nullptr && "Provide valid arguments.");

	_mapReader = std::move(mapReader);
	_view = std::move(view);
	_vehicle = std::move(vehicle);
}

void model::App::run()
{
	// Read the map from the file
	auto obstacleData = _mapReader->Read();
	for (const auto& data : obstacleData) {
		_cones.emplace_back(data);
	}

	_perception = std::make_unique<model::Perception>(_cones);

	_view->setVehicle(*_vehicle);
	_view->init();
	double frameTime = _view->getFrameTime();
	// Run the game loop
	double updateTime = 0;
	Timer frameTimer;
	while (_view->isOpen()) {
		// Detect the cones
		std::vector<model::Cone*> detectedCones = _perception->detect(_vehicle->getPosition(), _vehicle->getOrientation());
		// Plan the path
		// TODO
		
		// Update the state
		double deltaTime = frameTimer.elapsedSeconds();
		frameTimer.reset();
		_vehicle->update(deltaTime);

		// Check for collisions with obstacles
		//  TODO
		_view->pollEvents();

		// Render the simulation
		_view->setCones(_cones);
		_view->render();
		// Update the vehicle
	}
}

