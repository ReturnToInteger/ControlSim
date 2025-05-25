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
	_view->setCones(_cones);
	_view->init();
	double frameTime = _view->getFrameTime();

	// Run path planning thread
	std::thread pathPlanningThread([this]() {
		while (_view->isOpen()) {
			_vehicle->clearPath();
			_vehicle->planPath();
			{
				std::lock_guard<std::mutex> lock(_simLock);
				_vehicle->setPlannedPath();
			}
		}
		});
	// Run the game loop
	double updateTime = 0;
	SimpleTimer frameTimer;
	while (_view->isOpen()) {

		// Detect the cones
		std::vector<model::Cone*> detectedCones = _perception->detect(_vehicle->getPosition(), _vehicle->getOrientation());
		//// Plan the path
		//_vehicle->clearPath();
		//timeFunction("Path planning", [this, &detectedCones]() {
		//	_vehicle->planPath(detectedCones);
		//	});

		//timeFunction("Setting planned path", [this]() {
		//	_vehicle->setPlannedPath();
		//	});

		// Update the state
		// Update the vehicle
		double deltaTime = frameTimer.elapsedSeconds();
		frameTimer.reset();
		_vehicle->update(deltaTime);
		
		_view->pollEvents();
		std::vector<model::Point> plannedPath;
		{
			std::lock_guard<std::mutex> lock(_simLock);
			plannedPath = _vehicle->getPlannedPath();
		}
		_cones = _perception->getCones();
		_view->setPath(plannedPath);
		// Render the simulation
		_view->render();
	}

	//pathPlanningThread.join();
}

