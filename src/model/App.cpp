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
	std::vector<Cone> obstacleData = _mapReader->Read();
	for (const Cone& data : obstacleData) {
		_cones.emplace_back(data);
	}

	_perception = std::make_unique<model::Perception>(_cones,M_PI, 20);

	_view->setVehicle(*_vehicle);
	_view->setCones(_cones);
	_view->init();
	double frameTime = _view->getFrameTime();

	// 
	std::atomic<bool> running=true;
	std::vector<const model::Cone*> detectedCones = _perception->detect(_vehicle->getPose());
	model::Point goalPos(10.0,0.0);
	// Run path planning thread
	// Sets planned path ✓ safe
	// Reads Perception ✓ safe
	// Reads VehicleState ✓ safe
	// Modifies _vehicle ?
	std::thread pathPlanningThread([this, &detectedCones, &running, &goalPos]() {
		while (running) {
			_vehicle->clearPath();
			std::vector<const model::Cone*> detectedCopy;
			model::VehicleState stateCopy;
			{
				std::lock_guard<std::mutex> lock(_simLock);
				detectedCopy = detectedCones;
				stateCopy = _vehicle->getStateCopy();
				//_vehicle->setGoal(goalPos);
				//std::cout << "Path goal set: " << goalPos << std::endl;
			}
			double maxL=0, maxR=0;
			const Cone *maxLCone=nullptr, *maxRCone=nullptr;
			for (const auto& cone : detectedCopy) {
				double magnitude = (cone->getPosition()- stateCopy.getPosition()).magnitude();
				if (cone->getType() == ConeType::LEFT) {
					if (magnitude> maxL) {
						maxLCone = cone;
						maxL = magnitude;
					}
				} 
				if (cone->getType() == ConeType::RIGHT) {
					if (magnitude > maxR) {
						maxRCone = cone;
						maxR = magnitude;
					}
				}
			}
			if (maxRCone && maxLCone) {
				Point goal((maxRCone->getPosition() + maxLCone->getPosition()) / 2.0);
				_vehicle->setGoal(goal);
			}
			timeFunction("Path planning", [this, &detectedCopy, &stateCopy]() {
				_vehicle->planPath(detectedCopy, stateCopy);
				});
			//_vehicle->planPath(detectedCopy, stateCopy);
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
		std::vector<model::Pose> plannedPathCopy;
		double deltaTime = frameTimer.elapsedSeconds();

		_view->pollEvents();
		// Update the vehicle
		// Locked basically the whole loop, maybe it doesn't need to be like this, but this is fast to compute
		// View reads user input for pathPlanning
		// Sets VehicleState ✓ safe
		// Gets plannedPath ✓ safe
		// Gets detectedCones ✓ safe
		// Sets detectedCones ✓ safe
		// Modifies _vehicle ✓ safe
		// View reads vehicle?

		{
			std::lock_guard<std::mutex> lock(_simLock);
			if (goalPos != _view->getClickGlobalPos()) {
				std::cout << "Previous goal: " << goalPos;
				goalPos=_view->getClickGlobalPos();
				std::cout << "Next Goal: " << goalPos;

			}
			frameTimer.reset();
			// Update the state
			_vehicle->update(deltaTime);
			plannedPathCopy = _vehicle->getPlannedPath();
		}
		_view->setConeDetectedFlag(detectedCones);
		_view->setPath(plannedPathCopy);
		// Render the simulation
		_view->render();
		{
			std::lock_guard<std::mutex> lock(_simLock);
			detectedCones = _perception->detect(_vehicle->getPose());
		}
	}

	running.store(false);
	pathPlanningThread.join();
}

