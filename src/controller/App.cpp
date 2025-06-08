#include "controller/App.h"
#include <cassert>
#include <queue>
#include <thread>
#include <variant>
#include "model/Vehicle.h"
#include "view/AppView.h"
#include "model/utils/SimpleTimer.h"
#include "model/mapReaders/IMapReader.h"
#include "model/perception/Perception.h"
#include "model/pathPlanning/PathPlanner.h"
#include "model/utils/ModelUtils.h"
#include "model/items/ObstacleBase.h"
#include "model/utils/FixSizedQueue.h"

namespace controller {
	void App::_calcGoal(std::vector<const model::Cone*> const& cones, model::VehicleState const& state)
	{
		double maxL = 0, maxR = 0;
		const model::Cone* maxLCone = nullptr, * maxRCone = nullptr;
		for (auto const& cone : cones) {
			double magnitude = (cone->getPosition() - state.getPosition()).magnitude();
			if (cone->getType() == model::ConeType::LEFT) {
				if (magnitude > maxL) {
					maxLCone = cone;
					maxL = magnitude;
				}
			}
			if (cone->getType() == model::ConeType::RIGHT) {
				if (magnitude > maxR) {
					maxRCone = cone;
					maxR = magnitude;
				}
			}
		}
		if (maxRCone && maxLCone) {
			model::Point goal((maxRCone->getPosition() + maxLCone->getPosition()) / 2.0);
			_vehicle->setGoal(goal);
		}

	}
	App::App(std::unique_ptr<model::Vehicle> vehicle, std::unique_ptr<model::IMapReader> mapReader, std::unique_ptr<view::AppView> view)
	{
		assert(mapReader != nullptr && "Provide valid arguments.");
		assert(vehicle != nullptr && "Provide valid arguments.");
		//assert(view != nullptr && "Provide valid arguments.");

		_mapReader = std::move(mapReader);
		_vehicle = std::move(vehicle);
		_view = std::move(view);
		if (_view) _view->attach(this);
	}

	void App::run()
	{
		// Read the map from the file
		std::vector<model::Cone> obstacleData = _mapReader->Read();
		for (model::Cone const& data : obstacleData) {
			_cones.emplace_back(data.getPosition(), data.getRadius(), data.getType());
		}

		_perception = std::make_unique<model::Perception>(_cones, M_PI / 2.0, 20);

		double cellSize = _vehicle->getCellSize();
		if (_view) {
			_view->setVehicle(*_vehicle);
			_view->setCones(_cones);
			_view->init();
			_view->setGridSize(cellSize);
			double frameTime = _view->getFrameTime();
		} 
		model::FixSizedQueue<model::Path> paths(5);
		// 
		std::atomic<bool> running = true;
		std::vector<const model::Cone*> detectedCones = _perception->detect(_vehicle->getPose());
		model::Point goalPos(10.0, 0.0);
		// Run path planning thread
		// Sets planned path ✓ safe
		// Reads Perception ✓ safe
		// Reads VehicleState ✓ safe
		// Modifies _vehicle ?
		std::thread pathPlanningThread([this, &detectedCones, &running, &goalPos, &paths]() {
			double time = 0.0;
			int iter = 0;
			std::vector<double> sorted_time;
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

				// Bit of a hack to get the furthest 2 cones middle point
				_calcGoal(detectedCopy, stateCopy);

				auto dur = model::timeFunction("Path planning", [this, &detectedCopy, &stateCopy]() {
					_vehicle->planPath(detectedCopy, stateCopy);
					});
				time += dur.count();
				sorted_time.emplace_back(dur.count());
				{
					std::lock_guard<std::mutex> lock(_simLock);
					_vehicle->setPlannedPath();
					paths.push(_vehicle->getPlannedPath());
				}
				std::this_thread::sleep_for(std::chrono::duration<double>(0.1 - dur.count()));
				iter++;
			}
			std::sort(sorted_time.begin(), sorted_time.end());
			std::cout << "Avg. path planning time: " << time / iter << std::endl <<
				"Path planning iterations: " << iter << std::endl <<
				"Slowest: " << sorted_time.back() << std::endl <<
				"Slowest (99th percentile): " << sorted_time[sorted_time.size() * 0.99] << std::endl <<
				"Slowest (9th percentile): " << sorted_time[sorted_time.size() * 0.9] << std::endl;
			});

		// Run the game loop
		double updateTime = 0;
		SimpleTimer frameTimer;
		int iter = 0;
		bool open = true;
		while (open) {
			//model::Path plannedPathCopy;
			double deltaTime = frameTimer.elapsedSeconds();
			if (_view) {
				_view->pollEvents();
			}
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
				if (_view) {
					if (goalPos != _view->getClickGlobalPos()) {
						std::cout << "Previous goal: " << std::endl << goalPos << std::endl;
						goalPos = _view->getClickGlobalPos();
						std::cout << "Next Goal: " << std::endl << goalPos << std::endl;
					}
				}

				frameTimer.reset();
				// Update the state
				_vehicle->update(deltaTime);
				if (_view) {
					_view->setPath(paths.get());
				}
				//plannedPathCopy = _vehicle->getPlannedPath();
			}
			if (_view) {
				_view->setConeDetectedFlag(detectedCones);
				// Render the simulation
				_view->render();
			}
			{
				std::lock_guard<std::mutex> lock(_simLock);
				detectedCones = _perception->detect(_vehicle->getPose());
			}
			iter++;
			if (_view) {
				open = _view->isOpen();
			}
		}
		running.store(false);
		pathPlanningThread.join();
		std::cout << "Main thread iterations: " << iter << std::endl;
	}

	App::~App() = default;

	void controller::App::handleInputEvent(std::string const& src, model::events::InputEvent const& e)
	{
		std::visit(InputTranslate(), e);
	}

}