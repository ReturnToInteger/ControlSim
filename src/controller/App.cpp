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
#include <windows.h>
#include <string>



namespace controller {
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

		_perception = std::make_unique<model::Perception>(_cones, M_PI / 2.0, 15);

		double cellSize = _vehicle->getCellSize();
		double frameTime;
		if (_view) {
			_view->setVehicle(*_vehicle);
			_view->setCones(_cones);
			_view->init();
			_view->setGridSize(cellSize);
			frameTime = _view->getFrameTime();
		}
		else frameTime = 1.0 / 60.0;
		std::atomic<bool> running = true;
		std::vector<const model::Cone*> detectedCones = _perception->detect(_vehicle->getPose());
		const int threadCount = 5;
		std::function<void(int)> pathPlanningLambda = [this, &detectedCones, &running, &threadCount](int index) {
			_pathPlanningWorker(detectedCones, running, threadCount, index);
			};
		std::vector<std::thread> threads;
		_startPlanningThreads(threadCount, threads, pathPlanningLambda);

		// Run the game loop
		double updateTime = 0;
		SimpleTimer frameTimer;
		int iter = 0;
		bool open = true;
		while (open) {
			double deltaTime = frameTimer.elapsedSeconds();
			// Update the vehicle
			// Locked basically the whole loop, maybe it doesn't need to be like this, but this is fast to compute
			// View reads user input for pathPlanning
			// Sets VehicleState ✓ safe
			// Gets plannedPath ✓ safe
			// Gets detectedCones ✓ safe
			// Sets detectedCones ✓ safe
			// Modifies _vehicle ✓ safe
			// View reads vehicle?
			if (_view) {
				_view->pollEvents();
			}
			std::deque<model::Path> pathCopy(5);
			{
				std::lock_guard<std::mutex> lock(_simLock);
				frameTimer.reset();
				// Update the state
				_vehicle->update(clamp(deltaTime,frameTime,frameTime*5));
				if (_view) {
					pathCopy=_vehicle->getPlannedPaths().get();
				}
			}
			if (_view) {
				_view->setPath(pathCopy);
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
			if (!_view)
				std::this_thread::sleep_for(std::chrono::duration<double>(frameTime - deltaTime));
		}
		running.store(false);
		for (auto& t : threads) {
			t.join();
		}
		std::cout << "________________" << std::endl <<
			"Main thread iterations: " << iter << std::endl;
	}


	// Path planning threadsafe loop
	// Sets planned path ✓ safe
	// Reads Perception ✓ safe
	// Reads VehicleState ✓ safe
	// Modifies _vehicle ?
	void App::_pathPlanningWorker(std::vector<const model::Cone*>& detectedCones,std::atomic_bool& running,int const threadCount, int const index)
	{
		std::cout << "Planning index is: " << index << std::endl;

        // Convert the integer index to a wide string (PCWSTR) for SetThreadDescription
        std::wstring threadDescription = L"Thread " + std::to_wstring(index);
        SetThreadDescription(GetCurrentThread(), threadDescription.c_str());

		// Track time of function calls
		double time = 0.0;
		// Track iterations
		int iter = 0;
		// Sorted durations of calls for statistics
		std::vector<double> sorted_time;
		while (running.load(std::memory_order_relaxed)) {

			// Read data from main
			std::vector<const model::Cone*> detectedCopy;
			model::VehicleState stateCopy;
			{
				std::lock_guard<std::mutex> lock(_simLock);
				detectedCopy = detectedCones;
				stateCopy = _vehicle->getStateCopy();
			} // End of reading

			// Dealing with the current pathPlanner
			std::chrono::duration<double> dur;

			//// Clear residual data before planning new one
			_vehicle->clearPath(index);

			// Bit of a hack to get the furthest 2 cones middle point
			_calcGoal(detectedCopy, stateCopy);

			// Time path planning
			dur = model::timeFunction("Path planning", [this, &detectedCopy, &stateCopy, &index]() {
				_vehicle->planPath(detectedCopy, stateCopy, index);
				});

			time += dur.count();
			sorted_time.emplace_back(dur.count());
			// Sending data back to main
			{
				std::lock_guard<std::mutex> lock(_simLock);
				_vehicle->setPlannedPath(index);
				} // End of sending data
			double sleepDur = 0.005 * threadCount * (double)(index+1.0) / (double)threadCount; // Spread out thread execution over time
			std::this_thread::sleep_for(std::chrono::duration<double>(sleepDur - dur.count())); // Sleep to avoid pointless CPU overload.
			// End of dealing with the current pathPlanner
			iter++;
		}
		std::sort(sorted_time.begin(), sorted_time.end());
		std::lock_guard<std::mutex> lock(_pathLock);
		std::cout << "________________" << std::endl <<
			"THREAD ID: " << std::this_thread::get_id() << std::endl <<
			"Avg. path planning time: " << time / iter << std::endl <<
			"Path planning iterations: " << iter << std::endl <<
			"Slowest: " << sorted_time.back() << std::endl <<
			"Slowest (99th percentile): " << sorted_time[static_cast<size_t>(sorted_time.size() * 0.99)] << std::endl <<
			"Slowest (9th percentile): " << sorted_time[static_cast<size_t>(sorted_time.size() * 0.9)] << std::endl;

	}

	void controller::App::_startPlanningThreads(int threadCount, std::vector<std::thread>& threads, std::function<void(int)> const& loopLambda)
	{
		// Assuming one planner already exists
		for (int i = 0; i < threadCount - 1; i++) {
			_vehicle->addPlanner();
		}
		// Starting threads
		for (int i = 0; i < threadCount; i++) {
			threads.emplace_back([loopLambda, i]() { loopLambda(i); });
		}
	}
	App::~App() = default;

	void controller::App::handleInputEvent(std::string const& src, model::events::InputEvent const& e)
	{
		// Log
		std::visit(InputTranslate(), e);
		// Events
		std::visit(overload{ 
			[this](model::events::PressedEsc const&) {_view->close(); },
			[this](model::events::ClickedAt const& click) {
					std::lock_guard<std::mutex> lock(_simLock);
					_vehicle->setAllGoals(model::Point(click.x,click.y));
					},
			[this](model::events::Scrolled const& s) {_view->zoom(1 - s.delta * 0.25); },
			[](auto&&) {}
			}, e);
	}
	void App::_calcGoal(std::vector<const model::Cone*> const& cones, model::VehicleState const& state)
	{
		double maxL = 0, maxR = 0, maxDist = 10;
		const model::Cone* maxLCone = nullptr, * maxRCone = nullptr;
		for (auto const& cone : cones) {
			double magnitude = (cone->getPosition() - state.getPosition()).magnitude();
			if (cone->getType() == model::ConeType::LEFT) {
				if (magnitude > maxL&& magnitude <=maxDist) {
					maxLCone = cone;
					maxL = magnitude;
				}
			}
			if (cone->getType() == model::ConeType::RIGHT) {
				if (magnitude > maxR && magnitude<= maxDist) {
					maxRCone = cone;
					maxR = magnitude;
				}
			}
		}
		if (maxRCone && maxLCone) {
			model::Point goal((maxRCone->getPosition() + maxLCone->getPosition()) / 2.0);

			_vehicle->setAllGoals(goal);
		}
		else if (maxRCone) {
			model::Point goal(maxRCone->getPosition());
			model::Angle direction = state.getOrientation()+M_PI;
			model::Point offset(_vehicle->getLength()/2 * cos(direction), _vehicle->getLength()/2 * sin(direction));
			_vehicle->setAllGoals(goal+offset);
		}
		else if (maxLCone) {
			model::Point goal(maxLCone->getPosition());
			model::Angle direction = state.getOrientation() - M_PI ;
			model::Point offset(_vehicle->getLength()/2 * cos(direction), _vehicle->getLength()/2 * sin(direction));
			_vehicle->setAllGoals(goal+offset);
		}

	}

}