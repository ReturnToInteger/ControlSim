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

#ifdef ENABLE_DEBUG_DRAW
#include <view/DebugDraw.h>
#endif // ENABLE_DEBUG_DRAW




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

		_perception = std::make_unique<model::Perception>(_cones, radian(75), 15);

		double cellSize = _vehicle->getCellSize();
		double frameTime;
		if (_view) {
			_view->setVehicle(*_vehicle);
			_view->setCones(_cones);
			_view->init();
			_view->setGridSize(cellSize);
			frameTime = _view->getFrameTime();
		}
		else 
			frameTime = 1.0 / 60.0;
		std::atomic<bool> running = true;
		std::unordered_set<const model::Cone*> detectedCones = _perception->detect(_vehicle->getPose());
		const int threadCount = 1;
		std::function<void(int)> pathPlanningLambda = [this, &detectedCones, &running, &threadCount](int index) {
			_pathPlanningWorker(detectedCones, running, threadCount, index);
			};
		std::vector<std::thread> threads;
		_startPlanningThreads(threadCount, threads, pathPlanningLambda);

		// Run the game loop
		double updateTime = 0;
		SimpleTimer frameTimer;
		int iter = 0;
		while (_view->isOpen()) {
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
				auto currentDetected = _perception->detect(_vehicle->getPose());
				if (iter % (60) == 0)
					detectedCones.clear();

				for (auto const& cone: currentDetected) {
					detectedCones.emplace(cone);
				}
			}
			if (_view) {
				_view->setPath(pathCopy);
				_view->setConeDetectedFlag(detectedCones);
				// Render the simulation
				_view->render();
			}
			if (!_view)
				std::this_thread::sleep_for(std::chrono::duration<double>(frameTime - deltaTime));
			iter++;
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
	void App::_pathPlanningWorker(std::unordered_set<const model::Cone*>& detectedCones,std::atomic_bool& running,int const threadCount, int const index)
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
		std::vector<double> sortedTime;
		
		model::Point goal1;
		model::Point goal2;

		{
			std::lock_guard<std::mutex> lock(_simLock);
			goal1 = _vehicle->getPosition();
		} 
		goal2 = goal1;

		model::VehicleState stateCopy;
		double filterParam = 0.25;
		bool foundPath = false;
		while (running.load(std::memory_order_relaxed)) {

			// Read data from main
			std::unordered_set<const model::Cone*> detectedCopy;
			model::VehicleState stateCopy;
			{
				std::lock_guard<std::mutex> lock(_simLock);
				detectedCopy = detectedCones;
				stateCopy = _vehicle->getStateCopy();
			} // End of reading



			// Bit of a hack to get the furthest 2 cones middle point
			model::Point previous1(goal1);
			model::Point previous2(goal2);
			bool foundGoal1 = _calcGoal(detectedCopy, stateCopy, goal1, 10, index);
			bool foundGoal2 = _calcGoal(detectedCopy, stateCopy, goal2, 150, index);
			goal1 = (1 - filterParam) * goal1 + filterParam * previous1;
			goal2 = (1 - filterParam) * goal2 + filterParam * previous2;
			_vehicle->setGoal(goal1, index);
			_vehicle->setGoal(goal2, index);
			#ifdef ENABLE_DEBUG_DRAW
			view::DebugDraw::instance().circle(goal1, 0.5, sf::Color::Cyan);
			view::DebugDraw::instance().circle(goal2, 0.5, sf::Color::Green);
			#endif // ENABLE_DEBUG_DRAW

			// Dealing with the current pathPlanner
			std::chrono::duration<double> dur;
			// Time path planning
			if (foundGoal1 || foundGoal2/* || !foundPath*/) {
				//// Clear residual data before planning new one
				_vehicle->clearPath(index);
				dur = model::timeFunction("Path planning", [this, &detectedCopy, &stateCopy, &index, &foundPath]() {
					foundPath=_vehicle->planPath(detectedCopy, stateCopy, index);
					});
			}
			else dur = std::chrono::duration<double>(0);
			time += dur.count();
			sortedTime.emplace_back(dur.count());
			// Sending data back to main
			{
				std::lock_guard<std::mutex> lock(_simLock);
				_vehicle->setPlannedPath(index);
				} // End of sending data
			double sleepDur = std::max(0.005 * threadCount * (double)(index+1.0) / (double)threadCount-dur.count(),0.005); // Spread out thread execution over time
			std::this_thread::sleep_for(std::chrono::duration<double>(sleepDur)); // Sleep to avoid pointless CPU overload.
			// End of dealing with the current pathPlanner
			iter++;
		}
		std::sort(sortedTime.begin(), sortedTime.end());
		std::lock_guard<std::mutex> lock(_pathLock);
		std::cout << "________________" << std::endl <<
			"THREAD ID: " << std::this_thread::get_id() << std::endl <<
			"Avg. path planning time: " << time / iter << std::endl <<
			"Path planning iterations: " << iter << std::endl <<
			"Slowest: " << sortedTime.back() << std::endl <<
			"Slowest (99th percentile): " << sortedTime[static_cast<size_t>(sortedTime.size() * 0.99)] << std::endl <<
			"Slowest (9th percentile): " << sortedTime[static_cast<size_t>(sortedTime.size() * 0.9)] << std::endl;

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
		double rotateSpeed = 6.0;
		std::visit(Overload{ 
			[this](model::events::PressedEsc const&) {_view->close(); },
			//[this](model::events::ClickedAt const& click) {
			//		std::lock_guard<std::mutex> lock(_simLock);
			//		#ifdef ENABLE_DEBUG_DRAW
			//		view::DebugDraw::instance().circle(model::Point(click.x, click.y), 0.5, sf::Color::Red);
			//		#endif // ENABLE_DEBUG_DRAW

			//		_vehicle->setAllGoals(model::Point(click.x,click.y));
			//		},
			[this](model::events::Scrolled const& s) {_view->zoom(1 - s.delta * 0.25); },
			[this,  rotateSpeed](model::events::RightClickDown const& c) {
				double delta = c.currentX- c.lastX;
				_view->rotate(delta*rotateSpeed*_view->getFrameTime()); 
			},
			[](auto&&) {}
			}, e);
	}
	bool controller::App::_calcGoal(std::unordered_set<const model::Cone*> const& cones, model::VehicleState const& state, model::Point& currentGoal, double maxDist, int i)
	{
		double maxL = 0, maxR = 0;
		const model::Cone* maxLCone = nullptr, * maxRCone = nullptr;
		Point pos = state.getPosition();
		Angle theta = state.getOrientation();
		for (auto const& cone : cones) {
			Point relPos = cone->getPosition() - pos;
			double magnitude = relPos.magnitude();
			double xDist = relPos.X() * cos(theta) + relPos.Y() * sin(theta);
			if (magnitude <= maxDist && xDist > 0) {
				if (cone->getType() == model::ConeType::LEFT&& magnitude > maxL) {
					maxLCone = cone;
					maxL = magnitude;
				}
				if (cone->getType() == model::ConeType::RIGHT&& magnitude > maxR) {
					maxRCone = cone;
					maxR = magnitude;
				}
			}
		}
		if (maxRCone && maxLCone) {
			currentGoal=(maxRCone->getPosition() + maxLCone->getPosition()) / 2.0;

			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW
			return true;
		}
		else if (maxRCone) {
			model::Angle direction = state.getOrientation() + M_PI / 2.0;
			model::Point offset(_vehicle->getLength()/2* cos(direction), _vehicle->getLength()/2 * sin(direction));
			currentGoal=maxRCone->getPosition() + offset;

			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW
			return true;
		}
		else if (maxLCone) {
			model::Angle direction = state.getOrientation() - M_PI / 2.0;
			model::Point offset(_vehicle->getLength()/2 * cos(direction), _vehicle->getLength()/2 * sin(direction));
			currentGoal=maxLCone->getPosition()+offset;

			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW
			return true;
		}
		else 
			return false;
	}

}