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
#include <numbers>
#include <algorithm>
#include <ranges>


#ifdef ENABLE_DEBUG_DRAW
#include <view/DebugDraw.h>
#endif // ENABLE_DEBUG_DRAW




namespace controller {
	App::App(std::unique_ptr<model::Vehicle> vehicle,
		std::unique_ptr<model::IMapReader> mapReader,
		std::unique_ptr<view::AppView> view,
		int threadCount)
		: _mapReader(std::move(mapReader)),
		_vehicle(std::move(vehicle)),
		_view(std::move(view)),
		_threadCount(threadCount),
		_running(false),
		_sharedGoals{ 
			.goals= { _vehicle->getPosition(), _vehicle->getPosition() },
			.goalHasChanged= false }
	{
		assert(_mapReader != nullptr && "Provide valid arguments.");
		assert(_vehicle != nullptr && "Provide valid arguments.");
		//assert(_view != nullptr && "Provide valid arguments.");

		if (_view)
		{
			_view->attach(this);
		}
	}

	void App::run()
	{
		// Read the map from the file
		std::vector<model::Cone> obstacleData = _mapReader->Read();
		for (model::Cone const& data : obstacleData) {
			_cones.emplace_back(data.getPosition(), data.getRadius(), data.getType());
		}
		Angle perceptionAngle = radian(75);
		double perceptionDistance = 15;
		_perception = std::make_unique<model::Perception>(_cones, perceptionAngle, perceptionDistance);

		double cellSize = _vehicle->getCellSize();
		double frameTime;
		if (_view) {
			_view->setVehicle(*_vehicle);
			_view->setCones(_cones);
			_view->init();
			_view->setGridSize(cellSize);
			frameTime = _view->getFrameTime();
		}
		else {
			frameTime = 1.0 / 60.0; // TO DO: Make NullView
		}
		_running = true;
		std::unordered_set<const model::Cone*> detectedCones = _perception->detect(_vehicle->getPose());
		std::function<void(int)> pathPlanningLambda = [this, &detectedCones](int index) {
			_pathPlanningWorker(detectedCones, index);
			};
		std::vector<std::thread> threads;
		_startPlanningThreads(_threadCount, threads, pathPlanningLambda);

		// Run the game loop
		double updateTime = 0;
		SimpleTimer frameTimer;
		int iter = 0;
		int coneFlushIter = 60;
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
				if (iter % (coneFlushIter) == 0) {
					detectedCones.clear();
				}
				for (auto const& cone: currentDetected) {
					detectedCones.emplace(cone);
				}
				double wayPoint1Distance = 10;
				double wayPoint2Distance = 150;
				bool foundCurrent1 = _calcGoal(detectedCones, _vehicle->getStateCopy(), _sharedGoals.goals[0], wayPoint1Distance);
				bool foundCurrent2 = _calcGoal(detectedCones, _vehicle->getStateCopy(), _sharedGoals.goals[1], wayPoint2Distance);
				if (foundCurrent1 || foundCurrent2) {
					_sharedGoals.goalHasChanged.store(true);
				}

			}
			if (_view) {
				_view->setPath(pathCopy);
				_view->setConeDetectedFlag(detectedCones);
				// Render the simulation
				_view->render();
			}
			if (!_view)
			{
				std::this_thread::sleep_for(std::chrono::duration<double>(frameTime - deltaTime));
			}
			iter++;
		}
		_running.store(false);
		for (auto& t : threads) {
			t.join();
		}
		std::cout << "________________\n" <<
			"Main thread iterations: " << iter << "\n";
	}


	// Path planning threadsafe loop
	// Sets planned path ✓ safe
	// Reads Perception ✓ safe
	// Reads VehicleState ✓ safe
	// Modifies _vehicle ?
	void App::_pathPlanningWorker(std::unordered_set<const model::Cone*>& detectedCones, int const index)
	{
		std::cout << "Planning index is: " << index << "\n";

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
		while (_running.load(std::memory_order_relaxed)) {

			// Read data from main
			std::unordered_set<const model::Cone*> detectedCopy;
			model::VehicleState stateCopy;				
			model::Point previous1(goal1);
			model::Point previous2(goal2);

			{
				std::lock_guard<std::mutex> lock(_simLock);
				detectedCopy = detectedCones;
				stateCopy = _vehicle->getStateCopy();			

				goal1 = _sharedGoals.goals[0];
				goal2 = _sharedGoals.goals[1];
			} // End of reading



			// Bit of a hack to get the furthest 2 cones middle point
			goal1 = (1 - filterParam) * goal1 + filterParam * previous1;
			goal2 = (1 - filterParam) * goal2 + filterParam * previous2;
			if ((stateCopy.getPosition() - goal1).magnitude() <= (stateCopy.getPosition() - goal2).magnitude()) {
				_vehicle->setGoal(goal1, index);
				_vehicle->setGoal(goal2, index);
			}
			else {
				_vehicle->setGoal(goal2, index);
				_vehicle->setGoal(goal1, index);
			}
			#ifdef ENABLE_DEBUG_DRAW
			view::DebugDraw::instance().circle(goal1, 0.5, sf::Color::Cyan);
			view::DebugDraw::instance().circle(goal2, 0.5, sf::Color::Green);
			#endif // ENABLE_DEBUG_DRAW

			// Dealing with the current pathPlanner
			std::chrono::duration<double> dur{};
			// Time path planning
			if (_sharedGoals.goalHasChanged.load()) {
				// Clear residual data before planning new one
				_vehicle->clearPath(index);
				dur = model::timeFunction("Path planning", [this, &detectedCopy, &stateCopy, &index, &foundPath]() {
					foundPath = _vehicle->planPath(detectedCopy, stateCopy, index);
					});
				_sharedGoals.goalHasChanged.store(false);
			}
			else
			{
				dur = std::chrono::duration<double>(0);
			}
			time += dur.count();
			sortedTime.emplace_back(dur.count());
			// Sending data back to main
			{
				std::lock_guard<std::mutex> lock(_simLock);
				_vehicle->setPlannedPath(index);
			} // End of sending data
			double delayScnd = 0.005;
			double sleepDur = std::max(delayScnd * _threadCount * (double)(index+1.0) / (double)_threadCount-dur.count(), delayScnd); // Spread out thread execution over time
			std::this_thread::sleep_for(std::chrono::duration<double>(sleepDur)); // Sleep to avoid pointless CPU overload.
			// End of dealing with the current pathPlanner
			iter++;
		}
		std::ranges::sort(sortedTime);
		std::lock_guard<std::mutex> lock(_pathLock);
		std::cout << "________________\n" <<
			"THREAD ID: " << std::this_thread::get_id() << "\n" <<
			"Avg. path planning time: " << time / iter << "\n" <<
			"Path planning iterations: " << iter << "\n" <<
			"Slowest: " << sortedTime.back() << "\n" <<
			"Slowest (99th percentile): " << sortedTime[static_cast<size_t>(sortedTime.size() * 99/100)] << "\n" <<
			"Slowest (9th percentile): " << sortedTime[static_cast<size_t>(sortedTime.size() * 9/10)] << "\n";

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
	bool controller::App::_calcGoal(std::unordered_set<const model::Cone*> const& cones, model::VehicleState const& state, model::Point& currentGoal, double maxDist)
	{
		double maxL = 0;
		double maxR = 0;
		const model::Cone* maxLCone = nullptr;
		const model::Cone* maxRCone = nullptr;
		Point pos = state.getPosition();
		Angle theta = state.getOrientation();
		for (auto const& cone : cones) 
		{
			Point relPos = cone->getPosition() - pos;
			double magnitude = relPos.magnitude();
			double xDist = relPos.X() * cos(theta) + relPos.Y() * sin(theta);
			if (magnitude <= maxDist && xDist > 0) 
			{
				if (cone->getType() == model::ConeType::LEFT&& magnitude > maxL) 
				{
					maxLCone = cone;
					maxL = magnitude;
				}
				if (cone->getType() == model::ConeType::RIGHT&& magnitude > maxR) 
				{
					maxRCone = cone;
					maxR = magnitude;
				}
			}
		}
		if (maxRCone && maxLCone) 
		{
			currentGoal=(maxRCone->getPosition() + maxLCone->getPosition()) / 2.0;

			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW
			return true;
		}
		if (maxRCone) 
		{
			model::Angle direction = state.getOrientation() + std::numbers::pi / 2.0;
			model::Point offset(_vehicle->getLength()/2* cos(direction), _vehicle->getLength()/2 * sin(direction));
			currentGoal=maxRCone->getPosition() + offset;

			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW
			return true;
		}
		if (maxLCone) 
		{
			model::Angle direction = state.getOrientation() - std::numbers::pi / 2.0;
			model::Point offset(_vehicle->getLength()/2 * cos(direction), _vehicle->getLength()/2 * sin(direction));
			currentGoal=maxLCone->getPosition()+offset;

			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW
			return true;
		}
		return false;
	}

}