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
#include "model/items/Obstacle.h"
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
	App::App(std::unique_ptr<model::IVehicle> vehicle,
		std::unique_ptr<model::IMapReader> mapReader,
		std::unique_ptr<view::AppView> view,
		int threadCount)
		: m_mapReader(std::move(mapReader)),
		m_vehicle(std::move(vehicle)),
		m_view(std::move(view)),
		m_threadCount(threadCount),
		m_running(false),
		m_sharedGoals{ 
			.goals= { m_vehicle->getPosition(), m_vehicle->getPosition() },
			.goalHasChanged= false }
	{
		assert(m_mapReader != nullptr && "Provide valid arguments.");
		assert(m_vehicle != nullptr && "Provide valid arguments.");
		//assert(m_view != nullptr && "Provide valid arguments.");

		if (m_view)
		{
			m_view->attach(this);
		}
		//Setup
		// Read the map from the file
		m_cones = m_mapReader->Read();

		//Create perception
		Angle perceptionAngle = radian(75);
		double perceptionDistance = 15;
		m_perception = std::make_unique<model::Perception>(m_cones, perceptionAngle, perceptionDistance);

		// Set up view
		double cellSize = m_vehicle->getCellSize();
		if (m_view) {
			m_view->setVehicle(*m_vehicle);
			m_view->setCones(m_cones);
			m_view->setGridSize(cellSize);
			m_view->init();
			m_frameTime = m_view->getFrameTime();
		}
		else {
			m_frameTime = 1.0 / 60.0; // TO DO: Make NullView
		}
	}

	void App::run()
	{
		// Run pathplanning threads
		m_running.store(true);
		std::unordered_set<const model::Cone*> detectedCones = m_perception->detect(m_vehicle->getPose());
		//std::function<void(int)> pathPlanningLambda = ;
		std::vector<std::thread> threads;
		startPlanningThreads(m_threadCount, threads, [this, &detectedCones](int index) {
			pathPlanningWorker(detectedCones, index);
			});

		// Run the game loop
		double updateTime = 0;
		SimpleTimer frameTimer;
		int iter = 0;
		while (m_view->isOpen()) {
			double deltaTime = frameTimer.elapsedSeconds();
			// Update the vehicle
			// Locked basically the whole loop, maybe it doesn't need to be like this, but this is fast to compute
			// View reads user input for pathPlanning
			// Sets VehicleState ✓ safe
			// Gets plannedPath ✓ safe
			// Gets detectedCones ✓ safe
			// Sets detectedCones ✓ safe
			// Modifies m_vehicle ✓ safe
			// View reads vehicle?
			if (m_view) {
				m_view->pollEvents();
			}
			std::deque<model::Path> pathCopy(5);
			{
				std::lock_guard<std::mutex> lock(m_simLock);
				frameTimer.reset();
				// Update the state
				m_vehicle->update(model::clamp(deltaTime,m_frameTime,m_frameTime*5));
				// Safely copy for viewing
				if (m_view) {
					pathCopy=m_vehicle->getPlannedPaths().get();
				}

				// Detect for mapping
				detectedCones= m_perception->detect(m_vehicle->getPose());

				// Calc goal for path planning
				double wayPoint1Distance = 10;
				double wayPoint2Distance = 50;
				std::unique_ptr<model::IVehicleState> stateCopy=m_vehicle->getStateCopy();
				bool foundCurrent1 = calcGoal(detectedCones, *stateCopy, m_sharedGoals.goals[0], wayPoint1Distance);
				bool foundCurrent2 = calcGoal(detectedCones, *stateCopy, m_sharedGoals.goals[1], wayPoint2Distance);
				if (foundCurrent1 || foundCurrent2) {
					m_sharedGoals.goalHasChanged.store(true);
				}

			}
			if (m_view) {
				m_view->setPath(pathCopy);
				m_view->setConeDetectedFlag(detectedCones);
				// Render the simulation
				m_view->render();
			}
			if (!m_view)
			{
				std::this_thread::sleep_for(std::chrono::duration<double>(m_frameTime - deltaTime));
			}
			iter++;
		}
		m_running.store(false);
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
	// Modifies m_vehicle ?
	void App::pathPlanningWorker(std::unordered_set<const model::Cone*>& detectedCones, int const index)
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
			std::lock_guard<std::mutex> lock(m_simLock);
			goal1 = m_vehicle->getPosition();
		} 
		goal2 = goal1;

		model::VehicleState stateCopy;
		double filterParam = 0.25;
		bool foundPath = false;
		while (m_running.load(std::memory_order_relaxed)) {

			// Read data from main
			std::unordered_set<const model::Cone*> detectedCopy;
			std::unique_ptr<model::IVehicleState> stateCopy;				
			model::Point previous1(goal1);
			model::Point previous2(goal2);

			{
				std::lock_guard<std::mutex> lock(m_simLock);
				detectedCopy = detectedCones;
				stateCopy = m_vehicle->getStateCopy();			

				goal1 = m_sharedGoals.goals[0];
				goal2 = m_sharedGoals.goals[1];
			} // End of reading



			// Bit of a hack to get the furthest 2 cones middle point
			goal1 = (1 - filterParam) * goal1 + filterParam * previous1;
			goal2 = (1 - filterParam) * goal2 + filterParam * previous2;
			if ((stateCopy->getPosition() - goal1).magnitude() <= (stateCopy->getPosition() - goal2).magnitude()) {
				m_vehicle->setGoal(goal1, index);
				m_vehicle->setGoal(goal2, index);
			}
			else {
				m_vehicle->setGoal(goal2, index);
				m_vehicle->setGoal(goal1, index);
			}
			#ifdef ENABLE_DEBUG_DRAW
			view::DebugDraw::instance().circle(goal1, 0.5, sf::Color::Cyan);
			view::DebugDraw::instance().circle(goal2, 0.5, sf::Color::Green);
			#endif // ENABLE_DEBUG_DRAW

			// Dealing with the current pathPlanner
			std::chrono::duration<double> dur(0);
			// Only plan if goal was changed
			if (/*m_sharedGoals.goalHasChanged.load()*/true) {
				// Clear residual data before planning new one
				m_vehicle->clearPath(index);
				// Plan inside timer
				dur = model::timeFunction("Path planning", [this, &detectedCopy, &stateCopy, &index, &foundPath]() {
						foundPath = m_vehicle->planPath(detectedCopy, *stateCopy, index);
						// Sending data back to main
						{
							std::lock_guard<std::mutex> lock(m_simLock);
							m_vehicle->setPlannedPath(index);
						} // End of sending data
					});
				m_sharedGoals.goalHasChanged.store(false);			
			}// End of dealing with the current pathPlanner

			time += dur.count();
			sortedTime.emplace_back(dur.count());
			double delayScnd = 0.005;
			double sleepDur = std::max(delayScnd * m_threadCount * (double)(index+1.0) / (double)m_threadCount-dur.count(), delayScnd); // Spread out thread execution over time
			std::this_thread::sleep_for(std::chrono::duration<double>(sleepDur)); // Sleep to avoid pointless CPU overload.
			iter++;
		}
		std::ranges::sort(sortedTime);
		std::lock_guard<std::mutex> lock(m_pathLock);
		std::cout << "________________\n" <<
			"THREAD ID: " << std::this_thread::get_id() << "\n" <<
			"Avg. path planning time: " << time / iter << "\n" <<
			"Path planning iterations: " << iter << "\n" <<
			"Slowest: " << sortedTime.back() << "\n" <<
			"Slowest (99th percentile): " << sortedTime[static_cast<size_t>(sortedTime.size() * 99/100)] << "\n" <<
			"Slowest (9th percentile): " << sortedTime[static_cast<size_t>(sortedTime.size() * 9/10)] << "\n";

	}

	void controller::App::startPlanningThreads(int threadCount, std::vector<std::thread>& threads, std::function<void(int)> const& loopLambda)
	{
		// Assuming one planner already exists
		for (int i = 0; i < threadCount - 1; i++) {
			m_vehicle->addPlanner();
		}
		// Starting threads
		for (int i = 0; i < threadCount; i++) {
			threads.emplace_back([loopLambda, i]() { loopLambda(i); });
		}
	}
	App::~App() = default;

	void controller::App::handleInputEvent(std::string const& src, controller::events::InputEvent const& e)
	{
		// Log
		std::visit(InputTranslate(), e);
		// Events
		double rotateSpeed = 6.0;
		std::visit(Overload{ 
			[this](controller::events::PressedEsc const&) {m_view->close(); },
			//[this](controller::events::ClickedAt const& click) {
			//		std::lock_guard<std::mutex> lock(m_simLock);
			//		#ifdef ENABLE_DEBUG_DRAW
			//		view::DebugDraw::instance().circle(model::Point(click.x, click.y), 0.5, sf::Color::Red);
			//		#endif // ENABLE_DEBUG_DRAW

			//		m_vehicle->setAllGoals(model::Point(click.x,click.y));
			//		},
			[this](controller::events::Scrolled const& s) {m_view->zoom(1 - s.delta * 0.25); },
			[this,  rotateSpeed](controller::events::RightClickDown const& c) {
				double delta = c.currentX- c.lastX;
				m_view->rotate(delta*rotateSpeed*m_view->getFrameTime()); 
			},
			[](auto&&) {}
			}, e);
	}
	bool controller::App::calcGoal(std::unordered_set<const model::Cone*> const& cones, model::IVehicleState const& state, model::Point& currentGoal, double maxDist)
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
			model::Point offset(m_vehicle->getLength()/2* cos(direction), m_vehicle->getLength()/2 * sin(direction));
			currentGoal=maxRCone->getPosition() + offset;

			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW
			return true;
		}
		if (maxLCone) 
		{
			model::Angle direction = state.getOrientation() - std::numbers::pi / 2.0;
			model::Point offset(m_vehicle->getLength()/2 * cos(direction), m_vehicle->getLength()/2 * sin(direction));
			currentGoal=maxLCone->getPosition()+offset;

			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW
			return true;
		}
		return false;
	}

}