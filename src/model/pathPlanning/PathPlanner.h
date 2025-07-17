#pragma once  
#include <vector>  
#include <iostream>  
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <array>

#include "model/IVehicleState.h"
#include "model/pathPlanning/PathNode.h"
#include "model/pathPlanning/heuristics/DubinsStateSpace.h"
#include "model/items/Path.h"
#include "model/utils/FixSizedQueue.h"

namespace model {
	class Cone;
	namespace pathPlanning {
		enum class SteeringMode : std::uint8_t
		{
			LOW,
			MEDIUM,
			HIGH,
			SQUARED
		}; 

		enum class Waypoints : std::uint8_t
		{
			SINGLE,
			DOUBLE
		};

		struct SteeringPresets {
			static constexpr std::array<double, 5> steeringModeLOW = { 0, -0.5, 0.5, -1, 1 };
			static constexpr std::array<double, 9> steeringModeMEDIUM = { 0, -0.25, 0.25, -0.5, 0.5, -0.75, 0.75, -1, 1 };
			static constexpr std::array<double, 11> steeringModeHIGH = { 0.0, 0.2, -0.2, 0.4, -0.4, 0.6, -0.6, 0.8, -0.8, 1.0, -1.0 };
			static constexpr std::array<double, 9> steeringModeSQUARED = { 0.0, 0.06, -0.06, 0.25, -0.25, 0.56, -0.56, 1.0, -1.0 };
		};
		static constexpr int defaultContainerSize = 200;

		// cellSize
		// stepSize
		// angleBins
		// SteeringMode: 
		// --- linear=LOW, MEDIUM, HIGH
		// --- higher density near smaller angles: SQUARED
		// Waypoints: SINGLE - single goal planning. DOUBLE - goal + waypoint planning. WARNING: Not tested in SINGLE mode currently
		// maxContainerSize: limit the visited and unvisited nodes
		struct PlannerConfig {
			double cellSize;
			double stepSize;
			int angleBins;
			SteeringMode steeringMode;
			// Not tested in SINGLE mode currently
			Waypoints waypoints;
			int maxContainerSize = defaultContainerSize;
		};

		class PathPlanner
		{
		public:
			PathPlanner(PlannerConfig config);
			bool planPath(std::unordered_set<const model::Cone*> const& cones, model::IVehicleState const& vehicleState);
			//Sets planned path based on current node
			void setPlannedPath();
			const Path& getPlannedPath() const;
			void setGoal(Point const& goal);
			void clear();
			[[nodiscard]] double getCellSize() const { return m_cellSize; }
            static Point rotatePoint(model::Point const& point, Angle const& angle);


		private:
			// Storing nodes based on discrete pair of coordinates (x,y)
			std::unordered_map<std::tuple<int, int, int,int>, PathNode> m_openList;
			std::priority_queue<PQNode,
				std::vector<PQNode>,
				std::greater<>> m_openQueue;
			std::unordered_set<std::tuple<int, int, int,int>> m_closedList;
			#ifdef ENABLE_DEBUG_DRAW
			std::unordered_set<std::tuple<int, int, int,int>> m_collidingList;
			#endif // ENABLE_DEBUG_DRAW
			Path m_plannedPath;
			int m_angleBins;
			double m_stepSize;
			double m_cellSize;
			const double* m_steeringInputs = nullptr;
			int m_maxContainerSize;
			size_t m_anglesSize;

			FixSizedQueue<Point> m_goals;
			PathNode m_finalNode;
			DubinsStateSpace m_dubins;

			static void selectSteeringMode(SteeringMode const& mode, double const*& steeringInputs, size_t& size);
			void updateNeightbours(std::unordered_set<const model::Cone*> const& cones, PathNode const& node, int stage);
			static std::unique_ptr<IVehicleState> stepByDistance(IVehicleState const& state, double distance, double steeringInput);
			[[nodiscard]] std::tuple<int, int, int,int> discretizePoint(model::Pose const& pose, int stage) const;
			// Returns if it collides with 100% accuracy, and vehicle pose
			// More computationally expensive
			[[nodiscard]] std::pair<bool, model::Pose> detectCollision(IVehicleState const& state, std::unordered_set<const model::Cone*> const& cones) const;

			// Simple collision detection that uses vehicle width
			// Less computationally expensive
			[[nodiscard]] std::pair<bool, model::Pose> lazyDetectCollision(IVehicleState const& state, std::unordered_set<const model::Cone*> const& cones) const;

			//		Detect collision between a node and one step away by interpolating
			// Returns the first collision if it happens, and the vehicle's pose
			// If there is no collision, the return is the vehicle's new pose
			std::pair<bool, model::Pose> checkCollisionWithinStep(int stepCount, IVehicleState const& state, std::unordered_set<const model::Cone*> const& cones);

			PathNode createNewNode(PathNode const& node, double steeringInput, int stage) const;
			void processValidNode(PathNode & node, std::tuple<int, int, int,int> const& key, int stage);
			
			[[nodiscard]] std::vector<model::Point> getBoundary(IVehicleState const& state) const;
			// Dubins
			double getHeuristics(model::IVehicleState const& start, model::Point const& goal);
			// Eucledian only
			double getHeuristics(model::Point const& startPoint, model::Point const& goalPoint);
			// Multiple waypoints
			double getHeuristics(model::IVehicleState const& start, model::Point const& wayPoint, model::Point const& goalPoint);

			// gets position-orientation pairs until new position is reached
			std::vector<std::unique_ptr<model::IVehicleState>>  stepUntilNew(IVehicleState const& state, double distanceStep) const;

			[[nodiscard]] bool isAtGoal(const model::IVehicleState& state, int index) const;
			[[nodiscard]] bool hasMoreNodes() const;
			PQNode popAndCloseNextNode();

			
			void setDubins(double minTurningRadius);

		};
	}
}
