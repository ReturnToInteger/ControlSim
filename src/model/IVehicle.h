#pragma once
#include "model/items/Item.h"
#include "model/items/Path.h"
#include "model/utils/Pose.h"
#include "model/utils/Angle.h"
#include "model/utils/Point.h"
#include "model/utils/FixSizedQueue.h"
#include "model/utils/IDrawableVehicle.h"
#include <memory>
#include <vector>
#include <unordered_set>
#include <optional>
#include <type_traits>


namespace model {
	class IVehicleState;
	class Perception;
	class Cone;
	namespace pathPlanning {
		class PathPlanner;
	}
	class IVehicle : public Item, public IDrawableVehicle {
	public:
		virtual ~IVehicle() = default;

		virtual Point getPosition() const override = 0;

		// state and control methods

		virtual Pose getPose() const = 0;
		virtual Angle getOrientation() const = 0;
		virtual double getLength() const = 0;
		virtual double getWidth() const = 0;
		virtual double getSpeed() const = 0;
		virtual std::unique_ptr<IVehicleState> getStateCopy() const = 0;

		virtual void update(double dt) = 0;
		virtual void setPose(double x, double y, std::optional<double> orientation) = 0;

		// path planning methods

		virtual FixSizedQueue<Path> getPlannedPaths() const = 0;
		virtual std::vector<model::Point> getGoal() const = 0;
		virtual double getCellSize(int i = 0) const = 0;

		virtual void clearPath(int i) = 0;
		virtual bool planPath(std::unordered_set<const model::Cone*> const& cones, IVehicleState const& state, int i) = 0;
		virtual void setPlannedPath(int i) = 0;
		virtual void setGoal(model::Point goal, int i) = 0;
		virtual void setAllGoals(model::Point goal) = 0;
		virtual void addPlanner() = 0;
		
		static constexpr std::size_t maxHistorySize = 5;
	};
}