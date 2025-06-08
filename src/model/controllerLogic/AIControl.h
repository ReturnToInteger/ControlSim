#pragma once
#include "model/controllerLogic/IControllerLogic.h"
#include "model/controllerLogic/ControlCommand.h"

namespace model {
    class Path;
    class VehicleState;
    struct Pose;
    namespace pathPlanning {
        class PathPlanner;
    }
    class AIControl :
        public IControllerLogic
    {
    public:
		AIControl(int lookAhead = 1) : _lookAhead(lookAhead) {}
        ControlCommand drive(VehicleState const& state, pathPlanning::PathPlanner const& pathPlanner) override;

    private:
        Pose const* _getAtRange(Pose const& vehiclePose, Path const& path) const;
        int _lookAhead;
    };
}
