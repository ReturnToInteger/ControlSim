#pragma once
#include "model/controllerLogic/IControllerLogic.h"
#include "model/controllerLogic/ControlCommand.h"

namespace model {
    class Path;
    class VehicleState;
    namespace pathPlanning {
        class PathPlanner;
    }
    class AIControl :
        public IControllerLogic
    {
    public:
		AIControl(int lookAhead = 1) : _lookAhead(lookAhead) {}
        ControlCommand drive(const VehicleState & state, const pathPlanning::PathPlanner & pathPlanner) override;

    private:
        const Pose * _getAtRange(const Pose& vehiclePose, const Path& path) const;
        int _lookAhead;
    };
}
