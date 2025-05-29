#pragma once
#include "model/controllerLogic/IControllerLogic.h"
#include "model/controllerLogic/ControlCommand.h"
#include "model/VehicleState.h"
#include "model/pathPlanner/PathPlanner.h"
#include "model/utils/ModelUtils.h"
#include <iostream>

namespace model {
    class AIControl :
        public model::IControllerLogic
    {
    public:
		AIControl(int lookAhead = 1) : _lookAhead(lookAhead) {}
        model::ControlCommand drive(const model::VehicleState & state, const model::PathPlanner & pathPlanner) override;

        const model::Pose * getAtRange(const model::Pose& vehiclePose, const std::vector<model::Pose>& path) const;
    private:
        int _lookAhead;
    };
}
