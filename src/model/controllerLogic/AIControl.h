#pragma once
#include "model/controllerLogic/IControllerLogic.h"
#include "model/controllerLogic/ControlCommand.h"

namespace model {
    struct model::ControlCommand;
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
