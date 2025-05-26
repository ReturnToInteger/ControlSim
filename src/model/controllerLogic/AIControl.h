#pragma once
#include "src/model/controllerLogic/IControllerLogic.h"
#include "src/model/controllerLogic/ControlCommand.h"
#include "src/model/VehicleState.h"
#include "src/model/pathPlanner/PathPlanner.h"
#include "src/model/utils/ModelUtils.h"

namespace model {
    class AIControl :
        public model::IControllerLogic
    {
    public:
		AIControl(int lookAhead = 1) : _lookAhead(lookAhead) {}
        model::ControlCommand drive(const model::VehicleState & state, const model::PathPlanner & pathPlanner) override;
    private:
        int _lookAhead;
    };
}
