#pragma once
#include "src/model/controllerLogic/IControllerLogic.h"
#include "src/model/VehicleState.h"
#include <src/model/pathPlanner/PathPlanner.h>

namespace model {
    class AIControl :
        public model::IControllerLogic
    {
    public:
		AIControl(int lookAhead = 1) : _lookAhead(lookAhead) {}
        void drive(model::VehicleState& state, model::PathPlanner& pathPlanner) override;
    private:
        int _lookAhead;
    };
}
