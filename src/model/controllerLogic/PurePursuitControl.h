#pragma once
#include "model/controllerLogic/IControllerLogic.h"
#include "model/controllerLogic/ControlCommand.h"

namespace model {
    class Path;
    class VehicleState;
    struct Pose;

    class PurePursuitControl :
        public IControllerLogic
    {
    public:
        PurePursuitControl(int lookAhead = 1) : _lookAhead(lookAhead), _previous( 0,0 ) {}
        ControlCommand drive(VehicleState const& state, model::Path const& path) override;
        ~PurePursuitControl() = default;
        static constexpr double defaultSpeedInput = .2;
    private:
        VehicleState const* _getAtRange(Pose const& vehiclePose, Path const& path) const;
        int _lookAhead;
        ControlCommand _previous;
    };
}
