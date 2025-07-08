#pragma once
#include "model/controllerLogic/IControllerLogic.h"
#include "model/controllerLogic/ControlCommand.h"

namespace model {
    class Path;
    class VehicleState;
    class IVehicleState;
    struct Pose;

    class PurePursuitControl :
        public IControllerLogic
    {
    public:
        PurePursuitControl(int lookAhead = 1) : _lookAhead(lookAhead), _previous( 0,0 ) {}
        ControlCommand drive(VehicleState const& state, model::Path const& path) override;
        ControlCommand drive(model::diffDrive::State const& state, model::Path const& path) override;
        ~PurePursuitControl() = default;
        static constexpr double defaultSpeedInput = .2;
    private:
        IVehicleState const* _getAtRange(Pose const& vehiclePose, Path const& path) const;
        static Pose _rearFromCenter(IVehicleState const& center, double wheelbase);
        int _lookAhead;
        ControlCommand _previous;
        double _wheelbase;
        double _maxSteeringAngle;

    };
}
