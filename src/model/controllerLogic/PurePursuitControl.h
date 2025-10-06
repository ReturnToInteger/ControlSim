#pragma once
#include "model/controllerLogic/IControllerLogic.h"
#include "model/controllerLogic/ControlCommand.h"

namespace model {
    class Path;
    class VehicleState;
    class IVehicleState;
    struct Pose;

    class PurePursuitControl :
        public IControllerLogic<VehicleState>
    {
    public:
        PurePursuitControl(int lookAhead = 1) : m_lookAhead(lookAhead), m_previous( 0,0 ) {}
        ControlCommand drive(VehicleState const& state, model::Path const& path) override;
        ~PurePursuitControl() = default;
        static constexpr double defaultSpeedInput = .2;
    private:
        IVehicleState const* getAtRange(Pose const& vehiclePose, Path const& path) const;
        static Pose rearFromCenter(IVehicleState const& center, double wheelbase);
        int m_lookAhead;
        ControlCommand m_previous;
        double m_wheelbase;
        double m_maxSteeringAngle;

    };
}
