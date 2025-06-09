#pragma once
#include "model/items/Item.h"
#include "model/utils/Point.h"
#include "model/VehicleState.h"
#include <vector>

namespace model {        
    // Read-only vector<VehicleState> wrapper
    class Path :
        public Item
    {
    public:
        // Inherited via Item
        Point getPosition() const override;
        Path(std::vector<VehicleState> const& path);
        Path() : _path() {}


        VehicleState const& operator[](size_t index) const { return _path[index]; }
        std::vector<VehicleState>::const_iterator begin() const { return _path.begin(); }
        std::vector<VehicleState>::const_iterator end() const { return _path.end(); }
        size_t size() const { return _path.size(); }
        VehicleState const& back() const { return _path.back(); }

    private:        
        std::vector<VehicleState> _path;
    };
}
