#pragma once
#include "model/items/Item.h"
#include "model/utils/Point.h"
#include "model/utils/Pose.h"
#include <vector>
//#include <model/pathPlanner/PathNode.h>
namespace model {
    class Path :
        public Item
    {
    public:
        // Inherited via Item
        Point getPosition() const override;
        Path() : _path() {}
        Path(const std::vector<Pose>& path) : _path(path) {}

        // Vector wrapper
        //Pose& operator[](size_t index) { return _path[index]; }
        const Pose& operator[](size_t index) const { return _path[index]; }
        //std::vector<Pose>::iterator begin() { return _path.begin(); }
        //std::vector<Pose>::iterator end() { return _path.end(); }
        std::vector<Pose>::const_iterator begin() const { return _path.begin(); }
        std::vector<Pose>::const_iterator end() const { return _path.end(); }
        size_t size() const { return _path.size(); }

    private:
        std::vector<Pose> _path;
    };
}
