#pragma once
#include "model/items/Item.h"
#include <model/pathPlanner/PathNode.h>
namespace model {
    class Path :
        public Item
    {
    public:
        // Inherited via Item
        Point getPosition() const override;
    private:
        std::vector<PathNode> path;
    };
}
