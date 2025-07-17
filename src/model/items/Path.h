#pragma once
#include "model/items/Item.h"
#include "model/utils/Point.h"
#include "model/IVehicleState.h"   // interface header
#include <vector>
#include <memory>

namespace model {
    // Read-only vector<IVehicleState> wrapper
    class Path : public Item {
    public:
        // Inherited via Item
        Point getPosition() const override;

        // Construct from vector of unique_ptr<IVehicleState>
        Path(std::vector<std::unique_ptr<IVehicleState>>&& path)
            : m_path(std::move(path)) 
        {
            if (m_path.empty())
            {
                std::cout << "No path created\n";
            }

        }
        Path() = default;
        // Copy constructor
        Path(const Path& other) {
            m_path.reserve(other.m_path.size());
            for (const auto& ptr : other.m_path) {
                m_path.push_back(ptr ? ptr->clone() : nullptr);
            }
        }

        // Copy assignment
        Path& operator=(const Path& other) {
            if (this != &other) {
                std::vector<std::unique_ptr<IVehicleState>> newPath;
                newPath.reserve(other.m_path.size());
                for (const auto& ptr : other.m_path) {
                    newPath.push_back(ptr ? ptr->clone() : nullptr);
                }
                m_path = std::move(newPath);
            }
            return *this;
        }

        // Enable moving (default move constructor and move assignment)
        Path(Path&&) noexcept = default;
        Path& operator=(Path&&) noexcept = default;

        ~Path() = default;
        // Access elements as const IVehicleState references
        const IVehicleState& operator[](size_t index) const { return *m_path[index]; }

        // Iterators returning const pointers or references
        std::vector<std::unique_ptr<IVehicleState>>::const_iterator begin() const { return m_path.begin(); }
        std::vector<std::unique_ptr<IVehicleState>>::const_iterator end() const { return m_path.end(); }

        size_t size() const { return m_path.size(); }

        const IVehicleState& back() const { return *m_path.back(); }

    private:
        std::vector<std::unique_ptr<IVehicleState>> m_path;
    };
}
