#pragma once
#ifdef ENABLE_DEBUG_DRAW
#include <vector>
#include <SFML/Graphics.hpp>
#include "model/utils/Point.h"
#include "model/utils/FixSizedQueue.h"
#include <mutex>

namespace view {
    class DebugDraw : public sf::Drawable {
    public:
        static DebugDraw& instance() {
            static DebugDraw s_inst;
            return s_inst;
        }
        DebugDraw() : m_circles(2) {}
        void line(sf::Vector2f a, sf::Vector2f b, sf::Color color = sf::Color::Red) {
            std::lock_guard<std::mutex> lock(m_mutex);

            sf::VertexArray va(sf::Lines, 2);
            va[0].position = a;
            va[0].color = color;
            va[1].position = b;
            va[1].color = color;
            m_lines.push_back(va);
        }
        void points(std::vector<model::Point> points, sf::Color color = sf::Color::Green) {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_points.clear();
            m_points.setPrimitiveType(sf::Points);

            for (auto const& point : points) {
                m_points.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.X()), static_cast<float>(point.Y())), color));
            }
        }
        void points2(std::vector<model::Point> points, sf::Color color = sf::Color::Red) {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_points2.clear();
            m_points2.setPrimitiveType(sf::Points);

            for (auto const& point : points) {
                m_points2.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.X()), static_cast<float>(point.Y())), color));
            }
        }
        void points3(std::vector<model::Point> points, sf::Color color = sf::Color::Red) {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_points3.clear();
            m_points3.setPrimitiveType(sf::Points);

            for (auto const& point : points) {
                m_points3.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.X()), static_cast<float>(point.Y())), color));
            }
        }
        void lineStrip(std::vector<model::Point> points, sf::Color color = {255,255,255,127}) {
            std::lock_guard<std::mutex> lock(m_mutex);

            m_lineStrip.clear();
            m_lineStrip.setPrimitiveType(sf::LineStrip);

            for (auto const& point : points) {
                m_lineStrip.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.X()), static_cast<float>(point.Y())), color));
            }
        }

        void circle(model::Point center, float radius, sf::Color color = sf::Color::Green) {
            std::lock_guard<std::mutex> lock(m_mutex);

            sf::CircleShape c(radius);
            c.setPosition(sf::Vector2f(static_cast<float>(center.X()), static_cast<float>(center.Y())) - sf::Vector2f(radius, radius));
            c.setFillColor(sf::Color::Transparent);
            c.setOutlineColor(color);
            c.setOutlineThickness(0.5f);
            m_circles.push(c);
        }
        void circle2(model::Point center, float radius, sf::Color color = sf::Color::Red) {
            std::lock_guard<std::mutex> lock(m_mutex);

            sf::CircleShape c(radius);
            c.setPosition(sf::Vector2f(static_cast<float>(center.X()), static_cast<float>(center.Y())) - sf::Vector2f(radius, radius));
            c.setFillColor(sf::Color::Transparent);
            c.setOutlineColor(color);
            c.setOutlineThickness(0.1f);
            m_circle2=c;
        }

        void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
            std::lock_guard<std::mutex> lock(m_mutex);
            for (auto& l : m_lines) target.draw(l);
            for (auto& c: m_circles.get()) {
                target.draw(c);
            }
            target.draw(m_points);
            target.draw(m_points2);
            target.draw(m_points3);
            target.draw(m_lineStrip);
            target.draw(m_circle2);
        }

    private:
        mutable std::mutex m_mutex;

        std::vector<sf::VertexArray> m_lines;
        model::FixSizedQueue <sf::CircleShape> m_circles;
        sf::CircleShape m_circle2;
        sf::VertexArray m_points;
        sf::VertexArray m_points2;
        sf::VertexArray m_points3;
        sf::VertexArray m_lineStrip;
        
    };
}


#endif