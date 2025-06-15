#pragma once
#include <vector>
#include <SFML/Graphics.hpp>
#include "model/utils/Point.h"
#include "model/utils/FixSizedQueue.h"
#include <mutex>

namespace view {
    class DebugDraw : public sf::Drawable {
    public:
        static DebugDraw& instance() {
            static DebugDraw inst;
            return inst;
        }
        DebugDraw() : _circles(2) {}
        void line(sf::Vector2f a, sf::Vector2f b, sf::Color color = sf::Color::Red) {
            std::lock_guard<std::mutex> lock(_mutex);

            sf::VertexArray va(sf::Lines, 2);
            va[0].position = a;
            va[0].color = color;
            va[1].position = b;
            va[1].color = color;
            _lines.push_back(va);
        }
        void points(std::vector<model::Point> points, sf::Color color = sf::Color::Green) {
            std::lock_guard<std::mutex> lock(_mutex);

            _points.clear();
            _points.setPrimitiveType(sf::Points);

            for (auto const& point : points) {
                _points.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.X()), static_cast<float>(point.Y())), color));
            }
        }
        void points2(std::vector<model::Point> points, sf::Color color = sf::Color::Red) {
            std::lock_guard<std::mutex> lock(_mutex);

            _points2.clear();
            _points2.setPrimitiveType(sf::Points);

            for (auto const& point : points) {
                _points2.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.X()), static_cast<float>(point.Y())), color));
            }
        }
        void points3(std::vector<model::Point> points, sf::Color color = sf::Color::Red) {
            std::lock_guard<std::mutex> lock(_mutex);

            _points3.clear();
            _points3.setPrimitiveType(sf::Points);

            for (auto const& point : points) {
                _points3.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.X()), static_cast<float>(point.Y())), color));
            }
        }
        void lineStrip(std::vector<model::Point> points, sf::Color color = {255,255,255,127}) {
            std::lock_guard<std::mutex> lock(_mutex);

            _lineStrip.clear();
            _lineStrip.setPrimitiveType(sf::LineStrip);

            for (auto const& point : points) {
                _lineStrip.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.X()), static_cast<float>(point.Y())), color));
            }
        }

        void circle(model::Point center, float radius, sf::Color color = sf::Color::Green) {
            std::lock_guard<std::mutex> lock(_mutex);

            sf::CircleShape c(radius);
            c.setPosition(sf::Vector2f(static_cast<float>(center.X()), static_cast<float>(center.Y())) - sf::Vector2f(radius, radius));
            c.setFillColor(sf::Color::Transparent);
            c.setOutlineColor(color);
            c.setOutlineThickness(0.5f);
            _circles.push(c);
        }
        void circle2(model::Point center, float radius, sf::Color color = sf::Color::Red) {
            std::lock_guard<std::mutex> lock(_mutex);

            sf::CircleShape c(radius);
            c.setPosition(sf::Vector2f(static_cast<float>(center.X()), static_cast<float>(center.Y())) - sf::Vector2f(radius, radius));
            c.setFillColor(sf::Color::Transparent);
            c.setOutlineColor(color);
            c.setOutlineThickness(0.1f);
            _circle2=c;
        }

        void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
            std::lock_guard<std::mutex> lock(_mutex);
            for (auto& l : _lines) target.draw(l);
            for (auto& c: _circles.get()) {
                target.draw(c);
            }
            target.draw(_points);
            target.draw(_points2);
            target.draw(_points3);
            target.draw(_lineStrip);
            target.draw(_circle2);
        }

    private:
        void clear() {
            _lines.clear();
        }
        mutable std::mutex _mutex;

        std::vector<sf::VertexArray> _lines;
        model::FixSizedQueue <sf::CircleShape> _circles;
        sf::CircleShape _circle2;
        sf::VertexArray _points;
        sf::VertexArray _points2;
        sf::VertexArray _points3;
        sf::VertexArray _lineStrip;
        
    };
}