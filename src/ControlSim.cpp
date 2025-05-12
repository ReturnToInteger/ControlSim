#include <SFML/Graphics.hpp>
#include "model/Vehicle.h"
#include "controller/KeyboardControl.h"
#include <iostream>
#include "model/Point.h"
#include "model/Cone.h"

double Mod(double a, double b) {
	return std::fmod(std::fmod(a, b) + b, b);
}

int main() {
	// Create a window
	int videoWidth = 1600, videoHeight = 900;
	sf::RenderWindow window(sf::VideoMode(videoWidth, videoHeight), "Vehicle Simulation");
	int frameRate = 144;
	double dt = 1.0 / frameRate;
	window.setFramerateLimit(144);
	// Create a vehicle
	double zoom = 25.0;
	model::Point startPos(videoWidth / 2, videoHeight / 2);
	//model::Vehicle vehicle(videoWidth /zoom/2, videoHeight/zoom/2,0.0,0.0);
	model::Vehicle vehicle(std::make_unique<controller::KeyboardControl>());

	std::array<model::Cone, 18> map({
		model::Cone(0, 0, 0.5),
		model::Cone(0, 6, 0.5),
		model::Cone(8, 0, 0.5),
		model::Cone(8, 6, 0.5),
		model::Cone(16, 0, 0.5),
		model::Cone(16, 6, 0.5),
		model::Cone(24, 0, 0.5),
		model::Cone(24, 6, 0.5),
		model::Cone(32, 0, 0.5),
		model::Cone(32, 6, 0.5),
		model::Cone(40, 0, 0.5),
		model::Cone(40, 6, 0.5),
		model::Cone(48, 0, 0.5),
		model::Cone(48, 6, 0.5),
		model::Cone(56, 0, 0.5),
		model::Cone(56, 6, 0.5),
		model::Cone(64, 0, 0.5),
		model::Cone(64, 6, 0.5),
		});

	
	// Main loop
	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed)
				window.close();

			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Escape)
					window.close();
			}
		}
		//// Steering
		//vehicle.steer();

		//// Acceleration
		//vehicle.drive();

		// Update vehicle state
		vehicle.update(dt);
		// Clear the window
		window.clear(sf::Color::Black);
		//// Draw the vehicle (for demonstration purposes)
		//auto positions = vehicle.getAllPositions();
		//auto orientations = vehicle.getAllOrientations();
		//std::array<sf::Color, 3> shapeColor = { sf::Color::Green, sf::Color::Red,sf::Color::Red };
		//std::array<std::array<double, 2>, 3> shapeSize{};
		//shapeSize[0] = { vehicle.getLength(),vehicle.getWidth() };
		//shapeSize[1] = { vehicle.getWidth() * 0.75,vehicle.getWidth()*1.5 };
		//shapeSize[2] = shapeSize[1];
		//for (int i = 0; i < 3;i++) {
		//	sf::RectangleShape shape(sf::Vector2f(shapeSize[i][0] * zoom, shapeSize[i][1] * zoom));
		//	shape.setOrigin(shapeSize[i][0] * zoom / 2, shapeSize[i][1] * zoom / 2);
		//	model::Point realPos = (positions[i]-positions[1]) * zoom + startPos;
		//	realPos.setX(Mod(realPos.X(), videoWidth));
		//	realPos.setY(Mod(realPos.Y(), videoHeight));

		//	shape.setFillColor(shapeColor[i]);
		//	shape.setPosition(realPos.X(), realPos.Y());
		//	shape.setRotation(orientations[i]/M_PI*180);
		//	//std::cout << "Vehicle Position: " << positions[0].X() << ", " << positions[0].Y() << std::endl;
		//	window.draw(shape);
		//}
		// Draw the vehicle
		auto position = vehicle.getPosition();

		sf::CircleShape vehicleShape(0.7 * zoom);
		vehicleShape.setFillColor(sf::Color::Red);
		vehicleShape.setPosition(videoWidth / 2, videoHeight / 2);
		window.draw(vehicleShape);

		// Draw the map
		for (auto& cone : map) {
			sf::CircleShape shape(0.5 * zoom);
			//model::Point realPos = (cone.getPosition() - positions[1] )* zoom + startPos;
			model::Point realPos = (cone.getPosition() - position) * zoom + startPos;
			realPos.setX(Mod(realPos.X(), videoWidth));
			realPos.setY(Mod(realPos.Y(), videoHeight));
			shape.setFillColor(sf::Color::Blue);
			shape.setPosition(realPos.X(), realPos.Y());
			window.draw(shape);
		}
		// Display the contents of the window
		window.display();
	}
}