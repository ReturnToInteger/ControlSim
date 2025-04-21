#include <SFML/Graphics.hpp>
#include "model/Vehicle.h"
#include <iostream>

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
	double zoom = 80.0;
	model::Vehicle vehicle(400/zoom, 300/zoom,0.0,0.0);
	
	int iter = 0;
	double steeringRate = 0;
	bool isSteering = false;
	// Main loop
	while (window.isOpen()) {
		sf::Event event;
		while (window.pollEvent(event)) {
			if (event.type == sf::Event::Closed)
				window.close();

			if (event.type == sf::Event::KeyPressed) {
				if (event.key.code == sf::Keyboard::Escape)
					window.close();
				if (event.key.code == sf::Keyboard::W) {
					vehicle.accelerateTo(vehicle.getMaxSpeed());
				}
				if (event.key.code == sf::Keyboard::S) {
					vehicle.accelerateTo(-vehicle.getMaxSpeed());
				}
				if (event.key.code == sf::Keyboard::A) {
					vehicle.steer(-1000);
				}
				if (event.key.code == sf::Keyboard::D) {
					vehicle.steer(1000);
				}
			}
		}
		if (event.type == sf::Event::KeyReleased) {
			if (event.key.code == sf::Keyboard::A || event.key.code == sf::Keyboard::D) {
				vehicle.steer(0);
			}
			if (event.key.code == sf::Keyboard::W || event.key.code == sf::Keyboard::S) {
				vehicle.accelerateTo(0);
			}
		}
		// Update vehicle state
		vehicle.updateState(dt); // Assuming a frame rate of 60 FPS
		// Clear the window
		window.clear(sf::Color::Black);
		// Draw the vehicle (for demonstration purposes)
		//vehicle.steer(1000*(sin(iter*0.01)>0 ? 1:-1));
		//vehicle.accelerateTo(0.01*iter);
		auto positions = vehicle.getAllPositions();
		auto orientations = vehicle.getAllOrientations();
		std::array<sf::Color, 3> shapeColor = { sf::Color::Green, sf::Color::Red,sf::Color::Red };
		std::array<std::array<double, 2>, 3> shapeSize{};
		shapeSize[0] = { vehicle.getLength(),vehicle.getWidth() / 6 };
		shapeSize[1] = { vehicle.getLength() / 2,vehicle.getWidth() / 8 };
		shapeSize[2] = shapeSize[1];
		for (int i = 0; i < 3;i++) {
			sf::RectangleShape shape(sf::Vector2f(shapeSize[i][0] * zoom, shapeSize[i][1] * zoom));
			shape.setOrigin(shapeSize[i][0] * zoom / 2, shapeSize[i][1] * zoom / 2);
			model::Point realPos = positions[i]*zoom;
			realPos.setX(Mod(realPos.X(), videoWidth));
			realPos.setY(Mod(realPos.Y(), videoHeight));

			shape.setFillColor(shapeColor[i]);
			shape.setPosition(realPos.X(), realPos.Y());
			shape.setRotation(orientations[i]/M_PI*180);
			//std::cout << "Vehicle Position: " << positions[0].X() << ", " << positions[0].Y() << std::endl;
			window.draw(shape);
		}
		// Display the contents of the window
		window.display();
		iter++;
	}
}