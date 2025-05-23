#include <SFML/Graphics.hpp>
#include "model/Vehicle.h"
#include "controller/KeyboardControl.h"
#include <iostream>
#include "model/Point.h"
#include "model/Cone.h"
#include "model/App.h"
#include "model/mapReaders/IMapReader.h"
#include "view/AppView.h"
#include "model/mapReaders/ManualMapReader.h"

double Mod(double a, double b) {
	return std::fmod(std::fmod(a, b) + b, b);
}

int main() {

	std::array<model::Cone, 18> map({
		model::Cone(0, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(0, 1.5, 0.15,model::ConeType::RIGHT),
		model::Cone(5, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(5, 1.5, 0.15,model::ConeType::RIGHT),
		model::Cone(10, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(10, 1.5, 0.15,model::ConeType::RIGHT),
		model::Cone(15, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(15, 1.5, 0.15,model::ConeType::RIGHT),
		model::Cone(20, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(20, 1.5, 0.15,model::ConeType::RIGHT),
		model::Cone(25, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(25, 1.5, 0.15,model::ConeType::RIGHT),
		model::Cone(30, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(30, 1.5, 0.15,model::ConeType::RIGHT),
		model::Cone(35, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(35, 1.5, 0.15,model::ConeType::RIGHT),
		model::Cone(40, -1.5, 0.15,model::ConeType::LEFT),
		model::Cone(40, 1.5, 0.15,model::ConeType::RIGHT),
		});

	//std::array<model::Cone, 0> map({});
	
	model::App app(
		std::make_unique<model::Vehicle>(std::make_unique<controller::KeyboardControl>(),std::make_unique<model::PathPlanner>(40,1.5,1/15.0)),
		std::make_unique<model::ManualMapReader<std::array<model::Cone, 18>>>(map),
		std::make_unique<view::AppView>()
	);

	app.run();
	
}