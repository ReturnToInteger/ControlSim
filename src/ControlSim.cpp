#include <SFML/Graphics.hpp>
#include "model/Vehicle.h"
#include "controller/KeyboardControl.h"
#include <iostream>
#include "model/utils/Point.h"
#include "model/items/Cone.h"
#include "model/App.h"
#include "model/mapReaders/IMapReader.h"
#include "view/AppView.h"
#include "model/mapReaders/ManualMapReader.h"
#include "model/controllerLogic/AIControl.h"
#include "model/mapReaders/MatlabMapReader.h"

double Mod(double a, double b) {
	return std::fmod(std::fmod(a, b) + b, b);
}

int main() {

	//model::App app(
	//	std::make_unique<model::Vehicle>(std::make_unique<model::AIControl>(2), std::make_unique<model::PathPlanner>(60, 3.0, 1 / 8.0)),
	//	std::make_unique<model::MatlabMapReader>("TestTrack.mat"),
	//	std::make_unique<view::AppView>()
	//);
	//app.run();

	//std::array<model::Cone, 18> map({
	//	model::Cone(0, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(0, 1.5, 0.15,model::ConeType::RIGHT),
	//	model::Cone(5, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(5, 1.5, 0.15,model::ConeType::RIGHT),
	//	model::Cone(10, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(10, 1.5, 0.15,model::ConeType::RIGHT),
	//	model::Cone(15, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(15, 1.5, 0.15,model::ConeType::RIGHT),
	//	model::Cone(20, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(20, 1.5, 0.15,model::ConeType::RIGHT),
	//	model::Cone(25, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(25, 1.5, 0.15,model::ConeType::RIGHT),
	//	model::Cone(30, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(30, 1.5, 0.15,model::ConeType::RIGHT),
	//	model::Cone(35, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(35, 1.5, 0.15,model::ConeType::RIGHT),
	//	model::Cone(40, -1.5, 0.15,model::ConeType::LEFT),
	//	model::Cone(40, 1.5, 0.15,model::ConeType::RIGHT),
	//	});

	std::array<model::Cone, 1> map({model::Cone(40,10,1.5)});
	map[0].setType(model::ConeType::UNKNOWN);
	model::App app(
		std::make_unique<model::Vehicle>(std::make_unique<model::AIControl>(), std::make_unique<model::PathPlanner>(60, 3.0, 1 / 8.0)),
		std::make_unique<model::ManualMapReader<std::array<model::Cone,1>>>(map),
		std::make_unique<view::AppView>()
	);
	app.run();




}