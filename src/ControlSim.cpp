#include <SFML/Graphics.hpp>
#include <iostream>
#include <cstdlib>


#include "model/utils/Point.h"
#include "model/items/Cone.h"

#include "model/Vehicle.h"
#include "controller/App.h"
#include "view/AppView.h"

#include "model/mapReaders/ManualMapReader.h"
#include "model/mapReaders/MATLAB/MatlabMapReader.h"
#include "controller/KeyboardControl.h"
#include "model/controllerLogic/PurePursuitControl.h"
#include "model/pathPlanning/PathPlanner.h"
#include "model/utils/ModelUtils.h"
double Mod(double a, double b) {
	return std::fmod(std::fmod(a, b) + b, b);
}

int main() {

	model::pathPlanning::PlannerConfig plannerConfig={ 0.2,1.5,20,model::pathPlanning::SteeringMode::EXTREME };
	
	controller::App app(
		std::make_unique<model::Vehicle>(std::make_unique<model::PurePursuitControl>(), 
			std::make_unique<model::pathPlanning::PathPlanner>(plannerConfig)),
		std::make_unique<model::MatlabMapReader>("TestTrack.mat"),
		std::make_unique<view::AppView>() /*nullptr*/
	);
	app.run();

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

	//std::array<model::Cone, 1> map({model::Cone(40,10,1.5)});
	//map[0].setType(model::ConeType::UNKNOWN);
	//controller::App app(
	//	std::make_unique<model::Vehicle>(std::make_unique<model::PurePursuitControl>(), std::make_unique<model::pathPlanning::PathPlanner>(60, 3.0, 1.0)),
	//	std::make_unique<model::ManualMapReader<std::array<model::Cone,1>>>(map),
	//	std::make_unique<view::AppView>()
	//);
	//app.run();




}