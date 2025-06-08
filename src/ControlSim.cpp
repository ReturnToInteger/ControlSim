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
#include "model/controllerLogic/AIControl.h"
//#include "model/pathPlanning/Heuristics/DubinsStateSpace.h"
#include "model/utils/ModelUtils.h"
double Mod(double a, double b) {
	return std::fmod(std::fmod(a, b) + b, b);
}

int main() {

	//double R_turning = 3 / tan(M_PI / 6.0);
	//model::DubinsStateSpace space(R_turning);
	//std::cout <<"Turning distance: " << R_turning << std::endl;
	//auto durMultiple=model::timeFunction("Dubins distance", [&space]() {
	//	for (int y = 0; y < 1000; y++) {
	//		for (int x = 0; x < 1000; x++) {
	//			int div = 24;
	//			for (int a = 0; a < div; a++) {
	//				space.simpleDistance(model::Pose(0, 0, 0), model::Point(double(x) * 0.5, double(y) * 0.5) );
	//			}
	//		}
	//	}
	//	space.distance(model::Pose(0, 0, 0), model::Pose(40, 0, M_PI / 2));
	//	});
	//std::cout << "Duration: " << durMultiple.count() << std::endl
	//	<< "Avg: " << durMultiple.count() / (1000 * 1000 * 24) << std::endl;
	
	//double distance=0;
	//auto dur = model::timeFunction("Dubins distance", [&space, &distance]() {
	//	distance = space.simpleDistance(model::Pose(0, 0, 0), model::Point(1, -5.0));
	//	});
	//std::cout<< "Distance: "<< distance<< std::endl;
	//std::cout << "Duration: " << dur.count() << std::endl;

	controller::App app(
		std::make_unique<model::Vehicle>(std::make_unique<model::AIControl>(3), std::make_unique<model::pathPlanning::PathPlanner>(60, 3.0, 0.2)),
		std::make_unique<model::MatlabMapReader>("TestTrack.mat"),
		std::make_unique<view::AppView>()
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
	//	std::make_unique<model::Vehicle>(std::make_unique<model::AIControl>(2), std::make_unique<model::PathPlanner>(60, 3.0, 1.0)),
	//	std::make_unique<model::ManualMapReader<std::array<model::Cone,1>>>(map),
	//	std::make_unique<view::AppView>()
	//);
	//app.run();




}