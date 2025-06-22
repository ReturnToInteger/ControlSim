
//#include "model/mapReaders/ManualMapReader.h"
//#include "model/items/Cone.h"

#include "model/Vehicle.h"
#include "controller/App.h"
#include "view/AppView.h"

#include "model/mapReaders/MATLAB/MatlabMapReader.h"
#include "controller/KeyboardControl.h"
#include "model/controllerLogic/PurePursuitControl.h"
#include "model/pathPlanning/PathPlanner.h"
//#include <iostream>
//#include <cstdlib>
//#include <new> // for std::size_t
//#include <atomic>
//#include <thread>
//#include "model/pathPlanning/PathNode.h"
//
//std::atomic<size_t> g_allocCount = 0;
//constexpr size_t MAX_LOGGED_ALLOCS = 5000;
//
//
//void* operator new(std::size_t size) {
//	if (g_allocCount++ < MAX_LOGGED_ALLOCS) {
//		std::cout << "[global new on thread " << std::this_thread::get_id() << "] " << size << " bytes\n";
//	}
//	return std::malloc(size);
//}
//void operator delete(void* ptr) noexcept {
//	if (g_allocCount++ < MAX_LOGGED_ALLOCS) {
//		std::cout << "[global delete on thread " << std::this_thread::get_id() << "]\n";
//	}
//	std::free(ptr);
//}
//void* operator new[](std::size_t size) {
//	if (g_allocCount++ < MAX_LOGGED_ALLOCS) {
//		std::cout << "[global new[] on thread " << std::this_thread::get_id() << "] " << size << " bytes\n";
//	}
//	return std::malloc(size);
//}
//void operator delete[](void* ptr) noexcept {
//	if (g_allocCount++ < MAX_LOGGED_ALLOCS) {
//		std::cout << "[global delete[] on thread " << std::this_thread::get_id() << "]\n";
//	}
//	std::free(ptr);
//}

double Mod(double a, double b) {
	return std::fmod(std::fmod(a, b) + b, b);
}

int main() {
	std::cout << "Size of PathNode: " << sizeof(model::PathNode) << std::endl;
	model::pathPlanning::PlannerConfig plannerConfig={ 
		0.2,
		1.5,
		20,
		model::pathPlanning::SteeringMode::HIGH, 
		model::pathPlanning::Waypoints::DOUBLE
	};
	
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
