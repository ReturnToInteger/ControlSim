#include "AppView.h" 
#include <cassert>
#include "model/items/Item.h"
#include "model/utils/IDrawableVehicle.h"
#include "VehicleView.h"

//#include "imgui.h"
//#include "imgui-sfml.h"
#ifdef ENABLE_DEBUG_DRAW
#include "DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW


namespace view {
	AppView::AppView()
		: m_videoWidth(DefaultAppViewConfig::width),
		m_videoHeight(DefaultAppViewConfig::height),
		m_frameRate(DefaultAppViewConfig::frameRate),
		m_zoom(DefaultAppViewConfig::zoom),
		m_lastX(-1),
		m_gridZoomLimit(defaultGridZoomLimit),
		m_cellSize(0)
	{
	}

	AppView::AppView(model::IDrawableVehicle const& vehicle, std::vector<model::Cone> const& map)
		: m_videoWidth(DefaultAppViewConfig::width),
		m_videoHeight(DefaultAppViewConfig::height),
		m_frameRate(DefaultAppViewConfig::frameRate),
		m_zoom(DefaultAppViewConfig::zoom),
		m_lastX(-1),
		m_vehicleView(vehicle, sf::Color::Red),
		m_gridZoomLimit(defaultGridZoomLimit),
		m_cellSize(0)
	{
		m_coneViews.reserve(map.size());
		for (const auto& cone : map) {
			m_coneViews.emplace_back(cone);
			m_itemViewTable.emplace(&cone, &m_coneViews.back());
		}
	}

	void AppView::init()
	{
		m_window.create(sf::VideoMode(m_videoWidth, m_videoHeight), "Vehicle Simulation");
		m_window.setFramerateLimit(m_frameRate);
		m_view.setSize(static_cast<float>(m_videoWidth), static_cast<float>(m_videoHeight));
		m_view.setCenter(m_vehicleView.getPosition());
		m_view.zoom(static_cast<float>(m_zoom));
		m_window.setView(m_view);
	}

	void AppView::render()
	{
		setupGrid(m_gridZoomLimit);
		m_window.clear(sf::Color::Black);
		m_view.setCenter(m_vehicleView.getPosition());
		m_window.setView(m_view);
		//Drawing
		drawGrid();
		#ifdef ENABLE_DEBUG_DRAW
		m_window.draw(view::DebugDraw::instance());
		#endif // ENABLE_DEBUG_DRAW

		for (auto const& coneView : m_coneViews) {
			m_window.draw(coneView);
		}
		for (auto& path : m_pathViews) {
			m_window.draw(path);
		}
		m_window.draw(m_vehicleView);
		//if (m_drawables) {
		//	for (auto const& drawable : m_drawables) {
		//		m_window.draw(drawable);
		//	}
		//}

		m_window.display();
	}

	void AppView::pollEvents()
	{
		sf::Event event;
		while (m_window.pollEvent(event)){
			if (event.type == sf::Event::Closed) 
			{
				m_window.close();
			}
			else if (event.type == sf::Event::Resized)
			{
				// update the view to the new size of the window
				auto resized = event.size;
				m_videoWidth = resized.width;
				m_videoHeight = resized.height;
				m_view.setSize(static_cast<float>(resized.width), static_cast<float>(resized.height));
				m_view.setCenter(m_vehicleView.getPosition());
				m_view.zoom(static_cast<float>(m_zoom));
				
				notify("AppView", model::events::Resized{});
			} 
			else
			{
				model::events::InputEvent e = translateEventToInput(event);
				if (!std::holds_alternative<model::events::None>(e))
				{
					notify("AppView", e);
				}
			}

		}

		// Poll keys not in event
		if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right)) 
		{
			sf::Vector2i pixelPos = sf::Mouse::getPosition(m_window);
			if (m_lastX < 0) 
			{
				m_lastX = pixelPos.x;
			}

			notify("AppView", model::events::RightClickDown{ .lastX= m_lastX,.currentX= (double)pixelPos.x });
			m_lastX = pixelPos.x;
		}
		else 
		{
			m_lastX = -1;
		}
	}

	bool AppView::isOpen() const
	{
		return m_window.isOpen();
	}

	double AppView::getFrameTime() const
	{
		return 1.0/(double) m_frameRate;
	}

	void AppView::close()
	{
		m_window.close();
	}

	void AppView::setVehicle(model::IDrawableVehicle const& vehicle)
	{
		m_vehicleView = view::VehicleView(vehicle, sf::Color::Red);
		m_view.setCenter(m_vehicleView.getPosition());
	}

	void AppView::setCones(std::vector<model::Cone> const& cones)
	{
		m_coneViews.clear();
		m_coneViews.reserve(cones.size());
		for (auto const& cone : cones) {
			m_coneViews.emplace_back(cone);	
			m_itemViewTable.emplace(&cone, &m_coneViews.back());
		}
	}

	void AppView::setPath(model::Path const& path)
	{
		m_pathViews = std::vector<PathView>{ PathView(path) };

	}

	void AppView::setPath(std::deque<model::Path> const& pathVector)
	{
		// TODO: Delete from itemViewTable
		m_pathViews.clear();
		if (pathVector.empty()) {
			m_pathViews.emplace_back();
			return;
		}
		for (auto const& path : pathVector) {
			m_pathViews.emplace_back(path);
			//m_itemViewTable.emplace(&path, m_pathViews.back());
		}
		m_pathViews.back().setColor(sf::Color::Green);

	}

	void AppView::setConeDetectedFlag(const std::unordered_set<const model::Cone*>& detectedCones)
	{
		for (auto& coneView : m_coneViews) {
			coneView.isDetected = false;
		}
		for (auto const& item : detectedCones) {
			m_itemViewTable[item]->isDetected = true;
		}
	}

	void AppView::zoom(double factor)
	{
		m_zoom = m_zoom * factor;
		//std::cout << delta << "\n";
		m_view.zoom(static_cast<float>(factor));

	}

	void AppView::rotate(double delta)
	{
		m_view.rotate(static_cast<float>(delta));
	}

	void AppView::setupGrid(double limit)
	{
		sf::Vector2f topLeft(m_window.mapPixelToCoords({ 0,0 }));
		sf::Vector2f bottomRight(m_window.mapPixelToCoords(sf::Vector2i( (int)m_videoWidth,(int)m_videoHeight )));
		if (m_cellSize / m_zoom < limit) {
			m_gridLines.clear();
			return;
		}
		double cellSize = m_cellSize;

		int startX = int(std::floor((topLeft.x - m_videoWidth) / cellSize));
		int endX = int((std::floor((bottomRight.x + m_videoWidth) / cellSize) + 1));
		int startY = int(std::floor((topLeft.y - m_videoHeight) / cellSize));
		int endY = int((std::floor((bottomRight.y + m_videoHeight) / cellSize) + 1));

		sf::VertexArray lines(sf::Lines);

		sf::Color lineColor(80, 80, 80); // subtle gray, TO DO: gridView class


		// vertical lines
		for (int x = startX; x <= endX; ++x) {
			auto px = static_cast<float>(x * cellSize);
			lines.append(sf::Vertex(sf::Vector2f(px, static_cast<float>(startY * cellSize)), lineColor));
			lines.append(sf::Vertex(sf::Vector2f(px, static_cast<float>(endY * cellSize)), lineColor));
		}

		// horizontal lines
		for (int y = startY; y <= endY; ++y) {
			auto py = static_cast<float>(y * cellSize);
			lines.append(sf::Vertex(sf::Vector2f(static_cast<float>(startX * cellSize), py), lineColor));
			lines.append(sf::Vertex(sf::Vector2f(static_cast<float>(endX * cellSize), py), lineColor));
		}
		m_gridLines = lines;
	}

	void AppView::drawGrid()
	{
		m_window.draw(m_gridLines);
	}

	model::events::InputEvent view::AppView::translateEventToInput(sf::Event event)
	{

		if (event.type == sf::Event::KeyPressed) 
		{
			if (event.key.code == sf::Keyboard::Escape)
			{
				return model::events::PressedEsc();
			}
				//m_window.close();
			if (event.key.code == sf::Keyboard::LShift) {
				//// Wait until released
				//while (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)) {
				//	// Do nothing
				//}
				return model::events::PressedLShift();
			}
		}
		if (event.type == sf::Event::MouseButtonPressed)
		{
			if (event.mouseButton.button == sf::Mouse::Left) {
				sf::Vector2i pixelPos = sf::Mouse::getPosition(m_window);

				sf::Vector2f worldPos = m_window.mapPixelToCoords(pixelPos);

				//std::cout << "Map coords (on button press):\n";
				//std::cout << "map x: " << worldPos.x << "\n";
				//std::cout << "map y: " << worldPos.y << "\n";
				//m_clickGlobalPos = model::Point(worldPos.x, worldPos.y);
				return model::events::ClickedAt{ .x = (double)worldPos.x, .y = (double)worldPos.y };
			}
		}
		if (event.type == sf::Event::MouseWheelMoved)
		{
			int delta =event.mouseWheel.delta;
			return model::events::Scrolled{ delta };
		}
		if (event.type == sf::Event::Resized) {
			return model::events::Resized();
		}		
		if (event.type == sf::Event::LostFocus) {
			return model::events::LostFocus();
		}
		if (event.type == sf::Event::GainedFocus) {
			return model::events::GainedFocus();
		}
		return model::events::None();
	}

//	void AppView::addDrawable(sf::Drawable& drawable)
//	{
//		m_drawables.emplace_back(drawable);
//	}
//
//	void AppView::clearDrawables()
//	{
//		m_drawables.clear();
//	}
//
}

