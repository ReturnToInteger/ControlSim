#include "BitmapReader.h"
#include <SFML/Graphics.hpp>
#include "model/mapReaders/Map.h"

model::BitmapReader::BitmapReader(const char* filePath) : m_filePath(filePath)
{
}

model::Map model::BitmapReader::Read()
{
	model::Map map;
    sf::Image image;
    if (!image.loadFromFile(path))
        throw std::runtime_error("Failed to load BMP");

    auto size = image.getSize();
    for (unsigned y = 0; y < size.y; ++y)
        for (unsigned x = 0; x < size.x; ++x) {
            sf::Color c = image.getPixel(x, y);
            if (c == sf::Color::Black)
                map.addObstacle(x, y);
        }

}
