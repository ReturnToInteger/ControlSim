#include "BitmapReader.h"
#include <SFML/Graphics.hpp>
#include "model/mapReaders/Map.h"
#include "model/utils/EDT.h"
#include <vector>

model::BitmapReader::BitmapReader(const char* filePath) : m_filePath(filePath)
{
}

model::Map model::BitmapReader::Read()
{
    sf::Image image;
    if (!image.loadFromFile(m_filePath))
        throw std::runtime_error("Failed to load BMP");

    auto size = image.getSize();
	std::vector<std::vector<uint8_t>> occupancy(size.y,std::vector<uint8_t>(size.x));
    for (unsigned y = 0; y < size.y; ++y)
        for (unsigned x = 0; x < size.x; ++x) {
            sf::Color c = image.getPixel(x, y);
            occupancy[y][x] = (c.r < 128);
        }
    std::vector<std::vector<float>> dist(size.y, std::vector<float>(size.x));
    model::edt2D(occupancy, dist, size.x, size.y, 0.2);

}
