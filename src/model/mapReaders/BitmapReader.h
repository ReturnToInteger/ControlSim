#pragma once

#include "model/mapReaders/IMapReader.h"
#include "model/mapReaders/Map.h"

namespace model {
	class BitmapReader :
		public IMapReader
	{
	public:
		BitmapReader(const char* filePath);
		// Inherited via IMapReader
		model::Map Read() override;
	private:
		const char* m_filePath;
	};
}