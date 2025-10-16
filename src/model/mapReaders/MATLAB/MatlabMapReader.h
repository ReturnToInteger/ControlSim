#include "model/mapReaders/IMapReader.h"
#include "model/mapReaders/Map.h"
#include "model/items/Cone.h"

namespace model {
	class MatlabMapReader :
		public IMapReader 
	{
	public:
		MatlabMapReader(const char * filePath);
		// Inherited via IMapReader
		model::Map Read() override;
	private:
		const char * m_filePath;
	};
}