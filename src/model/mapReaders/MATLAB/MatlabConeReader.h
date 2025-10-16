#include "model/mapReaders/IMapReader.h"
#include "model/mapReaders/Map.h"
#include "model/items/Cone.h"

namespace model {
	class MatlabConeReader :
		public IMapReader 
	{
	public:
		MatlabConeReader(const char * filePath);
		// Inherited via IMapReader
		model::Map Read() override;
	private:
		const char * m_filePath;
	};
}