#include "IMapReader.h"
#include "model/items/Cone.h"

namespace model {
	class MatlabMapReader :
		public IMapReader 
	{
	public:
		MatlabMapReader(const char * filePath);
		// Inherited via IMapReader
		std::vector<Cone> Read() override;
	private:
		const char * _filePath;
	};
}