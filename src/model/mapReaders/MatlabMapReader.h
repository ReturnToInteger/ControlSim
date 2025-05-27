#include "IMapReader.h"
#include "src/model/items/Cone.h"
#include "mat.h"
#include <string>

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