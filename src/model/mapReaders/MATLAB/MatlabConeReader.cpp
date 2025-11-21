#include "MatlabConeReader.h"
#include <iostream>
#include <string>
#include <stdexcept>
#include "mat.h"


namespace model {
    model::MatlabConeReader::MatlabConeReader(const char * filePath) : m_filePath(filePath)
    {
    }

    model::Map model::MatlabConeReader::Read()
    {
        mxArray* ConePosXLft, * ConePosYLft, * ConePosXRgt, * ConePosYRgt/*, * radius*/;
        //const char* file = "TestTrack.mat";


        std::cout << "Reading...\n";

        /*
         * Open file
         */
        MATFile* pmat = matOpen(m_filePath, "r");
        if (pmat == nullptr) {
            throw std::ios_base::failure("Could not open file: " + std::string(m_filePath));
        }

        /*
         * Read in each array
         */
        ConePosXLft = matGetVariable(pmat, "ConePosXLft");
        if (ConePosXLft == nullptr) {
            throw std::runtime_error("Error reading variable\n");
        }
        if (!mxIsNumeric(ConePosXLft)) {
            throw std::invalid_argument("ConePos must be numeric");
        }
        ConePosYLft = matGetVariable(pmat, "ConePosYLft");
        if (ConePosYLft == nullptr) {
            throw std::runtime_error("Error reading variable\n");
        }
        if (!mxIsNumeric(ConePosYLft)) {
            throw std::invalid_argument("ConePos must be numeric");
        }
        ConePosXRgt = matGetVariable(pmat, "ConePosXRgt");
        if (ConePosXRgt == nullptr) {
            throw std::runtime_error("Error reading variable\n");
        }
        if (!mxIsNumeric(ConePosXRgt)) {
            throw std::invalid_argument("ConePos must be numeric");
        }
        ConePosYRgt = matGetVariable(pmat, "ConePosYRgt");
        if (ConePosYRgt == nullptr) {
            throw std::runtime_error("Error reading variable\n");
        }
        if (!mxIsNumeric(ConePosYRgt)) {
            throw std::invalid_argument("ConePos must be numeric");
        }

        model::Map map;
        double* x = mxGetPr(ConePosXLft);
        double* y = mxGetPr(ConePosYLft);
        size_t vSize = mxGetNumberOfElements(ConePosXLft);
        if (vSize != mxGetNumberOfElements(ConePosYLft)) { throw std::invalid_argument("X and Y must be the same size."); }

        //printf("Length: %zu\n", vSize);
        for (size_t i = 0; i < vSize; i++) {
            map.addObstacle(std::make_unique<Cone>(*x, *y, defaultRadius, ConeType::LEFT));
            x++;
            y++;
        }
        x = mxGetPr(ConePosXRgt);
        y = mxGetPr(ConePosYRgt);
        vSize = mxGetNumberOfElements(ConePosXRgt);
        if (vSize != mxGetNumberOfElements(ConePosYRgt)) { throw std::invalid_argument("X and Y must be the same size."); }

        //printf("Length: %zu\n", vSize);
        for (size_t i = 0; i < vSize; i++) {
            map.addObstacle(std::make_unique<Cone>(*x, *y, defaultRadius, ConeType::RIGHT));
            x++;
            y++;
        }


        /* clean up before exit */
        mxDestroyArray(ConePosXLft);
        mxDestroyArray(ConePosYLft);

        if (matClose(pmat) != 0) {
            throw std::ios_base::failure("Error closing the file");
        }
        std::cout <<  "Done\n";
        return map;
        //return std::vector<model::Cone>();
    }

} 