#include "MatlabMapReader.h"
#include <iostream>
#include <string>
#include <stdexcept>
#include "mat.h"


namespace model {
    model::MatlabMapReader::MatlabMapReader(const char * filePath) : _filePath(filePath)
    {
    }

    std::vector<model::Cone> model::MatlabMapReader::Read()
    {
        MATFile* pmat;
        mxArray* ConePosXLft, * ConePosYLft, * ConePosXRgt, * ConePosYRgt, * radius;
        //const char* file = "TestTrack.mat";


        std::cout << "Reading..." << std::endl;

        /*
         * Open file
         */
        pmat = matOpen(_filePath, "r");
        if (pmat == nullptr) {
            throw std::ios_base::failure("Could not open file: " + std::string(_filePath));
        }

        /*
         * Read in each array
         */
        ConePosXLft = matGetVariable(pmat, "ConePosXLft");
        if (ConePosXLft == NULL) {
            throw std::runtime_error("Error reading variable\n");
        }
        if (!mxIsNumeric(ConePosXLft)) {
            throw std::invalid_argument("ConePos must be numeric");
        }
        ConePosYLft = matGetVariable(pmat, "ConePosYLft");
        if (ConePosYLft == NULL) {
            throw std::runtime_error("Error reading variable\n");
        }
        if (!mxIsNumeric(ConePosYLft)) {
            throw std::invalid_argument("ConePos must be numeric");
        }
        ConePosXRgt = matGetVariable(pmat, "ConePosXRgt");
        if (ConePosXRgt == NULL) {
            throw std::runtime_error("Error reading variable\n");
        }
        if (!mxIsNumeric(ConePosXRgt)) {
            throw std::invalid_argument("ConePos must be numeric");
        }
        ConePosYRgt = matGetVariable(pmat, "ConePosYRgt");
        if (ConePosYRgt == NULL) {
            throw std::runtime_error("Error reading variable\n");
        }
        if (!mxIsNumeric(ConePosYRgt)) {
            throw std::invalid_argument("ConePos must be numeric");
        }

        std::vector<model::Cone> map;
        double* x = mxGetPr(ConePosXLft);
        double* y = mxGetPr(ConePosYLft);
        size_t vSize = mxGetNumberOfElements(ConePosXLft);
        if (vSize != mxGetNumberOfElements(ConePosYLft)) { throw std::invalid_argument("X and Y must be the same size."); }

        //printf("Length: %zu\n", vSize);
        for (size_t i = 0; i < vSize; i++) {
            map.emplace_back(*x, *y, 0.15, ConeType::LEFT);
            x++;
            y++;
        }
        x = mxGetPr(ConePosXRgt);
        y = mxGetPr(ConePosYRgt);
        vSize = mxGetNumberOfElements(ConePosXRgt);
        if (vSize != mxGetNumberOfElements(ConePosYRgt)) { throw std::invalid_argument("X and Y must be the same size."); }

        //printf("Length: %zu\n", vSize);
        for (size_t i = 0; i < vSize; i++) {
            map.emplace_back(*x, *y, 0.15, ConeType::RIGHT);
            x++;
            y++;
        }


        /* clean up before exit */
        mxDestroyArray(ConePosXLft);
        mxDestroyArray(ConePosYLft);

        if (matClose(pmat) != 0) {
            throw std::ios_base::failure("Error closing the file");
        }
        printf("Done\n");
        return map;
        //return std::vector<model::Cone>();
    }

} 