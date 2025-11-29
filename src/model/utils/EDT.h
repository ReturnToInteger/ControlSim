#pragma once
#include <vector>
#include <cmath>
#include <limits>
namespace model {

    // 1D squared Euclidean distance transform (Felzenszwalb–Huttenlocher)
    void edt1D(const std::vector<float>& source,
        std::vector<float>& out,
        int length)
    {
        // Parabola centers that form the lower envelope
        std::vector<int> parabolaIndex(length);

        // X-coordinates where we switch between parabolas
        std::vector<float> parabolaBound(length + 1);

        int envelopeSize = 0; // how many parabolas in the envelope

        parabolaIndex[0] = 0;
        parabolaBound[0] = -std::numeric_limits<float>::infinity();
        parabolaBound[1] = std::numeric_limits<float>::infinity();

        // Build lower envelope
        for (int q = 1; q < length; q++)
        {
            float intersection;

            // Calculate intersection with the last parabola in the envelope
            do {
                int prev = parabolaIndex[envelopeSize];
                intersection =
                    ((source[q] + q * q) - (source[prev] + prev * prev)) /
                    (2.0f * q - 2.0f * prev);

                // If intersection happens before the previous boundary,
                // the last parabola becomes useless
                if (intersection <= parabolaBound[envelopeSize])
                    envelopeSize--;
            } while (intersection <= parabolaBound[envelopeSize]);

            envelopeSize++;
            parabolaIndex[envelopeSize] = q;
            parabolaBound[envelopeSize] = intersection;
            parabolaBound[envelopeSize + 1] = std::numeric_limits<float>::infinity();
        }

        // Evaluate envelope
        int k = 0;
        for (int x = 0; x < length; x++)
        {
            while (parabolaBound[k + 1] < x)
                k++;

            int center = parabolaIndex[k];
            out[x] = (x - center) * (x - center) + source[center];
        }
    }


    void edt2D(const std::vector<std::vector<uint8_t>>& occ,
        std::vector<std::vector<float>>& dist,
        int w, int h,
        float cellSize)
    {
        const float INF = 1e10f;

        std::vector<float> tmp(std::max(w, h));
        std::vector<std::vector<float>> vert(h, std::vector<float>(w));

        // Column pass
        for (int x = 0; x < w; x++) {
            for (int y = 0; y < h; y++)
                tmp[y] = occ[y][x] ? 0.0f : INF;

            std::vector<float> out(h);
            edt1D(tmp, out, h);

            for (int y = 0; y < h; y++)
                vert[y][x] = out[y];
        }

        // Row pass
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++)
                tmp[x] = vert[y][x];

            edt1D(tmp, dist[y], w);

            for (int x = 0; x < w; x++)
                dist[y][x] = std::sqrt(dist[y][x]) * cellSize;
        }
    }
}