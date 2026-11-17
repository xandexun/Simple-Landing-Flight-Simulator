// terrain.h - procedural terrain (grass plain around the airport, mountains further away)
#pragma once
#include <vector>
#include "engine/vertex.h"

namespace Terrain {
    // Height of the ground (metres) at world position (x, z). Flat near the runway and
    // along the approach corridor, hills and mountains further away.
    float height(float x, float z);

    // Builds a regular grid of triangles (non-indexed, ready for glDrawArrays).
    // size = side length in metres, n = number of cells per side.
    std::vector<Vertex> buildMesh(float centerX, float centerZ, float size, int n);
}
