// world_mesh.h - turns the World description (buildings, ground patches) into triangle lists.
// All buildings are baked into ONE static mesh in world space, so the whole city costs a few
// draw calls (and can be frustum culled in chunks) instead of one draw call per building.
#pragma once
#include <vector>
#include "engine/vertex.h"
#include "world/airport.h"

namespace WorldMesh {
    // Every building (walls, flat or gable roof). Vertex layout for the facade shader:
    //   uv    = (metres along the wall, metres above the ground)
    //   color = wall / roof colour
    //   extra = (wall height in metres, window style + seed fraction); style 0 = no windows
    std::vector<Vertex> structures(const World& world);

    // Flat ground patches, one triangle list per layer (0 = asphalt, 1 = lots, 2 = markings)
    std::vector<std::vector<Vertex>> ground(const World& world);
}
