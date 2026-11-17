// vertex.h - vertex layout shared by every mesh (no OpenGL dependency)
#pragma once
#include <glm/glm.hpp>

struct Vertex {
    glm::vec3 pos;     // layout(location = 0)
    glm::vec3 normal;  // layout(location = 1)
    glm::vec2 uv;      // layout(location = 2)
    glm::vec3 color;   // layout(location = 3)  (per-vertex colour, used by the terrain and buildings)
    glm::vec2 extra{0.0f}; // layout(location = 4) (buildings: x = wall height in m, y = window style + seed)
};
