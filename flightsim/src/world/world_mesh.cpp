// world_mesh.cpp
#include "world/world_mesh.h"
#include <cmath>
#include <utility>

namespace {

void addBuilding(std::vector<Vertex>& out, const Building& b) {
    const float sy = std::sin(b.yaw), cy = std::cos(b.yaw);
    // local -> world (rotation around Y, then translation)
    auto W = [&](const glm::vec3& l) { return b.pos + glm::vec3(cy * l.x + sy * l.z, l.y, -sy * l.x + cy * l.z); };
    auto Nw = [&](const glm::vec3& n) { return glm::vec3(cy * n.x + sy * n.z, n.y, -sy * n.x + cy * n.z); };

    const float sx = b.size.x, H = b.size.y, sz = b.size.z, hx = sx * 0.5f, hz = sz * 0.5f;
    const float style = (b.windowMode > 0) ? float(b.windowMode) + glm::clamp(b.seed, 0.0f, 0.99f) : 0.0f;
    const glm::vec2 wallExtra(H, style), noExtra(0.0f, 0.0f);

    auto emit = [&](const glm::vec3& p, const glm::vec3& n, const glm::vec2& uv, const glm::vec3& col, const glm::vec2& ex) {
        out.push_back({W(p), Nw(n), uv, col, ex});
    };
    // quad a-b-c-d (counter-clockwise seen from outside) with a uv for each corner
    auto quad = [&](glm::vec3 a, glm::vec3 bb, glm::vec3 c, glm::vec3 d, const glm::vec3& n,
                    glm::vec2 ua, glm::vec2 ub, glm::vec2 uc, glm::vec2 ud, const glm::vec3& col, const glm::vec2& ex) {
        emit(a, n, ua, col, ex); emit(bb, n, ub, col, ex); emit(c, n, uc, col, ex);
        emit(a, n, ua, col, ex); emit(c, n, uc, col, ex); emit(d, n, ud, col, ex);
    };

    // four walls
    quad({-hx, 0, hz}, {hx, 0, hz}, {hx, H, hz}, {-hx, H, hz}, {0, 0, 1},
         {0, 0}, {sx, 0}, {sx, H}, {0, H}, b.color, wallExtra);
    quad({hx, 0, -hz}, {-hx, 0, -hz}, {-hx, H, -hz}, {hx, H, -hz}, {0, 0, -1},
         {sx, 0}, {0, 0}, {0, H}, {sx, H}, b.color, wallExtra);
    quad({hx, 0, hz}, {hx, 0, -hz}, {hx, H, -hz}, {hx, H, hz}, {1, 0, 0},
         {sz, 0}, {0, 0}, {0, H}, {sz, H}, b.color, wallExtra);
    quad({-hx, 0, -hz}, {-hx, 0, hz}, {-hx, H, hz}, {-hx, H, -hz}, {-1, 0, 0},
         {0, 0}, {sz, 0}, {sz, H}, {0, H}, b.color, wallExtra);

    if (b.roofHeight <= 0.0f) {
        // flat roof: darker version of the wall colour
        glm::vec3 roof = b.color * 0.45f + glm::vec3(0.12f);
        quad({-hx, H, hz}, {hx, H, hz}, {hx, H, -hz}, {-hx, H, -hz}, {0, 1, 0},
             {0, 0}, {1, 0}, {1, 1}, {0, 1}, roof, noExtra);
    } else {
        // gable roof with a ridge along the local X axis and a small overhang
        const float rh = b.roofHeight, rx = hx * 1.04f, rz = hz * 1.08f, y0 = H;
        glm::vec3 nF = glm::normalize(glm::vec3(0.0f, rz, rh)), nB = glm::normalize(glm::vec3(0.0f, rz, -rh));
        quad({-rx, y0, rz}, {rx, y0, rz}, {rx, y0 + rh, 0}, {-rx, y0 + rh, 0}, nF,
             {0, 0}, {1, 0}, {1, 1}, {0, 1}, b.roofColor, noExtra);
        quad({rx, y0, -rz}, {-rx, y0, -rz}, {-rx, y0 + rh, 0}, {rx, y0 + rh, 0}, nB,
             {0, 0}, {1, 0}, {1, 1}, {0, 1}, b.roofColor, noExtra);
        emit({rx, y0, rz}, {1, 0, 0}, {0, 0}, b.color, wallExtra);                // gables (wall colour)
        emit({rx, y0, -rz}, {1, 0, 0}, {1, 0}, b.color, wallExtra);
        emit({rx, y0 + rh, 0}, {1, 0, 0}, {0.5f, 1}, b.color, wallExtra);
        emit({-rx, y0, -rz}, {-1, 0, 0}, {0, 0}, b.color, wallExtra);
        emit({-rx, y0, rz}, {-1, 0, 0}, {1, 0}, b.color, wallExtra);
        emit({-rx, y0 + rh, 0}, {-1, 0, 0}, {0.5f, 1}, b.color, wallExtra);
    }
}

} // namespace

namespace WorldMesh {

std::vector<Vertex> structures(const World& world) {
    std::vector<Vertex> out;
    out.reserve(world.buildings.size() * 40);
    for (const Building& b : world.buildings) addBuilding(out, b);
    return out;
}

std::vector<std::vector<Vertex>> ground(const World& world) {
    std::vector<std::vector<Vertex>> layers(3);
    for (const GroundPatch& g : world.ground) {
        // two triangles with the same winding as the other meshes (front face seen from above)
        glm::vec2 q[4] = {g.c[0], g.c[1], g.c[2], g.c[3]};
        float area = 0.0f;
        for (int i = 0; i < 4; ++i) area += q[i].x * q[(i + 1) % 4].y - q[(i + 1) % 4].x * q[i].y;
        if (area > 0.0f) std::swap(q[1], q[3]);
        for (int i : {0, 1, 2, 0, 2, 3})
            layers[glm::clamp(g.layer, 0, 2)].push_back({glm::vec3(q[i].x, 0.0f, q[i].y), glm::vec3(0.0f, 1.0f, 0.0f),
                                                         glm::vec2(q[i].x / 50.0f, q[i].y / 50.0f), g.color});
    }
    return layers;
}

} // namespace WorldMesh
