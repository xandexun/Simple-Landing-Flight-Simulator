// frustum.h - view frustum culling (6 planes extracted from a projection*view matrix)
#pragma once
#include <glm/glm.hpp>

struct Frustum {
    glm::vec4 p[6];   // normalised planes (nx, ny, nz, d): inside when dot(n, x) + d >= 0

    explicit Frustum(const glm::mat4& m) {
        auto row = [&m](int r) { return glm::vec4(m[0][r], m[1][r], m[2][r], m[3][r]); };
        glm::vec4 r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
        p[0] = r3 + r0;   // left
        p[1] = r3 - r0;   // right
        p[2] = r3 + r1;   // bottom
        p[3] = r3 - r1;   // top
        p[4] = r3 + r2;   // near
        p[5] = r3 - r2;   // far
        for (glm::vec4& plane : p) plane = plane / glm::length(glm::vec3(plane));
    }

    bool intersectsAABB(const glm::vec3& bmin, const glm::vec3& bmax) const {
        for (int i = 0; i < 6; ++i) {
            glm::vec3 pv(p[i].x >= 0.0f ? bmax.x : bmin.x, p[i].y >= 0.0f ? bmax.y : bmin.y,
                         p[i].z >= 0.0f ? bmax.z : bmin.z);          // corner furthest along the normal
            if (p[i].x * pv.x + p[i].y * pv.y + p[i].z * pv.z + p[i].w < 0.0f) return false;
        }
        return true;
    }
};
