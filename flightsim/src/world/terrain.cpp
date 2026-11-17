// terrain.cpp - value-noise based height field
#include "world/terrain.h"
#include "world/airport.h"
#include <cmath>

namespace {

float hash2(int x, int z) {
    unsigned int h = static_cast<unsigned int>(x) * 374761393u + static_cast<unsigned int>(z) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return (h & 0xFFFFFF) / float(0xFFFFFF);   // [0,1]
}

float valueNoise(float x, float z) {
    int xi = static_cast<int>(std::floor(x)), zi = static_cast<int>(std::floor(z));
    float fx = x - xi, fz = z - zi;
    float ux = fx * fx * (3.0f - 2.0f * fx), uz = fz * fz * (3.0f - 2.0f * fz);
    float a = hash2(xi, zi), b = hash2(xi + 1, zi), c = hash2(xi, zi + 1), d = hash2(xi + 1, zi + 1);
    return glm::mix(glm::mix(a, b, ux), glm::mix(c, d, ux), uz);
}

// fractal Brownian motion, result roughly in [0,1]
float fbm(float x, float z, int octaves) {
    float sum = 0.0f, amp = 0.5f, norm = 0.0f;
    for (int i = 0; i < octaves; ++i) {
        sum += amp * valueNoise(x, z);
        norm += amp;
        x = x * 2.03f + 17.1f;
        z = z * 2.03f - 9.7f;
        amp *= 0.5f;
    }
    return sum / norm;
}

// Distance (m) to the "protected" area: runway + approach corridor (kept flat)
float distanceToCorridor(float x, float z) {
    float dx = std::fmax(std::fabs(x) - 450.0f, 0.0f);
    float dz = std::fmax(std::fmax(z - 3000.0f, -12000.0f - z), 0.0f);
    return std::sqrt(dx * dx + dz * dz);
}

} // namespace

namespace Terrain {

float height(float x, float z) {
    const float base = -0.3f;                  // just under the runway surface (y = 0)
    float d = distanceToCorridor(x, z);

    // gentle rolling hills a bit away from the airport
    float hills = (fbm(x / 900.0f, z / 900.0f, 4) - 0.35f) * 40.0f * glm::smoothstep(200.0f, 1400.0f, d);
    hills = std::fmax(hills, 0.0f);

    // mountains far away (ridged noise for sharper crests)
    float n = fbm(x / 3200.0f + 3.3f, z / 3200.0f - 1.7f, 6);
    float ridge = 1.0f - std::fabs(2.0f * n - 1.0f);
    float mountains = std::pow(ridge, 2.2f) * 1100.0f * glm::smoothstep(700.0f, 6500.0f, d);

    float h = base + hills + mountains;

    // flat ground under the city districts and the motorways, and around the airport landside
    for (const City::District& dist : City::districts())
        h = glm::mix(h, base, 1.0f - glm::smoothstep(1.15f, 1.6f, City::districtScore(dist, glm::vec2(x, z))));
    h = glm::mix(h, base, 1.0f - glm::smoothstep(150.0f, 500.0f, City::distanceToRoads(glm::vec2(x, z))));
    float dx = std::fmax(std::fmax(400.0f - x, x - 1050.0f), 0.0f), dz = std::fmax(std::fmax(-350.0f - z, z - 700.0f), 0.0f);
    h = glm::mix(h, base, 1.0f - glm::smoothstep(100.0f, 450.0f, std::sqrt(dx * dx + dz * dz)));
    return h;
}

std::vector<Vertex> buildMesh(float cx, float cz, float size, int n) {
    const float step = size / n;
    const float x0 = cx - size * 0.5f, z0 = cz - size * 0.5f;

    // sample heights, normals and colours on the grid points
    std::vector<Vertex> grid((n + 1) * (n + 1));
    auto idx = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            float x = x0 + i * step, z = z0 + j * step, h = height(x, z), e = step * 0.5f;
            float hL = height(x - e, z), hR = height(x + e, z), hD = height(x, z - e), hU = height(x, z + e);
            glm::vec3 nrm = glm::normalize(glm::vec3(hL - hR, 2.0f * e, hD - hU));
            float slope = 1.0f - nrm.y;                       // 0 = flat
            float v = fbm(x / 250.0f, z / 250.0f, 3);          // colour variation
            glm::vec3 grass = glm::mix(glm::vec3(0.20f, 0.36f, 0.11f), glm::vec3(0.33f, 0.45f, 0.16f), v);
            const glm::vec3 dryGrass(0.45f, 0.43f, 0.24f), rock(0.38f, 0.35f, 0.32f), snow(0.92f, 0.93f, 0.97f);
            glm::vec3 col = glm::mix(grass, dryGrass, glm::smoothstep(60.0f, 260.0f, h) * 0.8f);
            col = glm::mix(col, rock, glm::clamp(glm::smoothstep(0.12f, 0.35f, slope) + glm::smoothstep(350.0f, 600.0f, h), 0.0f, 1.0f));
            col = glm::mix(col, snow, glm::smoothstep(720.0f, 860.0f, h + v * 80.0f) * (1.0f - glm::smoothstep(0.35f, 0.6f, slope)));
            grid[idx(i, j)] = {glm::vec3(x, h, z), nrm, glm::vec2(x / 50.0f, z / 50.0f), col};
        }

    std::vector<Vertex> verts;
    verts.reserve(n * n * 6);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i)    // two counter-clockwise triangles (seen from above)
            for (int k : {idx(i, j), idx(i, j + 1), idx(i + 1, j), idx(i + 1, j), idx(i, j + 1), idx(i + 1, j + 1)})
                verts.push_back(grid[k]);
    return verts;
}

} // namespace Terrain
