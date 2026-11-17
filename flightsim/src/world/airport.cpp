// airport.cpp
#include "world/airport.h"
#include "world/terrain.h"
#include <algorithm>
#include <cmath>

namespace City {

const std::vector<District>& districts() {
    //                         centre (x, z)          radius angle pitch  hCentre hEdge seed
    static const std::vector<District> d = {
        // ---- left of the landing aircraft (+X), from the airport fence outwards
        {glm::vec2(2000.0f,   450.0f), 820.0f,  10.0f,  95.0f, 330.0f, 70.0f, 11u},   // downtown
        {glm::vec2(1850.0f,  -950.0f), 700.0f,  32.0f, 105.0f, 110.0f, 30.0f, 23u},
        {glm::vec2(2950.0f,  -250.0f), 750.0f, -15.0f, 110.0f, 140.0f, 30.0f, 37u},
        {glm::vec2(1900.0f,  1700.0f), 720.0f, -28.0f, 100.0f, 190.0f, 35.0f, 41u},
        {glm::vec2(3050.0f,  1150.0f), 780.0f,  22.0f, 115.0f, 150.0f, 30.0f, 43u},
        {glm::vec2(2850.0f,  2550.0f), 720.0f, -40.0f, 105.0f, 130.0f, 30.0f, 47u},
        // ---- ahead of the landing aircraft (+Z), beyond the runway end
        {glm::vec2( 850.0f,  3450.0f), 820.0f,   8.0f,  95.0f, 270.0f, 50.0f, 53u},   // second centre
        {glm::vec2(-950.0f,  3300.0f), 780.0f, -18.0f, 110.0f, 130.0f, 30.0f, 67u},
        {glm::vec2( 100.0f,  4750.0f), 780.0f,  25.0f, 105.0f, 160.0f, 30.0f, 79u},
        {glm::vec2(-2000.0f, 4300.0f), 700.0f,  35.0f, 110.0f, 100.0f, 25.0f, 83u},
        {glm::vec2(1900.0f,  3700.0f), 650.0f, -12.0f, 100.0f, 140.0f, 30.0f, 89u},
        {glm::vec2(1750.0f,  2650.0f), 620.0f,  15.0f, 100.0f, 120.0f, 30.0f, 97u},   // joins left and ahead
    };
    return d;
}

const std::vector<std::vector<glm::vec2>>& roadControls() {
    static const std::vector<std::vector<glm::vec2>> r = {
        // terminal -> curves east -> through downtown -> north-east
        {{545.0f, -160.0f}, {548.0f, 250.0f}, {800.0f, 560.0f}, {1300.0f, 620.0f}, {2000.0f, 450.0f},
         {2700.0f, 720.0f}, {3300.0f, 1150.0f}, {3700.0f, 1800.0f}},
        // branch to the districts ahead of the runway (crosses the extended centre line)
        {{800.0f, 560.0f}, {1350.0f, 1250.0f}, {1450.0f, 2050.0f}, {1300.0f, 2750.0f}, {900.0f, 3150.0f},
         {300.0f, 3500.0f}, {-500.0f, 3700.0f}, {-1300.0f, 3550.0f}, {-2100.0f, 3900.0f}},
        // ring to the southern districts
        {{1300.0f, 620.0f}, {1500.0f, -300.0f}, {2000.0f, -1100.0f}, {2700.0f, -1500.0f}},
        // north-south road through the districts ahead
        {{300.0f, 3500.0f}, {150.0f, 4300.0f}, {350.0f, 5300.0f}},
    };
    return r;
}

float districtScore(const District& d, glm::vec2 p) {
    glm::vec2 q = (p - d.center) / d.radius;
    float a = std::atan2(q.y, q.x) + d.seed;
    float edge = 1.0f + 0.12f * std::sin(3.0f * a) + 0.08f * std::sin(5.0f * a + 2.1f);
    return glm::length(q) / edge;
}

float distanceToRoads(glm::vec2 p) {
    float best = 1e9f;
    for (const std::vector<glm::vec2>& c : roadControls())
        for (size_t i = 0; i + 1 < c.size(); ++i) {
            glm::vec2 a = c[i], ab = c[i + 1] - a;
            float t = glm::clamp(glm::dot(p - a, ab) / glm::dot(ab, ab), 0.0f, 1.0f);
            best = std::min(best, glm::length(p - (a + ab * t)));
        }
    return best;
}

} // namespace City

namespace Airport {

bool onRunway(float x, float z) {
    return std::fabs(x) <= RUNWAY_HALF_WIDTH && z >= RUNWAY_START_Z && z <= RUNWAY_END_Z;
}

bool isPaved(float x, float z) {
    if (onRunway(x, z)) return true;
    if (x >= 118.0f && x <= 142.0f && z >= -1250.0f && z <= 1660.0f) return true;  // parallel taxiway
    if (x >= 150.0f && x <= 450.0f && z >= -300.0f && z <= 400.0f) return true;    // apron
    if (x >= 538.0f && x <= 780.0f && z >= -160.0f && z <= 250.0f) return true;    // access road + car park
    const float connectors[] = {-1100.0f, -400.0f, 300.0f, 1000.0f, 1600.0f};
    for (float cz : connectors)
        if (x >= 30.0f && x <= 130.0f && std::fabs(z - cz) <= 11.5f) return true;
    return false;
}

float groundHeight(float x, float z) {
    if (isPaved(x, z)) return 0.0f;
    return Terrain::height(x, z);
}

} // namespace Airport

namespace {
// small deterministic random generator (same scene every run)
struct Rng {
    unsigned int s;
    explicit Rng(unsigned int seed) : s(seed * 2654435761u + 12345u) { next(); next(); }
    float next() { s = s * 1664525u + 1013904223u; return (s >> 8) / float(1 << 24); }
    float range(float a, float b) { return a + (b - a) * next(); }
    int pick(int n) { return std::min(n - 1, static_cast<int>(next() * n)); }
};
} // namespace

void World::build() {
    using namespace Airport;
    buildings.clear();
    ground.clear();
    staticLights.clear();
    parkedPlanes.clear();
    roads.clear();

    const glm::vec3 asphalt(0.16f, 0.16f, 0.17f), concrete(0.42f, 0.42f, 0.40f);
    const glm::vec3 white(1.0f, 0.95f, 0.85f), yellow(1.0f, 0.8f, 0.25f), green(0.2f, 1.0f, 0.35f),
                    red(1.0f, 0.12f, 0.08f), blue(0.25f, 0.4f, 1.0f), lamp(1.0f, 0.78f, 0.45f);

    auto patch = [&](float x0, float x1, float z0, float z1, glm::vec3 col, int layer) {
        ground.push_back({{{x0, z0}, {x0, z1}, {x1, z1}, {x1, z0}}, col, layer});
    };
    auto quad = [&](glm::vec2 a, glm::vec2 b, glm::vec2 c, glm::vec2 d, glm::vec3 col, int layer) {
        ground.push_back({{a, b, c, d}, col, layer});
    };
    auto box = [&](float x, float y, float z, float w, float h, float d, glm::vec3 col, int windows = 0,
                   float seed = 0.0f, float yaw = 0.0f, float roofH = 0.0f, glm::vec3 roofCol = glm::vec3(0.5f, 0.2f, 0.15f)) {
        buildings.push_back({glm::vec3(x, y, z), glm::vec3(w, h, d), yaw, col, roofCol, roofH, windows, seed});
    };

    // ================================================================= airport
    patch(118.0f, 142.0f, -1250.0f, 1660.0f, asphalt, 0);                        // parallel taxiway
    for (float cz : {-1100.0f, -400.0f, 300.0f, 1000.0f, 1600.0f})
        patch(30.0f, 118.0f, cz - 11.5f, cz + 11.5f, asphalt, 0);                // connectors
    patch(150.0f, 450.0f, -300.0f, 400.0f, concrete, 0);                         // apron

    box(500.0f, 0.0f, 50.0f, 50.0f, 14.0f, 380.0f, glm::vec3(0.62f, 0.72f, 0.80f), 2, 0.37f); // terminal
    box(480.0f, 0.0f, -360.0f, 8.0f, 32.0f, 8.0f, glm::vec3(0.85f, 0.85f, 0.82f));          // tower shaft
    box(480.0f, 32.0f, -360.0f, 14.0f, 6.0f, 14.0f, glm::vec3(0.30f, 0.50f, 0.55f), 2, 0.9f); // tower cab
    box(480.0f, 38.0f, -360.0f, 16.0f, 1.2f, 16.0f, glm::vec3(0.3f, 0.3f, 0.32f));            // tower roof
    for (int i = 0; i < 3; ++i)                                                                                     // hangars
        box(220.0f + i * 80.0f, 0.0f, 480.0f, 60.0f, 12.0f, 70.0f,
            glm::vec3(0.55f, 0.58f, 0.62f), 0, 0.0f, 0.0f, 6.0f, glm::vec3(0.35f, 0.38f, 0.42f));
    for (int i = 0; i < 4; ++i)
        parkedPlanes.push_back({glm::vec3(300.0f, 0.0f, -200.0f + i * 45.0f), glm::radians(-90.0f)});

    // ================================================================= airport landside
    // car park, car park building and hotel (the motorway starts at the terminal)
    patch(560.0f, 780.0f, -150.0f, 250.0f, glm::vec3(0.20f, 0.20f, 0.21f), 0);     // car park
    patch(552.0f, 560.0f, 40.0f, 60.0f, asphalt, 0);                               // car park entrance
    for (float z = -140.0f; z <= 240.0f; z += 20.0f)                               // parking bays
        for (float x = 575.0f; x < 770.0f; x += 2.6f)
            patch(x, x + 0.15f, z - 5.0f, z + 5.0f, glm::vec3(0.85f), 2);
    box(960.0f, 0.0f, 250.0f, 40.0f, 46.0f, 26.0f, glm::vec3(0.46f, 0.55f, 0.66f), 2, 0.61f);  // hotel
    box(960.0f, 46.0f, 250.0f, 16.0f, 3.0f, 10.0f, glm::vec3(0.4f, 0.4f, 0.42f));
    box(830.0f, 0.0f, 40.0f, 70.0f, 15.0f, 120.0f, glm::vec3(0.62f, 0.62f, 0.60f), 1, 0.2f); // car park building

    // ================================================================= the city
    // Districts with rotated street grids (City::districts) around the airport: to the left of the
    // landing aircraft (+X) and ahead of it, beyond the runway end (+Z). Every block belongs to the
    // district whose (irregular) border is nearest, so the grids meet at different angles.
    // Only city buildings: skyscrapers in the centres, towers and apartment blocks around them.
    const glm::vec3 street(0.12f, 0.12f, 0.13f), sidewalk(0.50f, 0.50f, 0.48f), grassLot(0.30f, 0.42f, 0.22f),
                    parkGreen(0.24f, 0.40f, 0.17f), lineCol(0.72f, 0.68f, 0.34f);
    const glm::vec3 glassTints[] = {{0.55f, 0.66f, 0.78f}, {0.42f, 0.60f, 0.62f}, {0.70f, 0.72f, 0.76f},
                                    {0.36f, 0.44f, 0.58f}, {0.62f, 0.56f, 0.46f}, {0.50f, 0.55f, 0.60f},
                                    {0.30f, 0.38f, 0.45f}, {0.58f, 0.70f, 0.70f}};
    const glm::vec3 officeColors[] = {{0.80f, 0.76f, 0.68f}, {0.66f, 0.66f, 0.68f}, {0.74f, 0.50f, 0.40f},
                                      {0.86f, 0.86f, 0.84f}, {0.58f, 0.60f, 0.64f}, {0.70f, 0.62f, 0.52f},
                                      {0.90f, 0.84f, 0.72f}, {0.62f, 0.52f, 0.48f}};
    const glm::vec3 machinery(0.40f, 0.40f, 0.42f);

    // ---------------- motorways: Catmull-Rom splines through the control points
    for (const std::vector<glm::vec2>& cp : City::roadControls()) {
        RoadPath r;
        for (size_t i = 0; i + 1 < cp.size(); ++i) {
            glm::vec2 p0 = cp[i == 0 ? 0 : i - 1], p1 = cp[i], p2 = cp[i + 1], p3 = cp[std::min(i + 2, cp.size() - 1)];
            int n = std::max(2, static_cast<int>(glm::length(p2 - p1) / 20.0f));
            for (int k = 0; k < n; ++k) {
                float t = float(k) / n, t2 = t * t, t3 = t2 * t;
                r.pts.push_back(0.5f * ((2.0f * p1) + (p2 - p0) * t + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                                        (3.0f * p1 - p0 - 3.0f * p2 + p3) * t3));
            }
        }
        r.pts.push_back(cp.back());
        roads.push_back(r);
    }
    for (const RoadPath& r : roads) {
        // normal at every point (average of the neighbouring segments, so the quads join without gaps)
        std::vector<glm::vec2> nrm(r.pts.size());
        for (size_t i = 0; i < r.pts.size(); ++i) {
            glm::vec2 t = glm::normalize(r.pts[std::min(i + 1, r.pts.size() - 1)] - r.pts[i == 0 ? 0 : i - 1]);
            nrm[i] = glm::vec2(-t.y, t.x);
        }
        // a strip between offsets o0 and o1 from the centre line; dash > 0 draws only every other segment
        auto strip = [&](float o0, float o1, glm::vec3 col, int layer, bool dashed) {
            for (size_t i = 0; i + 1 < r.pts.size(); ++i) {
                if (dashed && (i % 2)) continue;
                glm::vec2 a = r.pts[i], b = r.pts[i + 1];
                glm::vec2 na = nrm[i], nb = nrm[i + 1];
                if (dashed) { b = a + (b - a) * 0.4f; nb = na; }
                quad(a + na * o0, a + na * o1, b + nb * o1, b + nb * o0, col, layer);
            }
        };
        const float H = City::ROAD_HALF;
        strip(-H, H, glm::vec3(0.14f, 0.14f, 0.15f), 0, false);            // asphalt
        strip(-1.2f, 1.2f, glm::vec3(0.55f, 0.55f, 0.52f), 1, false);       // central barrier
        for (float o : {-H + 1.0f, H - 1.0f})                               // edge lines
            strip(o - 0.15f, o + 0.15f, glm::vec3(0.88f), 2, false);
        for (float o : {-5.7f, 5.7f})                                       // lane lines (dashed)
            strip(o - 0.12f, o + 0.12f, glm::vec3(0.88f), 2, true);
        for (size_t i = 0; i < r.pts.size(); i += 4)                        // tall lamps on the barrier
            for (glm::vec2 p : {r.pts[i] + nrm[i] * 1.5f, r.pts[i] - nrm[i] * 1.5f})
                staticLights.push_back({glm::vec3(p.x, 12.0f, p.y), lamp, 2.4f, true});
    }

    // ---------------- buildings
    const std::vector<City::District>& D = City::districts();
    for (size_t di = 0; di < D.size(); ++di) {
        const City::District& dist = D[di];
        const float ang = glm::radians(dist.angleDeg), ca = std::cos(ang), sa = std::sin(ang);
        // local grid (u, v) -> world (x, z)
        auto W = [&](float u, float v) { return dist.center + glm::vec2(ca * u - sa * v, sa * u + ca * v); };
        const float P = dist.pitch, B = P - 22.0f;                         // block (streets are 22 m wide)
        const float yaw = -ang;                                             // Building yaw for this grid
        const int n = static_cast<int>(dist.radius * 1.3f / P);

        // a box in local grid coordinates
        auto lbox = [&](float u, float v, float y, float w, float h, float d, glm::vec3 col, int windows = 0, float seed = 0.0f) {
            glm::vec2 p = W(u, v);
            box(p.x, y, p.y, w, h, d, col, windows, seed, yaw);
        };
        auto tower = [&](Rng& rng, float u, float v, float w, float d, float Ht, bool glass, glm::vec3 col, float y0 = 0.0f) {
            int style = glass ? 2 : 1;
            float top;
            if (Ht > 200.0f) {
                float h1 = Ht * 0.55f, h2 = Ht * 0.82f;
                lbox(u, v, y0, w, h1, d, col, style, rng.next());
                lbox(u, v, y0 + h1, w * 0.8f, h2 - h1, d * 0.8f, col, style, rng.next());
                lbox(u, v, y0 + h2, w * 0.6f, Ht - h2, d * 0.6f, col, style, rng.next());
                lbox(u, v, y0 + Ht, w * 0.3f, 6.0f, d * 0.3f, machinery);
                top = y0 + Ht + 6.0f;
            } else if (Ht > 110.0f) {
                float lower = Ht * 0.72f;
                lbox(u, v, y0, w, lower, d, col, style, rng.next());
                lbox(u, v, y0 + lower, w * 0.72f, Ht - lower, d * 0.72f, col, style, rng.next());
                lbox(u, v, y0 + Ht, w * 0.34f, 4.0f, d * 0.34f, machinery);
                top = y0 + Ht + 4.0f;
            } else {
                lbox(u, v, y0, w, Ht, d, col, style, rng.next());
                lbox(u, v, y0 + Ht, w * 0.45f, 3.0f, d * 0.45f, machinery);
                top = y0 + Ht + 3.0f;
            }
            if (Ht > 85.0f) {
                float ant = (Ht > 200.0f) ? 45.0f : 25.0f;
                lbox(u, v, top, 1.4f, ant, 1.4f, glm::vec3(0.55f, 0.55f, 0.58f));
                glm::vec2 p = W(u, v);
                staticLights.push_back({glm::vec3(p.x, top + ant, p.y), red, 3.0f, true});   // obstruction light
            }
        };
        auto tree = [&](Rng& rng, float u, float v) {
            float h = rng.range(5.0f, 9.0f), c = rng.range(4.0f, 7.0f);
            lbox(u, v, 0.0f, 0.7f, h * 0.45f, 0.7f, glm::vec3(0.35f, 0.25f, 0.16f));
            lbox(u, v, h * 0.45f, c, h * 0.55f, c, glm::vec3(0.16f, rng.range(0.30f, 0.42f), 0.12f));
        };
        // ground quad in local coordinates
        auto lquad = [&](float u0, float u1, float v0, float v1, glm::vec3 col, int layer) {
            quad(W(u0, v0), W(u0, v1), W(u1, v1), W(u1, v0), col, layer);
        };

        for (int j = -n; j <= n; ++j)
            for (int i = -n; i <= n; ++i) {
                const float uc = i * P, vc = j * P;                         // block centre (local)
                const glm::vec2 pc = W(uc, vc);
                // the block belongs to the district with the lowest score
                float score = City::districtScore(dist, pc);
                if (score > 1.0f) continue;
                bool mine = true;
                for (size_t k = 0; k < D.size() && mine; ++k)
                    if (k != di && City::districtScore(D[k], pc) < score) mine = false;
                if (!mine) continue;
                // keep the airport, the runway, its extended centre line and the motorways free
                if (pc.x > -700.0f && pc.x < 1150.0f && pc.y > -2300.0f && pc.y < 2500.0f) continue;
                if (std::fabs(pc.x) < 260.0f && pc.y > 1800.0f) continue;
                if (City::distanceToRoads(pc) < City::ROAD_HALF + P * 0.6f) continue;

                Rng rng(dist.seed + 131u * unsigned(i + 100) + 7919u * unsigned(j + 100));
                const float u0 = uc - B * 0.5f, u1 = uc + B * 0.5f, v0 = vc - B * 0.5f, v1 = vc + B * 0.5f;
                lquad(uc - P * 0.5f, uc + P * 0.5f, vc - P * 0.5f, vc + P * 0.5f, street, 0);   // streets around
                lquad(uc - P * 0.5f, uc - P * 0.5f + 0.36f, v0 + 6.0f, v1 - 6.0f, lineCol, 2);   // centre lines
                lquad(u0 + 6.0f, u1 - 6.0f, vc - P * 0.5f, vc - P * 0.5f + 0.36f, lineCol, 2);
                for (glm::vec2 l : {W(uc - P * 0.5f + 8.0f, vc), W(uc, vc - P * 0.5f + 8.0f)})   // street lamps
                    staticLights.push_back({glm::vec3(l.x, 7.0f, l.y), lamp, 1.7f, true});

                if (rng.next() < 0.07f) {                                   // square with trees
                    lquad(u0, u1, v0, v1, parkGreen, 1);
                    for (int t = 0; t < 10; ++t) tree(rng, rng.range(u0 + 5.0f, u1 - 5.0f), rng.range(v0 + 5.0f, v1 - 5.0f));
                    continue;
                }
                lquad(u0, u1, v0, v1, sidewalk, 1);
                const float s = glm::clamp(score, 0.0f, 1.0f);
                const float hmax = glm::mix(dist.hCenter, dist.hEdge, std::sqrt(s));
                const float roll = rng.next();
                if (hmax > 170.0f && roll < 0.6f) {                         // podium + skyscraper
                    float podium = rng.range(12.0f, 20.0f);
                    lbox(uc, vc, 0.0f, B - 8.0f, podium, B - 8.0f, officeColors[rng.pick(8)], 1, rng.next());
                    tower(rng, uc, vc, B * rng.range(0.52f, 0.66f), B * rng.range(0.52f, 0.66f),
                          hmax * rng.range(0.65f, 1.0f), true, glassTints[rng.pick(8)], podium);
                } else if (hmax > 90.0f && roll < 0.75f) {                  // two towers on a diagonal
                    bool flip = rng.next() < 0.5f;
                    for (int t = 0; t < 2; ++t) {
                        float f = (t == 0) ? -0.22f : 0.22f;
                        tower(rng, uc + B * f, vc + B * (flip ? -f : f), B * 0.42f, B * 0.42f,
                              hmax * rng.range(0.45f, 0.95f), rng.next() < 0.75f, glassTints[rng.pick(8)]);
                    }
                } else if (roll < 0.45f) {                                  // apartment block with a courtyard
                    float Hb = std::max(15.0f, hmax * rng.range(0.35f, 0.7f)), t = 14.0f;
                    glm::vec3 col = officeColors[rng.pick(8)];
                    float sd = rng.next();
                    lbox(uc, v0 + t * 0.5f, 0.0f, B - 4.0f, Hb, t, col, 1, sd);
                    lbox(uc, v1 - t * 0.5f, 0.0f, B - 4.0f, Hb, t, col, 1, sd);
                    lbox(u0 + t * 0.5f, vc, 0.0f, t, Hb, B - 2.0f * t - 4.0f, col, 1, sd);
                    lbox(u1 - t * 0.5f, vc, 0.0f, t, Hb, B - 2.0f * t - 4.0f, col, 1, sd);
                    lquad(u0 + t + 2.0f, u1 - t - 2.0f, v0 + t + 2.0f, v1 - t - 2.0f, grassLot, 2);
                    tree(rng, uc, vc);
                } else {                                                    // four buildings of mixed heights
                    for (int a = 0; a < 2; ++a)
                        for (int c = 0; c < 2; ++c) {
                            float H = std::max(14.0f, hmax * rng.range(0.25f, 0.85f));
                            float u = uc + B * (a ? 0.25f : -0.25f), v = vc + B * (c ? 0.25f : -0.25f);
                            glm::vec3 col = rng.next() < 0.3f ? glassTints[rng.pick(8)] : officeColors[rng.pick(8)];
                            if (H > 85.0f) tower(rng, u, v, B * 0.4f, B * 0.4f, H, rng.next() < 0.6f, col);
                            else {
                                lbox(u, v, 0.0f, B * 0.42f, H, B * 0.42f, col, rng.next() < 0.25f ? 2 : 1, rng.next());
                                lbox(u, v, H, B * 0.15f, 2.5f, B * 0.15f, machinery);
                            }
                        }
                }
            }
    }

    // ================================================================= runway lights
    const float edgeX = RUNWAY_HALF_WIDTH + 1.0f;
    for (float z = THRESHOLD_Z; z <= RUNWAY_END_Z - 140.0f; z += 60.0f) {
        for (float x : {edgeX, -edgeX})
            staticLights.push_back({glm::vec3(x, 0.4f, z), (z > RUNWAY_END_Z - 140.0f - 600.0f) ? yellow : white, 1.4f});
    }
    for (int i = 0; i < 14; ++i) {                     // threshold (green) and runway end (red)
        float x = -RUNWAY_HALF_WIDTH + i * (2.0f * RUNWAY_HALF_WIDTH / 13.0f);
        staticLights.push_back({glm::vec3(x, 0.4f, THRESHOLD_Z - 2.0f), green, 1.5f});
        staticLights.push_back({glm::vec3(x, 0.4f, RUNWAY_END_Z - 140.0f), red, 1.5f});
    }
    for (int b = 1; b <= APPROACH_BARS; ++b) {         // approach light bars before the threshold
        float z = THRESHOLD_Z - 30.0f * b;
        for (int i = -2; i <= 2; ++i)
            staticLights.push_back({glm::vec3(i * 2.5f, 0.6f, z), white, 1.3f});
    }
    for (float z = -1250.0f; z <= 1660.0f; z += 50.0f)     // taxiway edge (blue)
        for (float x : {117.0f, 143.0f}) staticLights.push_back({glm::vec3(x, 0.3f, z), blue, 1.0f});
}

bool World::hitsBuilding(const glm::vec3& p, float r) const {
    for (const Building& b : buildings) {
        float top = b.pos.y + b.size.y + b.roofHeight;
        if (p.y - r > top || p.y + r < b.pos.y) continue;
        float dx = p.x - b.pos.x, dz = p.z - b.pos.z;
        float half = 0.5f * std::max(b.size.x, b.size.z) + r;
        if (dx * dx + dz * dz > 2.0f * half * half) continue;        // quick test
        // point in the building's own axes (the building is rotated by yaw around Y)
        float c = std::cos(b.yaw), s = std::sin(b.yaw);
        float lx = c * dx - s * dz, lz = s * dx + c * dz;
        if (std::fabs(lx) <= b.size.x * 0.5f + r && std::fabs(lz) <= b.size.z * 0.5f + r) return true;
    }
    return false;
}

int World::papiWhiteCount(const glm::vec3& eye) {
    using namespace Airport;
    float dist = AIM_POINT_Z - eye.z;
    if (dist <= 1.0f) return -1;                          // already past the PAPI
    float angle = glm::degrees(std::atan2(eye.y - 0.5f, dist));
    const float thresholds[4] = {2.5f, 2.83f, 3.17f, 3.5f};
    int n = 0;
    for (float t : thresholds) if (angle > t) ++n;
    return n;   // 0 = too low (4 red) ... 2 = on slope ... 4 = too high (4 white)
}

void World::dynamicLights(float time, const glm::vec3& aircraftPos, std::vector<LightPoint>& out) const {
    using namespace Airport;
    // PAPI on the left side of the runway, abeam the aiming point.
    // Light i (from the runway outwards) is white when the aircraft is above its angle.
    int whites = papiWhiteCount(aircraftPos);
    for (int i = 0; i < 4; ++i) {   // the light nearest to the runway turns white last (highest angle)
        bool isWhite = (whites >= 0) && (i >= 4 - whites);
        out.push_back({glm::vec3(RUNWAY_HALF_WIDTH + 15.0f + i * 9.0f, 1.0f, AIM_POINT_Z),
                       isWhite ? glm::vec3(1.0f, 0.97f, 0.9f) : glm::vec3(1.0f, 0.1f, 0.06f), 2.2f});
    }
    // Sequenced flashers ("rabbit") running towards the threshold twice per second
    float phase = std::fmod(time * 2.0f, 1.0f);
    int bar = APPROACH_BARS - static_cast<int>(phase * APPROACH_BARS);
    if (bar >= 1 && bar <= APPROACH_BARS)
        out.push_back({glm::vec3(0.0f, 1.0f, THRESHOLD_Z - 30.0f * bar), glm::vec3(1.0f), 4.0f});
}
