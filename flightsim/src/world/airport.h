// airport.h - airport layout: runway geometry constants, buildings and lights.
// No OpenGL here: this is pure data used by the physics and by the renderer.
#pragma once
#include <vector>
#include <glm/glm.hpp>

namespace Airport {
    // runway.obj is 120 m x 7320 m; it is drawn scaled by 0.5 -> 60 m x 3660 m (a real size)
    constexpr float RUNWAY_SCALE      = 0.5f;
    constexpr float RUNWAY_HALF_WIDTH = 30.0f;    // including the white edge stripes
    constexpr float RUNWAY_START_Z    = -1830.0f; // start of the paved surface (approach side)
    constexpr float RUNWAY_END_Z      =  1830.0f;
    constexpr float THRESHOLD_Z       = -1271.5f; // landing threshold of runway 27 (after the arrows)
    constexpr float AIM_POINT_Z       = THRESHOLD_Z + 300.0f; // touchdown aiming point
    constexpr float GLIDE_SLOPE_DEG   = 3.0f;
    constexpr float LOCALIZER_Z       = RUNWAY_END_Z;          // localizer antenna (far end)
    // approach light bars every 30 m before the threshold, only on the paved part of the runway
    // (the area with the arrows): 18 bars -> from the threshold back to 540 m (start of the pavement 558 m)
    constexpr int   APPROACH_BARS     = 18;
    constexpr float RUNWAY_HEADING    = 270.0f;   // flying towards +Z = heading 270 (runway 27)

    bool  onRunway(float x, float z);
    bool  isPaved(float x, float z);              // runway, taxiways or apron
    float groundHeight(float x, float z);         // height of what the wheels touch
}

// The city around the airport: several districts, each with its own street grid rotated by a
// different angle (so the city is not one big regular grid), joined by curved motorways.
// Districts to the left of the landing aircraft (+X) and ahead of it (beyond the runway end, +Z).
namespace City {
    struct District {
        glm::vec2 center;      // metres (x, z)
        float radius;          // metres (the border is irregular around this circle)
        float angleDeg;        // rotation of the street grid
        float pitch;           // block + street width
        float hCenter, hEdge;  // building height near the centre / near the border
        unsigned seed;
    };
    const std::vector<District>& districts();
    // Motorways: control points of curved roads (Catmull-Rom splines), metres (x, z)
    const std::vector<std::vector<glm::vec2>>& roadControls();
    constexpr float ROAD_HALF = 11.0f;          // 2 x 2 lanes + central barrier
    // Normalised distance of a point to the district border (< 1 = inside), with an irregular edge
    float districtScore(const District& d, glm::vec2 p);
    // Distance from a point to the motorway control polylines (metres)
    float distanceToRoads(glm::vec2 p);
}

// A road sampled as a polyline (for drawing and lamps)
struct RoadPath {
    std::vector<glm::vec2> pts;
};

// A box (optionally with a gable roof). The renderer bakes all of them into one static mesh.
struct Building {
    glm::vec3 pos{0.0f};      // centre of the base (metres)
    glm::vec3 size{1.0f};     // width (x), height (y), depth (z) in metres
    float yaw = 0.0f;         // rotation around Y (radians)
    glm::vec3 color{0.7f};
    glm::vec3 roofColor{0.5f, 0.2f, 0.15f};
    float roofHeight = 0.0f;  // > 0 -> gable roof on top
    int windowMode = 0;       // 0 = none, 1 = punched windows, 2 = glass curtain wall, 3 = house windows
    float seed = 0.0f;        // random value (0..1) used by the window lights
};

// Flat quad on the ground (taxiway, apron, street, lot, road segment...), any orientation:
// corners c[0..3] in order around the quad (x, z). Layers are drawn on top of each other with
// polygon offset: 0 = asphalt/concrete, 1 = lots and sidewalks, 2 = road markings.
struct GroundPatch {
    glm::vec2 c[4];
    glm::vec3 color;
    int layer;
};

struct LightPoint {
    glm::vec3 pos;
    glm::vec3 color;
    float size;               // apparent size in metres
    bool nightOnly = false;   // street lamps and obstruction lights are only lit at night
};

struct ParkedPlane {
    glm::vec3 pos;
    float yaw;
};

class World {
public:
    std::vector<Building> buildings;
    std::vector<GroundPatch> ground;
    std::vector<LightPoint> staticLights;   // runway edge, threshold, end, approach, taxiway
    std::vector<ParkedPlane> parkedPlanes;
    std::vector<RoadPath> roads;

    void build();
    // Lights that change with time or with the aircraft position:
    // PAPI (colour depends on the angle at which the aircraft sees it) and the
    // sequenced flashing approach lights ("rabbit").
    void dynamicLights(float time, const glm::vec3& aircraftPos, std::vector<LightPoint>& out) const;
    // Number of white PAPI lights seen from a position (2 = on the glide slope)
    static int papiWhiteCount(const glm::vec3& eye);
    // Does a sphere (centre p, radius r) touch a building? (used to crash into the city)
    bool hitsBuilding(const glm::vec3& p, float r) const;
};
