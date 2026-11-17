// renderer.h - draws the 3D scene: sky, terrain, runway, airport, aircraft and lights
#pragma once
#include <string>
#include <vector>
#include "engine/gl_utils.h"
#include "engine/mesh.h"
#include "world/airport.h"
#include "sim/explosion.h"

class Camera;
class Aircraft;
class Environment;

class Renderer {
public:
    bool init(const std::string& rootDir, const World& world);
    void render(const Camera& cam, const Environment& env, const Aircraft& ac, const World& world,
                const Explosion& explosion, int width, int height, float time);

private:
    Shader phong_, sky_, glow_;
    Mesh runway_, airplane_, terrain_;
    Mesh propeller_;                        // the two blades of airplane.obj, drawn rotating
    Mesh structures_;                       // every building of the airport and the city, baked
    Mesh ground_;                           // taxiways, apron, streets, lots, road markings (3 layers)
    GLuint emptyVAO_ = 0;                   // used to draw the full-screen sky triangle
    GLuint glowVAO_ = 0, glowVBO_ = 0;
    std::vector<int> runwayLayer_;          // draw order / polygon offset of each runway part
    // per-frame values used by the point sprite passes
    glm::mat4 view_{1.0f}, proj_{1.0f};
    float viewportH_ = 1.0f, fovScale_ = 1.0f;

    // resets the per-object uniforms of the Phong shader and sets the model matrix
    void beginObject(const glm::mat4& model, int pattern = 0, bool vertexColor = false);
    void setModel(const glm::mat4& model);
    void drawAircraft(const Aircraft& ac, bool crashed);
    void drawPoints(const std::vector<float>& data, int mode);
    void drawParticles(const Explosion& explosion);
    void drawLights(const Camera& cam, const Environment& env, const Aircraft& ac, const World& world, float time);
};
