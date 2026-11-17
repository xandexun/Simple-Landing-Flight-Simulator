// renderer.cpp
#include "render/renderer.h"
#include "sim/camera.h"
#include "sim/aircraft.h"
#include "sim/environment.h"
#include "world/terrain.h"
#include "world/world_mesh.h"
#include "engine/frustum.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <iostream>
#include <map>

bool Renderer::init(const std::string& root, const World& world) {
    bool ok = phong_.load(root + "shaders/phong.vert", root + "shaders/phong.frag");
    ok = sky_.load(root + "shaders/sky.vert", root + "shaders/sky.frag") && ok;
    ok = glow_.load(root + "shaders/glow.vert", root + "shaders/glow.frag") && ok;
    if (!ok) { std::cerr << "Failed to load shaders from " << root << "shaders/\n"; return false; }

    // runway: chunks of 500 model units (= 250 m) along the runway, culled against the view
    if (!loadOBJ(root + "assets/runway.obj", runway_, 500.0f)) return false;
    // airplane.obj is one object; the two propeller blades are separated by position (they are
    // in front of the engine, around the spinner) so they can be drawn rotating
    auto isBlade = [](const glm::vec3& c) {
        return c.z > 21.3f && (std::fabs(c.x) > 1.0f || std::fabs(c.y - 15.25f) > 1.6f);
    };
    if (!loadOBJ(root + "assets/airplane.obj", airplane_, 0.0f, [&](const glm::vec3& c) { return !isBlade(c); })) return false;
    if (!loadOBJ(root + "assets/airplane.obj", propeller_, 0.0f, isBlade)) return false;

    // The runway surfaces are coplanar (y = 0): they are drawn in layers with an increasing
    // polygon offset, so the upper layers always win the depth test (no z-fighting).
    const std::map<std::string, int> layerOf = {
        {"Material.001", 0},   // white edge stripes (full rectangle)
        {"Material.002", 1},   // grey overrun / blast pad
        {"Material", 2},       // asphalt
        {"white_strip", 3},    // markings, arrows, centre line
        {"material_0", 4}};    // runway numbers
    for (const SubMesh& p : runway_.parts) {
        auto it = layerOf.find(runway_.materials[p.material].name);
        runwayLayer_.push_back(it == layerOf.end() ? 3 : it->second);
    }

    // procedural terrain: 36 km x 36 km, 100 m cells, 2 km culling tiles
    terrain_.materials.push_back({"terrain", glm::vec3(1.0f), glm::vec3(1.0f), glm::vec3(0.0f), 1.0f});
    terrain_.upload(Terrain::buildMesh(0.0f, -3000.0f, 36000.0f, 360), 2000.0f);
    // all buildings baked into one mesh (world space), culled in chunks of 300 m
    structures_.materials.push_back({"building", glm::vec3(0.8f), glm::vec3(1.0f), glm::vec3(0.12f), 24.0f});
    structures_.upload(WorldMesh::structures(world), 300.0f);
    // flat ground patches: one part per layer, drawn with an increasing polygon offset
    std::vector<std::vector<Vertex>> layers = WorldMesh::ground(world);
    ground_.materials.assign(layers.size(), {"ground", glm::vec3(1.0f), glm::vec3(1.0f), glm::vec3(0.02f), 8.0f});
    ground_.uploadGroups(layers, 600.0f);

    glGenVertexArrays(1, &emptyVAO_);
    glGenVertexArrays(1, &glowVAO_);
    glGenBuffers(1, &glowVBO_);
    glBindVertexArray(glowVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, glowVBO_);
    vertexAttribs({3, 3, 1, 1});    // position, colour, size, alpha
    glBindVertexArray(0);
    return true;
}

void Renderer::setModel(const glm::mat4& model) {
    phong_.set("uModel", model);
    phong_.set("uNormalMatrix", glm::transpose(glm::inverse(glm::mat3(model))));
}

void Renderer::beginObject(const glm::mat4& model, int pattern, bool vertexColor) {
    phong_.set("uUseVertexColor", vertexColor);
    phong_.set("uPattern", pattern);
    phong_.set("uTint", glm::vec3(1.0f));
    phong_.set("uAlpha", 1.0f);
    setModel(model);
}

void Renderer::render(const Camera& cam, const Environment& env, const Aircraft& ac, const World& world,
                      const Explosion& explosion, int width, int height, float time) {
    glViewport(0, 0, width, height);
    glClearColor(env.skyHorizon.r, env.skyHorizon.g, env.skyHorizon.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const float aspect = (height > 0) ? float(width) / float(height) : 1.0f;
    view_ = cam.view();
    proj_ = cam.projection(aspect);
    viewportH_ = float(height);
    fovScale_ = 1.0f / std::tan(glm::radians(cam.currentFov()) * 0.5f);
    const glm::mat4 I(1.0f);

    // ---- sky (full screen, no depth) ----
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    sky_.use();
    sky_.set("uInvViewProj", glm::inverse(proj_ * view_));
    sky_.set("uViewPos", cam.eye);
    sky_.set("uSkyZenith", env.skyZenith);
    sky_.set("uSkyHorizon", env.skyHorizon);
    sky_.set("uSunDir", env.sunDir);
    sky_.set("uSunColor", env.sunColor);
    sky_.set("uNight", env.night);
    glBindVertexArray(emptyVAO_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LEQUAL);

    const Frustum frustum(proj_ * view_);
    phong_.use();
    phong_.set("uView", view_);
    phong_.set("uProj", proj_);
    phong_.set("uLightPos", env.lightPos);
    phong_.set("uLightColor", env.lightColor);
    phong_.set("uAmbient", env.ambient);
    phong_.set("uViewPos", cam.eye);
    phong_.set("uSkyHorizon", env.skyHorizon);
    phong_.set("uSunDir", env.sunDir);
    phong_.set("uSunColor", env.sunColor);
    phong_.set("uFogDensity", env.fogDensity);
    phong_.set("uNight", env.night);

    // ---- terrain (pushed slightly back so the runway is always drawn on top) ----
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.0f, 4.0f);
    beginObject(I, 1, true);
    terrain_.draw(phong_, &frustum);

    // ---- runway, layer by layer ----
    const glm::mat4 runwayModel = glm::scale(I, glm::vec3(Airport::RUNWAY_SCALE));
    beginObject(runwayModel);
    const Frustum runwayFrustum(proj_ * view_ * runwayModel);   // frustum in the runway model space
    for (int layer = 0; layer <= 4; ++layer) {
        glPolygonOffset(-1.0f * layer, -2.0f * layer);
        for (size_t i = 0; i < runway_.parts.size(); ++i)
            if (runwayLayer_[i] == layer) runway_.drawPart(phong_, i, &runwayFrustum);
    }

    // ---- taxiways, apron, streets, lots and road markings (flat patches, layered like the runway) ----
    beginObject(I, 1, true);
    for (size_t l = 0; l < ground_.parts.size(); ++l) {
        glPolygonOffset(-1.0f * (l + 1), -2.0f * (l + 1));
        ground_.drawPart(phong_, l, &frustum);
    }
    glPolygonOffset(0.0f, 0.0f);
    glDisable(GL_POLYGON_OFFSET_FILL);

    // ---- airport buildings and city (one baked mesh, facade windows in the shader) ----
    beginObject(I, 3, true);
    structures_.draw(phong_, &frustum);

    // ---- aircraft (hidden in the cockpit view: the camera is inside it) ----
    beginObject(I);
    for (const ParkedPlane& p : world.parkedPlanes) {
        glm::mat4 M = glm::rotate(glm::translate(I, p.pos), p.yaw, glm::vec3(0.0f, 1.0f, 0.0f));
        setModel(glm::translate(glm::scale(M, glm::vec3(Aircraft::MODEL_SCALE)), -Aircraft::modelPivot()));
        airplane_.draw(phong_);
        propeller_.draw(phong_);
    }
    if (cam.mode != CameraMode::Cockpit) drawAircraft(ac, ac.phase == Phase::Crashed);

    // ---- lights and explosion (point sprites) ----
    drawLights(cam, env, ac, world, time);
    drawParticles(explosion);
    glBindVertexArray(0);
}

// The aircraft with its propeller. The propeller turns at the engine rpm: when it turns slowly
// the blades are drawn sharp; when it turns fast (more than ~20 degrees per frame) several
// transparent copies spread over the angle it sweeps in one frame give the blurred disc that a
// camera (or the eye) sees.
void Renderer::drawAircraft(const Aircraft& ac, bool crashed) {
    const glm::mat4 M = ac.modelMatrix();
    beginObject(M);
    if (crashed) phong_.set("uTint", glm::vec3(0.10f, 0.09f, 0.08f));   // burnt wreck
    airplane_.draw(phong_);

    const glm::vec3 hub(0.0f, 15.25f, 0.0f);   // propeller axis in model units (parallel to +Z)
    auto drawBlades = [&](float angle) {
        setModel(glm::translate(glm::rotate(glm::translate(M, hub), angle, glm::vec3(0.0f, 0.0f, 1.0f)), -hub));
        propeller_.draw(phong_);
    };
    const float PI = 3.14159265f;
    float sweep = std::fmin(ac.propRpm / 60.0f * 2.0f * PI / 60.0f, PI);   // angle per frame at 60 fps
    if (crashed || sweep < 0.35f) {
        drawBlades(ac.propAngle);
        return;
    }
    const int copies = 24;
    float blur = glm::clamp((sweep - 0.35f) / (PI - 0.35f), 0.0f, 1.0f);
    phong_.set("uAlpha", glm::mix(0.35f, 0.035f, blur));
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (int i = 0; i < copies; ++i) drawBlades(ac.propAngle - sweep * i / copies);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    phong_.set("uAlpha", 1.0f);
}

// data: 8 floats per point (position, colour, size, alpha); mode as in glow.frag
void Renderer::drawPoints(const std::vector<float>& data, int mode) {
    if (data.empty()) return;
    glBindVertexArray(glowVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, glowVBO_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(data.size() * sizeof(float)), data.data(), GL_STREAM_DRAW);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    if (mode == 2) glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    else glBlendFunc(GL_ONE, GL_ONE);
    glDepthMask(GL_FALSE);
    glow_.use();
    glow_.set("uView", view_);
    glow_.set("uProj", proj_);
    glow_.set("uViewportHeight", viewportH_);
    glow_.set("uFovScale", fovScale_);
    glow_.set("uMode", mode);
    glow_.set("uSizeScale", mode == 0 ? 2.5f : 1.0f);
    glow_.set("uMaxSize", mode == 0 ? 64.0f : 1024.0f);
    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(data.size() / 8));
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void Renderer::drawParticles(const Explosion& explosion) {
    if (!explosion.active) return;
    std::vector<float> smoke, fire;
    for (const Particle& p : explosion.particles) {
        std::vector<float>& d = (p.kind == Particle::Smoke) ? smoke : fire;
        d.insert(d.end(), {p.pos.x, p.pos.y, p.pos.z, p.color.r, p.color.g, p.color.b, p.size(), p.alpha()});
    }
    drawPoints(smoke, 2);   // smoke first: the flames are added on top of it
    drawPoints(fire, 1);
}

void Renderer::drawLights(const Camera& cam, const Environment& env, const Aircraft& ac, const World& world, float time) {
    std::vector<LightPoint> lights = world.staticLights;
    world.dynamicLights(time, ac.pos, lights);

    // aircraft navigation lights: red on the left wing tip, green on the right, white strobe
    if (cam.mode != CameraMode::Cockpit && ac.phase != Phase::Crashed) {
        glm::mat4 M = ac.modelMatrix();
        auto at = [&M](float x, float y, float z) { return glm::vec3(M * glm::vec4(x, y, z, 1.0f)); };
        lights.push_back({at(65.5f, 23.0f, -6.0f), glm::vec3(1.0f, 0.1f, 0.05f), 0.6f});
        lights.push_back({at(-65.5f, 23.0f, -6.0f), glm::vec3(0.1f, 1.0f, 0.2f), 0.6f});
        if (std::fmod(time, 1.2f) < 0.08f)
            lights.push_back({at(0.0f, 33.5f, -70.0f), glm::vec3(1.0f), 1.5f});
    }

    std::vector<float> data;
    data.reserve(lights.size() * 8);
    for (const LightPoint& l : lights) {
        float k = l.nightOnly ? env.night : 1.0f;           // street lamps / obstruction lights: night only
        if (k <= 0.01f) continue;
        data.insert(data.end(), {l.pos.x, l.pos.y, l.pos.z, l.color.r * k, l.color.g * k, l.color.b * k, l.size, 1.0f});
    }
    glow_.use();
    glow_.set("uIntensity", glm::mix(0.55f, 1.4f, env.night));
    drawPoints(data, 0);
}
