// camera.h - four cameras: follow (3rd person), cockpit (1st person), free and control tower.
// The mouse rotates (drag with the left button) and the scroll wheel zooms.
#pragma once
#include <glm/glm.hpp>

class Aircraft;

enum class CameraMode { Follow, Cockpit, Free, Tower };

class Camera {
public:
    CameraMode mode = CameraMode::Follow;
    float fovDeg = 55.0f;

    // follow camera (orbit around the aircraft)
    float orbitYaw = 0.0f;                     // relative to the aircraft heading (rad)
    float orbitPitch = glm::radians(12.0f);
    float distance = 24.0f;
    // cockpit camera (pilot head)
    float headYaw = 0.0f, headPitch = glm::radians(-4.0f);
    // free camera
    glm::vec3 freePos{0.0f};
    float freeYaw = 0.0f, freePitch = 0.0f;    // yaw 0 = looking towards +Z
    // control tower camera: fixed at the tower cab, always turned to the aircraft, automatic zoom
    float towerZoom = 1.0f;                    // > 1 = closer (mouse wheel)
    float towerFov = 40.0f;                    // degrees, computed in update()

    // result of update()
    glm::vec3 eye{0.0f}, target{0.0f, 0.0f, 1.0f}, upVec{0.0f, 1.0f, 0.0f};

    void reset();
    void update(float dt, const Aircraft& ac);
    void mouseDrag(float dx, float dy);         // pixels
    void scroll(float dy);
    glm::mat4 view() const;
    glm::mat4 projection(float aspect) const;
    float currentFov() const { return mode == CameraMode::Tower ? towerFov : fovDeg; }   // degrees
    const char* name() const;
    void centerHead() { headYaw = 0.0f; headPitch = glm::radians(-4.0f); }

private:
    glm::vec3 smoothEye_{0.0f};
    bool first_ = true;
};
