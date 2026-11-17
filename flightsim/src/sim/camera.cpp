// camera.cpp
#include "sim/camera.h"
#include "sim/aircraft.h"
#include "world/airport.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

void Camera::reset() {
    orbitYaw = 0.0f;
    orbitPitch = glm::radians(12.0f);
    distance = 24.0f;
    centerHead();
    // free camera: next to the runway threshold, looking back at the approaching aircraft
    freePos = glm::vec3(-90.0f, 18.0f, Airport::THRESHOLD_Z + 120.0f);
    freeYaw = glm::radians(165.0f);
    freePitch = glm::radians(6.0f);
    first_ = true;
}

static glm::vec3 dirFromAngles(float yaw, float pitch) {
    return glm::vec3(std::cos(pitch) * std::sin(yaw), std::sin(pitch), std::cos(pitch) * std::cos(yaw));
}

void Camera::update(float dt, const Aircraft& ac) {
    const glm::mat4 R = ac.rotation();
    const glm::vec3 center = ac.pos + glm::vec3(R * glm::vec4(0.0f, 1.3f, 0.0f, 0.0f));   // middle of the aircraft
    upVec = glm::vec3(0.0f, 1.0f, 0.0f);
    switch (mode) {
    case CameraMode::Follow: {
        float yaw = ac.heading + 3.14159265f + orbitYaw;        // behind the aircraft
        glm::vec3 wanted = center + distance * dirFromAngles(yaw, orbitPitch);
        wanted.y = std::fmax(wanted.y, Airport::groundHeight(wanted.x, wanted.z) + 0.8f);
        // smooth movement (exponential), but no lag on the first frame
        if (first_) smoothEye_ = wanted;
        eye = glm::mix(smoothEye_, wanted, 1.0f - std::exp(-8.0f * dt));
        target = center;
        break;
    }
    case CameraMode::Cockpit:   // head direction in body axes (+X is the left of the aircraft)
        eye = ac.pos + glm::vec3(R * glm::vec4(Aircraft::cockpitEye(), 0.0f));
        target = eye + glm::vec3(R * glm::vec4(dirFromAngles(headYaw, headPitch), 0.0f));
        upVec = glm::vec3(R * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f));
        break;
    case CameraMode::Tower: {
        // the controller in the tower follows the aircraft with binoculars: the field of view
        // shrinks with the distance so the aircraft always looks about the same size
        eye = glm::vec3(468.0f, 36.0f, -360.0f);
        target = center;
        float dist = glm::length(target - eye);
        towerFov = glm::clamp(glm::degrees(2.0f * std::atan(28.0f / (towerZoom * std::fmax(dist, 1.0f)))), 1.5f, 70.0f);
        break;
    }
    case CameraMode::Free:
        eye = freePos;
        target = freePos + dirFromAngles(freeYaw, freePitch);
        break;
    }
    smoothEye_ = eye;
    first_ = false;
}

void Camera::mouseDrag(float dx, float dy) {
    const float s = 0.005f, lim = glm::radians(85.0f);
    switch (mode) {
    case CameraMode::Follow:
        orbitYaw -= dx * s;
        orbitPitch = glm::clamp(orbitPitch + dy * s, glm::radians(-10.0f), lim);
        break;
    case CameraMode::Cockpit:
        headYaw = glm::clamp(headYaw - dx * s, glm::radians(-150.0f), glm::radians(150.0f));
        headPitch = glm::clamp(headPitch - dy * s, glm::radians(-60.0f), glm::radians(70.0f));
        break;
    case CameraMode::Free:
        freeYaw -= dx * s;
        freePitch = glm::clamp(freePitch - dy * s, -lim, lim);
        break;
    default:
        break;
    }
}

void Camera::scroll(float dy) {
    switch (mode) {
    case CameraMode::Follow:  distance = glm::clamp(distance * std::pow(0.9f, dy), 8.0f, 400.0f); break;
    case CameraMode::Cockpit: fovDeg = glm::clamp(fovDeg - dy * 3.0f, 20.0f, 90.0f); break;
    case CameraMode::Tower:   towerZoom = glm::clamp(towerZoom * std::pow(1.2f, dy), 0.1f, 10.0f); break;
    case CameraMode::Free:    break;
    }
}

glm::mat4 Camera::view() const { return glm::lookAt(eye, target, upVec); }

glm::mat4 Camera::projection(float aspect) const {
    float nearPlane = mode == CameraMode::Cockpit ? 0.3f : mode == CameraMode::Tower ? 2.0f : 0.5f;
    return glm::perspective(glm::radians(currentFov()), aspect, nearPlane, 40000.0f);
}

const char* Camera::name() const {
    switch (mode) {
        case CameraMode::Follow:  return "seguir o avião";
        case CameraMode::Cockpit: return "cockpit (vista do piloto)";
        case CameraMode::Free:    return "livre";
        case CameraMode::Tower:   return "torre de controlo";
    }
    return "";
}
