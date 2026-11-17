// environment.h - the sun (a movable point light) and everything derived from it:
// light colour, ambient light, sky colours, fog and the day/night factor.
#pragma once
#include <glm/glm.hpp>

class Environment {
public:
    // Point light position in world space (metres). It represents the sun, so it is far
    // away, but it is a true point light: the light direction changes across the scene.
    glm::vec3 lightPos{-2500.0f, 3200.0f, -5200.0f};

    // Values derived in update()
    glm::vec3 sunDir{0.0f, 1.0f, 0.0f};   // direction from the airport to the light
    glm::vec3 lightColor{1.0f};
    glm::vec3 ambient{0.3f};
    glm::vec3 skyZenith{0.2f, 0.4f, 0.8f};
    glm::vec3 skyHorizon{0.7f, 0.8f, 0.9f};
    glm::vec3 sunColor{1.0f};             // colour of the sun disc / glow in the sky
    float night = 0.0f;                   // 0 = full day, 1 = full night
    float fogDensity = 1.0f / 14000.0f;

    void reset();
    // The sun moves on a circle around the scene (a vertical plane): angle 0 = on the horizon,
    // 90 deg = overhead, 180 deg = on the opposite horizon, > 180 deg = below the ground (night).
    void rotateSun(float dAngle);         // radians; positive = towards and past the top

private:
    float sunAngle_ = 0.0f;               // radians
    glm::vec2 sunAzimuth_{0.0f, -1.0f};   // horizontal direction (x, z) where the sun rises
    float sunDistance_ = 6600.0f;         // metres
    void update();
};
