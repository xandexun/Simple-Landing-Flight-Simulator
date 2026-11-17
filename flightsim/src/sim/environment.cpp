// environment.cpp
#include "sim/environment.h"
#include <cmath>

void Environment::reset() {
    lightPos = glm::vec3(-2500.0f, 3200.0f, -5200.0f);
    glm::vec2 h(lightPos.x, lightPos.z);
    sunAzimuth_ = glm::normalize(h);
    sunDistance_ = glm::length(lightPos);
    sunAngle_ = std::atan2(lightPos.y, glm::length(h));
    update();
}

void Environment::rotateSun(float dAngle) {
    const float TWO_PI = 6.2831853f;
    sunAngle_ = std::fmod(sunAngle_ + dAngle + TWO_PI, TWO_PI);
    float c = std::cos(sunAngle_), s = std::sin(sunAngle_);
    lightPos = sunDistance_ * glm::vec3(c * sunAzimuth_.x, s, c * sunAzimuth_.y);
    update();
}

void Environment::update() {
    sunDir = glm::normalize(lightPos);
    float elev = sunDir.y;                                     // sin(elevation)
    night = 1.0f - glm::smoothstep(-0.06f, 0.12f, elev);       // dark when the sun is below the horizon
    float low = 1.0f - glm::smoothstep(0.05f, 0.45f, elev);    // 1 at sunrise/sunset, 0 at noon

    // light colour: warm/orange when low, white-yellow when high, off at night
    lightColor = glm::mix(glm::vec3(1.00f, 0.97f, 0.90f), glm::vec3(1.00f, 0.55f, 0.28f), low) * (1.0f - night) * 1.05f;
    sunColor = glm::mix(glm::vec3(1.0f, 0.95f, 0.85f), glm::vec3(1.0f, 0.5f, 0.2f), low);
    // sky colours and ambient light (light coming from the whole sky): day -> dusk -> night
    auto dayDuskNight = [&](glm::vec3 day, glm::vec3 dusk, glm::vec3 night_) {
        return glm::mix(glm::mix(day, dusk, low), night_, night);
    };
    skyZenith  = dayDuskNight({0.18f, 0.38f, 0.78f}, {0.16f, 0.22f, 0.45f}, {0.005f, 0.01f, 0.03f});
    skyHorizon = dayDuskNight({0.70f, 0.80f, 0.92f}, {0.95f, 0.58f, 0.38f}, {0.03f, 0.04f, 0.08f});
    ambient    = dayDuskNight({0.42f, 0.46f, 0.55f}, {0.35f, 0.30f, 0.32f}, {0.07f, 0.08f, 0.13f});
}
