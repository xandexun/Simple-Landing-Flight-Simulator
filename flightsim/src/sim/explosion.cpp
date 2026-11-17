// explosion.cpp
#include "sim/explosion.h"
#include <algorithm>
#include <cmath>

float Particle::alpha() const {
    float t = age / life;
    switch (kind) {
        case Fire:  return (1.0f - t) * std::min(1.0f, t * 12.0f + 0.3f);
        case Spark: return 1.0f - t;
        case Smoke: return 0.55f * std::min(1.0f, t * 6.0f) * (1.0f - t);
    }
    return 0.0f;
}

float Explosion::rnd() {
    seed_ = seed_ * 1664525u + 1013904223u;
    return (seed_ >> 8) / float(1 << 24);
}

glm::vec3 Explosion::rndDir() {
    float z = rnd(-1.0f, 1.0f), a = rnd(0.0f, 6.2831853f), r = std::sqrt(1.0f - z * z);
    return glm::vec3(r * std::cos(a), z, r * std::sin(a));
}

void Explosion::start(const glm::vec3& where) {
    active = started = true;
    origin_ = where;
    time_ = fireAcc_ = smokeAcc_ = 0.0f;
    particles.clear();
    // fireball: big, bright, expanding quickly
    for (int i = 0; i < 70; ++i) {
        glm::vec3 d = rndDir();
        d.y = std::fabs(d.y) * 0.8f + 0.2f;
        particles.push_back({Particle::Fire, where + d * rnd(0.0f, 3.0f), d * rnd(4.0f, 22.0f),
                             glm::vec3(1.0f, rnd(0.55f, 0.85f), rnd(0.15f, 0.35f)), 0.0f, rnd(0.8f, 2.2f),
                             rnd(3.0f, 6.0f), rnd(12.0f, 24.0f)});
    }
    // sparks and burning debris thrown in all directions (they fall with gravity)
    for (int i = 0; i < 90; ++i) {
        glm::vec3 d = rndDir();
        d.y = std::fabs(d.y) + 0.3f;
        Particle p{Particle::Spark, where + glm::vec3(0.0f, 1.0f, 0.0f), glm::normalize(d) * rnd(10.0f, 45.0f),
                   glm::vec3(1.0f, rnd(0.6f, 0.9f), 0.3f), 0.0f, rnd(1.0f, 3.0f), rnd(0.8f, 1.5f), 0.3f};
        particles.push_back(p);
    }
    // first thick black smoke
    for (int i = 0; i < 25; ++i) emitSmoke();
}

void Explosion::emitFire(float scale) {
    glm::vec3 off(rnd(-5.0f, 5.0f), rnd(1.5f, 3.0f), rnd(-5.0f, 5.0f));   // above the ground so it does not cut the sprite
    Particle p{Particle::Fire, origin_ + off, glm::vec3(rnd(-1.0f, 1.0f), rnd(3.0f, 7.0f), rnd(-1.0f, 1.0f)),
               glm::vec3(1.0f, rnd(0.45f, 0.75f), 0.15f), 0.0f, rnd(0.5f, 1.2f), 3.0f * scale, 1.0f * scale};
    particles.push_back(p);
}

void Explosion::emitSmoke() {
    glm::vec3 off(rnd(-4.0f, 4.0f), rnd(1.0f, 4.0f), rnd(-4.0f, 4.0f));
    float g = rnd(0.06f, 0.16f);
    Particle p{Particle::Smoke, origin_ + off, glm::vec3(rnd(-1.0f, 1.0f) + 2.0f, rnd(4.0f, 8.0f), rnd(-1.0f, 1.0f)),
               glm::vec3(g), 0.0f, rnd(7.0f, 11.0f), rnd(5.0f, 8.0f), rnd(30.0f, 45.0f)};
    particles.push_back(p);
}

void Explosion::update(float dt) {
    if (!active) return;
    time_ += dt;
    // the wreck keeps burning for ~20 s, the smoke lasts longer
    float burn = std::max(0.0f, 1.0f - time_ / 20.0f);
    fireAcc_ += dt * 45.0f * burn;
    while (fireAcc_ >= 1.0f) { emitFire(0.6f + burn); fireAcc_ -= 1.0f; }
    smokeAcc_ += dt * (time_ < 35.0f ? 10.0f : 0.0f);
    while (smokeAcc_ >= 1.0f) { emitSmoke(); smokeAcc_ -= 1.0f; }

    for (Particle& p : particles) {
        p.age += dt;
        if (p.kind == Particle::Spark) {
            p.vel.y -= 9.81f * dt;
            p.pos += p.vel * dt;
            if (p.pos.y < origin_.y) { p.pos.y = origin_.y; p.vel *= 0.3f; }
        } else {
            p.vel *= std::exp(-(p.kind == Particle::Fire ? 2.5f : 0.4f) * dt);   // air drag
            p.vel.y += (p.kind == Particle::Fire ? 6.0f : 1.5f) * dt;             // hot air rises
            p.pos += p.vel * dt;
        }
        if (p.kind == Particle::Fire)   // yellow -> orange -> dark red as it cools down
            p.color = glm::mix(glm::vec3(1.0f, 0.8f, 0.35f), glm::vec3(0.7f, 0.12f, 0.02f), p.age / p.life);
    }
    particles.erase(std::remove_if(particles.begin(), particles.end(),
                                   [](const Particle& p) { return p.age >= p.life; }), particles.end());
    if (time_ > 50.0f && particles.empty()) active = false;
}
