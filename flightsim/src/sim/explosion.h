// explosion.h - particle system for the explosion when the aircraft crashes:
// a fireball, sparks thrown in all directions, flames that keep burning and a smoke column.
// No OpenGL here: the renderer draws the particles as point sprites.
#pragma once
#include <vector>
#include <glm/glm.hpp>

struct Particle {
    enum Kind { Fire, Spark, Smoke };
    Kind kind;
    glm::vec3 pos, vel;
    glm::vec3 color;
    float age, life;          // seconds
    float size0, size1;       // apparent size (metres) at birth and at the end of its life
    float size() const { return size0 + (size1 - size0) * (age / life); }
    float alpha() const;      // opacity / brightness (0..1) along the life
};

class Explosion {
public:
    bool active = false;
    bool started = false;     // start() was called (stays true until clear())
    std::vector<Particle> particles;

    void start(const glm::vec3& where);
    void clear() { active = started = false; particles.clear(); }
    void update(float dt);

private:
    glm::vec3 origin_{0.0f};
    float time_ = 0.0f;
    float fireAcc_ = 0.0f, smokeAcc_ = 0.0f;
    unsigned seed_ = 12345u;
    float rnd();                                // 0..1
    float rnd(float a, float b) { return a + (b - a) * rnd(); }
    glm::vec3 rndDir();                         // random unit vector
    void emitFire(float scale);
    void emitSmoke();
};
