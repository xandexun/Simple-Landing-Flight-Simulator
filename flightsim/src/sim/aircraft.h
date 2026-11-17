// aircraft.h - aircraft state, automatic landing (autoland) and manual flight physics.
// No OpenGL here, so it can be tested on its own.
//
// World axes: +Y up, the runway is along Z and the aircraft lands flying towards +Z.
//   pos      - position of the main wheels contact point (metres)
//   vel      - velocity (m/s)
//   attitude - rotation matrix body -> world (columns: left wing +X, up +Y, nose +Z of the model).
//              It has no limits: the aircraft can roll 360 degrees, loop and fly inverted.
//   heading, pitch, roll - Euler angles derived from the attitude (for the instruments):
//              heading 0 = flying towards +Z, positive turns towards +X (left);
//              pitch positive = nose up; roll positive = right wing down (+-180 = inverted)
#pragma once
#include <string>
#include <glm/glm.hpp>

enum class FlightMode { Auto, Manual };
enum class Phase { Approach, Flare, Rollout, Stopped, Flight, Ground, Crashed };

struct Controls {
    float elevator = 0.0f;   // -1..1  (+1 = nose up)
    float aileron  = 0.0f;   // -1..1  (+1 = roll right)
    float rudder   = 0.0f;   // -1..1  (+1 = yaw right)
    bool  brakes   = false;
};

// Landing data from the Cessna 172S manual (65 kt, 50 kt, 575 ft), flaps 30, sea level, in metric units
namespace TOLD {
    constexpr float VREF_KMH       = 120.4f;  // final approach speed
    constexpr float VTD_KMH        = 92.6f;   // touchdown speed
    constexpr float GROUND_ROLL_M  = 175.3f;  // landing ground roll
    constexpr float FLARE_HEIGHT_M = 6.0f;    // start of the flare (wheel height)
}

class Aircraft {
public:
    // geometry of airplane.obj (model units)
    static constexpr float MODEL_SCALE = 0.085f;                            // 130 units wingspan -> ~11 m
    static glm::vec3 modelPivot() { return glm::vec3(0.0f, -0.2f, -5.0f); } // main wheels
    static glm::vec3 cockpitEye() { return glm::vec3(0.0f, 1.55f, 0.95f); } // metres, from pivot

    glm::vec3 pos{0.0f}, vel{0.0f};
    float heading = 0.0f, pitch = 0.0f, roll = 0.0f;
    glm::mat3 attitude{1.0f};
    float throttle = 0.0f;       // 0..1
    int   flaps = 3;             // 0..3 -> 0, 10, 20, 30 degrees
    bool  onGround = false;
    bool  brakesOn = false;
    FlightMode mode = FlightMode::Auto;
    Phase phase = Phase::Approach;

    bool  hasTouchdown = false;
    float touchdownSpeed = 0.0f; // m/s
    float groundRoll = 0.0f;     // metres travelled on the ground since touchdown
    float propRpm = 0.0f;        // engine / propeller revolutions per minute
    float propAngle = 0.0f;      // propeller rotation angle (radians)
    std::string message;         // shown on the HUD
    float messageTime = 0.0f;

    void reset(FlightMode m);
    void update(float dt, const Controls& c);     // call with small dt (<= 1/60 s)
    void setMode(FlightMode m);
    void changeThrottle(float delta);
    void changeFlaps(int delta);
    void crash(const std::string& reason);        // bad landing or hit a building: it explodes

    glm::mat4 rotation() const { return glm::mat4(attitude); }   // body -> world
    glm::mat4 modelMatrix() const;                                // model matrix for airplane.obj
    glm::vec3 forward() const { return attitude[2]; }
    glm::vec3 up() const      { return attitude[1]; }
    glm::vec3 right() const   { return -attitude[0]; }
    float airspeed() const { return glm::length(vel); }
    float verticalSpeed() const { return vel.y; }
    float agl() const;                     // height of the wheels above the ground
    float headingDeg() const;              // compass heading (runway 27 = 270)
    float distanceToThreshold() const;     // metres, negative after the threshold
    float angleOfAttack() const;           // radians
    float glideslopeDeviationDots() const; // + = above the glide slope
    float localizerDeviationDots() const;  // + = aircraft left of the centre line
    const char* phaseName() const;

private:
    Controls smooth_;                  // control surfaces move smoothly
    float flareTime_ = 0.0f;
    float alphaTrim_ = 0.0f;           // angle of attack held with the stick released (manual mode)
    bool  stopReported_ = false;
    void attitudeFromEuler();          // attitude <- heading, pitch, roll
    void eulerFromAttitude();          // heading, pitch, roll <- attitude
    void updateAuto(float dt);
    void updateManual(float dt, const Controls& c);
    void groundContact(bool fromAuto);
    void reportStop();
    void setMessage(const std::string& m, float seconds = 8.0f) { message = m; messageTime = seconds; }
};
