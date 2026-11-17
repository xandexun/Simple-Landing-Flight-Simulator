// aircraft.cpp - automatic landing guidance + simplified manual flight model
#include "sim/aircraft.h"
#include "world/airport.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>
#include <cstdio>

namespace {
constexpr float G          = 9.81f;
constexpr float KMH        = 3.6f;              // m/s -> km/h
constexpr float MASS       = 1000.0f;           // kg (C172 near max weight ~1100 kg)
constexpr float WING_AREA  = 16.2f;             // m^2
constexpr float RHO        = 1.225f;            // air density at sea level
constexpr float MAX_THRUST = 2600.0f;           // N (static)
constexpr float CL0        = 0.25f;
constexpr float CLA        = 5.0f;              // lift slope per radian
constexpr float STALL_AOA  = 0.2618f;           // 15 degrees
const float FLAP_CL[4] = {0.0f, 0.20f, 0.40f, 0.55f};
const float FLAP_CD[4] = {0.0f, 0.010f, 0.025f, 0.045f};

const float VAPP = TOLD::VREF_KMH / KMH;         // 33.4 m/s
const float VTD  = TOLD::VTD_KMH / KMH;          // 25.7 m/s

float moveTowards(float cur, float target, float maxStep) {
    return cur + glm::clamp(target - cur, -maxStep, maxStep);
}

float wrapPi(float a) {
    while (a > 3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

// rotation matrix around a unit axis (Rodrigues formula)
glm::mat3 axisAngle(const glm::vec3& a, float angle) {
    float c = std::cos(angle), s = std::sin(angle), t = 1.0f - c;
    glm::mat3 R(1.0f);
    R[0] = glm::vec3(t * a.x * a.x + c,       t * a.x * a.y + s * a.z, t * a.x * a.z - s * a.y);
    R[1] = glm::vec3(t * a.x * a.y - s * a.z, t * a.y * a.y + c,       t * a.y * a.z + s * a.x);
    R[2] = glm::vec3(t * a.x * a.z + s * a.y, t * a.y * a.z - s * a.x, t * a.z * a.z + c);
    return R;
}

// body axes of airplane.obj: +X = left wing, +Y = up, +Z = nose
const glm::vec3 BODY_PITCH_UP(-1.0f, 0.0f, 0.0f);   // positive rotation raises the nose
const glm::vec3 BODY_ROLL_RIGHT(0.0f, 0.0f, 1.0f);  // positive rotation lowers the right wing
const glm::vec3 BODY_YAW_RIGHT(0.0f, -1.0f, 0.0f);  // positive rotation turns the nose right

float liftCoefficient(float alpha, int flaps) {
    float lin = CL0 + CLA * alpha + FLAP_CL[flaps];
    if (alpha <= STALL_AOA) return std::fmax(lin, -0.8f);
    float clMax = CL0 + CLA * STALL_AOA + FLAP_CL[flaps];
    return std::fmax(0.3f, clMax - 2.5f * (alpha - STALL_AOA));   // lift drops after the stall
}

// Angle of attack needed to fly at speed v on a flight path angle gamma (steady flight)
float trimAlpha(float v, float gamma, int flaps) {
    float qS = 0.5f * RHO * v * v * WING_AREA;
    float cl = MASS * G * std::cos(gamma) / std::fmax(qS, 1.0f);
    float a = (cl - CL0 - FLAP_CL[flaps]) / CLA;
    return glm::clamp(a, -0.1f, 0.24f);
}

// ground roll from the manual for the current flaps: with fewer flaps the stall speed is higher,
// the speeds grow with sqrt(CLmax30 / CLmax) and the distance with the square of the speed
float manualGroundRoll(int flaps) {
    float clMax30 = CL0 + CLA * STALL_AOA + FLAP_CL[3];
    float clMax = CL0 + CLA * STALL_AOA + FLAP_CL[flaps];
    float k = std::sqrt(clMax30 / clMax);
    return TOLD::GROUND_ROLL_M * k * k;
}
} // namespace

// ------------------------------------------------------------------------------------
void Aircraft::reset(FlightMode m) {
    using namespace Airport;
    mode = m;
    const float dist = 3000.0f;                                  // metres before the aiming point
    const float gs = glm::radians(GLIDE_SLOPE_DEG);
    float h = dist * std::tan(gs);                               // ~157 m (515 ft) on the glide slope
    pos = glm::vec3(m == FlightMode::Auto ? 35.0f : 0.0f, h, AIM_POINT_Z - dist);
    vel = VAPP * glm::vec3(0.0f, -std::sin(gs), std::cos(gs));
    heading = roll = 0.0f;
    flaps = 3;
    alphaTrim_ = trimAlpha(VAPP, -gs, flaps);
    pitch = -gs + alphaTrim_;
    attitudeFromEuler();
    throttle = 0.44f;
    onGround = brakesOn = hasTouchdown = stopReported_ = false;
    phase = (m == FlightMode::Auto) ? Phase::Approach : Phase::Flight;
    touchdownSpeed = groundRoll = flareTime_ = 0.0f;
    smooth_ = Controls();
    propRpm = 700.0f + 2000.0f * throttle + 6.0f * airspeed();
    setMessage(m == FlightMode::Auto ? "Aterragem automática: aproximação final à pista 27, rampa de 3°"
                                     : "Modo manual: aproximação final à pista 27 - os comandos são seus");
}

void Aircraft::setMode(FlightMode m) {
    if (m == mode) return;
    mode = m;
    smooth_ = Controls();
    if (phase == Phase::Crashed) return;
    if (m == FlightMode::Auto) {
        flaps = 3;
        if (onGround) phase = (airspeed() > 0.5f) ? Phase::Rollout : Phase::Stopped;
        else phase = (agl() < TOLD::FLARE_HEIGHT_M) ? Phase::Flare : Phase::Approach;
        setMessage("Aterragem automática ligada");
    } else {
        phase = onGround ? Phase::Ground : Phase::Flight;
        alphaTrim_ = glm::clamp(angleOfAttack(), glm::radians(-6.0f), glm::radians(9.0f));
        setMessage("Modo manual - os comandos são seus");
    }
}

void Aircraft::changeThrottle(float delta) {
    if (mode != FlightMode::Manual) return;
    throttle = glm::clamp(throttle + delta, 0.0f, 1.0f);
}

void Aircraft::changeFlaps(int delta) {
    if (mode != FlightMode::Manual) return;
    flaps = glm::clamp(flaps + delta, 0, 3);
    setMessage("Flaps a " + std::to_string(flaps * 10) + "°", 2.0f);
}

void Aircraft::update(float dt, const Controls& c) {
    if (messageTime > 0.0f) messageTime -= dt;

    // propeller: ~700 rpm (idle) to ~2700 rpm (full power), plus the effect of the airflow.
    // It stops after a crash, and the pilot shuts the engine down after landing and stopping.
    bool stoppedOnGround = onGround && hasTouchdown && airspeed() < 0.3f && (mode == FlightMode::Auto || throttle < 0.01f);
    bool engineOff = phase == Phase::Crashed || phase == Phase::Stopped || stoppedOnGround;
    float rpmTarget = engineOff ? 0.0f : 700.0f + 2000.0f * throttle + 6.0f * airspeed();
    float rate = (phase == Phase::Crashed) ? 4000.0f : engineOff ? 350.0f : 900.0f;   // rpm per second (slow spin-down)
    propRpm += glm::clamp(rpmTarget - propRpm, -rate * dt, rate * dt);
    propAngle = std::fmod(propAngle + propRpm / 60.0f * 6.2831853f * dt, 6.2831853f);

    if (phase == Phase::Crashed) return;
    if (mode == FlightMode::Auto) updateAuto(dt);
    else updateManual(dt, c);
}

void Aircraft::reportStop() {
    char buf[160];
    std::snprintf(buf, sizeof buf, "Parado - corrida no solo %.0f m (manual do avião: %.0f m). R para recomeçar",
                  groundRoll, manualGroundRoll(flaps));
    setMessage(buf, 1e9f);
}

// ------------------------------------------------------------------------------------
// Automatic landing: guidance laws that drive the aircraft along the ILS glide slope,
// then flare, touchdown, lower the nose wheel, brake and stop.
void Aircraft::updateAuto(float dt) {
    using namespace Airport;
    const float gs = glm::radians(GLIDE_SLOPE_DEG);
    float ground = groundHeight(pos.x, pos.z);

    if (!onGround) {
        float v = airspeed();
        float h = pos.y - ground;
        if (phase != Phase::Flare && h < TOLD::FLARE_HEIGHT_M) {
            phase = Phase::Flare;
            flareTime_ = 0.0f;
            setMessage("Arredondamento - motor em ralenti", 4.0f);
        }
        if (phase != Phase::Flare) phase = Phase::Approach;

        // speed: Vref on the approach, bleeding towards the touchdown speed in the flare
        float targetV = (phase == Phase::Flare) ? VTD : VAPP;
        v = moveTowards(v, targetV, (phase == Phase::Flare ? 0.9f : 1.0f) * dt);
        throttle = moveTowards(throttle, phase == Phase::Flare ? 0.0f : 0.44f, 0.3f * dt);

        // vertical guidance
        float vh = std::sqrt(std::fmax(v * v - vel.y * vel.y, 1.0f));
        float vyDes;
        if (phase == Phase::Approach) {
            float dist = AIM_POINT_Z - pos.z;
            float hGlide = dist * std::tan(gs);                       // glide slope height
            vyDes = -vh * std::tan(gs) + glm::clamp((hGlide - pos.y) / 8.0f, -3.0f, 3.0f);
        } else {
            flareTime_ += dt;
            vyDes = -std::fmax(0.35f, h * 0.30f);                     // exponential flare
        }
        float vy = moveTowards(vel.y, vyDes, 1.5f * dt);
        vy = glm::clamp(vy, -0.5f * v, 0.5f * v);
        vh = std::sqrt(std::fmax(v * v - vy * vy, 0.0f));

        // lateral guidance (localizer): converge to the centre line x = 0
        float vxDes = glm::clamp(-pos.x / 12.0f, -0.25f * vh, 0.25f * vh);
        float headingDes = std::asin(glm::clamp(vxDes / std::fmax(vh, 1.0f), -0.5f, 0.5f));
        float headingRate = glm::clamp(wrapPi(headingDes - heading) * 0.6f,
                                       glm::radians(-3.0f), glm::radians(3.0f));
        heading = wrapPi(heading + headingRate * dt);
        float rollDes = std::atan(-v * headingRate / G);              // coordinated turn bank
        // a large bank (e.g. inverted when the autopilot is engaged) is recovered faster
        float rollRate = std::fabs(wrapPi(rollDes - roll)) > glm::radians(20.0f) ? 60.0f : 8.0f;
        roll = moveTowards(roll, rollDes, glm::radians(rollRate) * dt);

        vel = glm::vec3(std::sin(heading) * vh, vy, std::cos(heading) * vh);
        pos += vel * dt;

        // attitude = flight path angle + angle of attack (+ extra nose up during the flare)
        float gamma = std::asin(glm::clamp(vy / std::fmax(v, 1.0f), -1.0f, 1.0f));
        float flareExtra = (phase == Phase::Flare) ? glm::radians(1.0f) * std::fmin(1.0f, flareTime_ / 2.5f) : 0.0f;
        float pitchDes = gamma + trimAlpha(v, gamma, flaps) + flareExtra;
        float pitchRate = std::fabs(pitchDes - pitch) > glm::radians(10.0f) ? 30.0f : 4.0f;
        pitch = moveTowards(pitch, pitchDes, glm::radians(pitchRate) * dt);
        attitudeFromEuler();

        ground = groundHeight(pos.x, pos.z);
        if (pos.y <= ground) groundContact(true);
    } else {
        // ---- rollout on the runway ----
        if (phase == Phase::Stopped) { vel = glm::vec3(0.0f); attitudeFromEuler(); return; }
        phase = Phase::Rollout;
        float v = glm::length(glm::vec2(vel.x, vel.z));
        roll = moveTowards(roll, 0.0f, glm::radians(10.0f) * dt);
        pitch = moveTowards(pitch, 0.0f, glm::radians(3.0f) * dt);    // lower the nose wheel
        bool noseDown = pitch <= glm::radians(0.3f);
        brakesOn = noseDown;
        throttle = 0.0f;
        float decel = noseDown ? 2.0f : 0.7f;                         // braking vs aerodynamic drag
        float headingDes = glm::clamp(-pos.x * 0.02f, -0.08f, 0.08f);
        heading = moveTowards(heading, headingDes, glm::radians(4.0f) * dt);
        v = std::fmax(0.0f, v - decel * dt);
        vel = glm::vec3(std::sin(heading) * v, 0.0f, std::cos(heading) * v);
        pos += vel * dt;
        pos.y = groundHeight(pos.x, pos.z);
        groundRoll += v * dt;
        attitudeFromEuler();
        if (v <= 0.05f) {
            vel = glm::vec3(0.0f);
            phase = Phase::Stopped;
            brakesOn = true;
            reportStop();
        }
    }
}

// ------------------------------------------------------------------------------------
// Manual flight: forces (lift, drag, thrust, gravity, ground friction) integrated with
// semi-implicit Euler.
// In the air the attitude is a rotation matrix with no limits, so the aircraft can roll
// all the way round, loop and fly inverted:
//   - the nose follows the flight path (the aircraft is stable like a weathervane), so the
//     angle of attack only changes with the elevator;
//   - the elevator commands an angle of attack (stick released = trimmed angle of attack,
//     pull = more lift -> the path curves "up" relative to the pilot, also when inverted);
//   - the ailerons give a roll rate around the nose axis, the rudder a yaw rate.
// On the ground the attitude is kept as Euler angles (nose wheel steering, rotation).
void Aircraft::updateManual(float dt, const Controls& c) {
    using namespace Airport;
    smooth_.elevator = moveTowards(smooth_.elevator, c.elevator, 3.0f * dt);
    smooth_.aileron  = moveTowards(smooth_.aileron,  c.aileron,  3.0f * dt);
    smooth_.rudder   = moveTowards(smooth_.rudder,   c.rudder,   3.0f * dt);
    brakesOn = c.brakes;

    float v = airspeed();
    float q = glm::clamp(v / 30.0f, 0.15f, 1.3f);       // control effectiveness grows with speed
    float alpha = angleOfAttack();
    bool stalled = !onGround && v > 1.0f && alpha > STALL_AOA;

    // ---- attitude ----
    if (!onGround) {
        // elevator -> commanded angle of attack (holding the stick also re-trims slowly)
        alphaTrim_ = glm::clamp(alphaTrim_ + smooth_.elevator * glm::radians(1.0f) * dt,
                                glm::radians(-6.0f), glm::radians(9.0f));
        float alphaCmd = alphaTrim_ + smooth_.elevator * glm::radians(smooth_.elevator > 0.0f ? 11.0f : 16.0f);
        float pitchRate = glm::clamp(3.0f * (alphaCmd - alpha), glm::radians(-45.0f), glm::radians(45.0f)) * std::fmin(q, 1.0f);
        if (stalled) {
            pitchRate = std::fmin(pitchRate, 0.0f) - glm::radians(10.0f);    // the nose drops in a stall
            if (messageTime <= 0.0f || message != "PERDA DE SUSTENTAÇÃO!") setMessage("PERDA DE SUSTENTAÇÃO!", 1.0f);
        }
        float rollRate = smooth_.aileron * glm::radians(80.0f) * q;
        if (std::fabs(smooth_.aileron) < 0.05f && std::fabs(roll) < glm::radians(30.0f))
            rollRate -= roll * 0.3f;                                          // gentle roll stability near wings level
        // sideslip: velocity to the right of the nose -> the fin turns the nose to the right
        glm::vec3 fwd = forward(), rgt = right();
        float beta = (v > 1.0f) ? std::atan2(glm::dot(vel, rgt), std::fmax(glm::dot(vel, fwd), 0.1f)) : 0.0f;
        float yawRate = 2.0f * beta + smooth_.rudder * glm::radians(12.0f) * q;

        attitude = attitude * axisAngle(BODY_PITCH_UP, pitchRate * dt)
                            * axisAngle(BODY_ROLL_RIGHT, rollRate * dt)
                            * axisAngle(BODY_YAW_RIGHT, yawRate * dt);
    } else {
        if (smooth_.elevator > 0.05f) pitch += smooth_.elevator * glm::radians(10.0f) * q * dt;  // rotate
        else pitch = moveTowards(pitch, 0.0f, glm::radians(4.0f) * dt);                       // nose wheel
        pitch = glm::clamp(pitch, 0.0f, glm::radians(20.0f));
        roll = moveTowards(roll, 0.0f, glm::radians(30.0f) * dt);
        heading = wrapPi(heading - smooth_.rudder * glm::radians(25.0f) * std::fmin(1.0f, v / 8.0f) * dt);  // nose wheel steering
        if (pitch > glm::radians(13.0f)) setMessage("CAUDA BATEU NA PISTA!", 3.0f);
        attitudeFromEuler();
    }

    // ---- forces ----
    glm::vec3 fwd = forward(), upv = up(), rgt = right();
    glm::vec3 F(0.0f, -MASS * G, 0.0f);                                    // weight
    F += fwd * (throttle * MAX_THRUST * std::fmax(0.0f, 1.0f - v / 90.0f)); // thrust
    float liftY = 0.0f;
    if (v > 1.0f) {
        glm::vec3 vd = vel / v;
        float cl = liftCoefficient(alpha, flaps);
        float cd = 0.032f + 0.045f * cl * cl + FLAP_CD[flaps];
        float qS = 0.5f * RHO * v * v * WING_AREA;
        glm::vec3 liftDir = upv - glm::dot(upv, vd) * vd;
        if (glm::length(liftDir) > 1e-4f) liftDir = glm::normalize(liftDir);
        F += liftDir * (cl * qS);
        F -= vd * (cd * qS);
        liftY = liftDir.y * cl * qS;
    }

    if (onGround) {
        float normal = std::fmax(0.0f, MASS * G - liftY);
        glm::vec3 fh = glm::normalize(glm::vec3(fwd.x, 0.0f, fwd.z));
        float vf = glm::dot(vel, fh);
        vf += glm::dot(F, fh) / MASS * dt;                                 // thrust and drag
        float mu = 0.02f + (brakesOn ? 0.40f : 0.0f) + (isPaved(pos.x, pos.z) ? 0.0f : 0.08f);
        float dv = mu * normal / MASS * dt;                                // rolling friction + brakes
        vf = (vf > 0.0f) ? std::fmax(0.0f, vf - dv) : std::fmin(0.0f, vf + dv);
        float vy = vel.y + (F.y + normal) / MASS * dt;                     // > 0 only when lift > weight
        vel = fh * vf + glm::vec3(0.0f, std::fmax(vy, 0.0f), 0.0f);        // wheels kill sideways motion
    } else {
        glm::vec3 dirOld = (v > 1.0f) ? vel / v : fwd;
        vel += F / MASS * dt;
        float speed = glm::length(vel);
        // the nose follows the turn of the flight path (lift and gravity bend it)
        if (speed > 1.0f && v > 1.0f) {
            glm::vec3 dirNew = vel / speed;
            glm::vec3 axis = glm::cross(dirOld, dirNew);
            float s = glm::length(axis);
            if (s > 1e-7f) attitude = axisAngle(axis / s, std::atan2(s, glm::dot(dirOld, dirNew))) * attitude;
        }
        // side force: the fuselage and fin turn the velocity towards the nose (little sideslip)
        float side = glm::dot(vel, rgt);
        vel -= rgt * side * std::fmin(1.0f, 2.5f * dt);
        if (glm::length(vel) > 1e-3f) vel = glm::normalize(vel) * speed;

        // keep the matrix a pure rotation (numerical drift) and update the Euler angles
        glm::vec3 nose = glm::normalize(attitude[2]);
        glm::vec3 top = glm::normalize(attitude[1] - glm::dot(attitude[1], nose) * nose);
        attitude[0] = glm::cross(top, nose);
        attitude[1] = top;
        attitude[2] = nose;
        eulerFromAttitude();
    }
    pos += vel * dt;

    float ground = groundHeight(pos.x, pos.z);
    if (pos.y <= ground) {
        if (!onGround) groundContact(false);
        if (phase == Phase::Crashed) return;
        pos.y = ground;
        vel.y = std::fmax(vel.y, 0.0f);
        attitudeFromEuler();
    } else if (pos.y > ground + 0.05f && onGround) {
        onGround = false;
        alphaTrim_ = glm::clamp(angleOfAttack(), glm::radians(-6.0f), glm::radians(9.0f));
        setMessage("Descolou", 2.0f);
    }

    if (onGround) {
        if (hasTouchdown) groundRoll += glm::length(vel) * dt;
        phase = Phase::Ground;
        if (hasTouchdown && !stopReported_ && glm::length(vel) < 0.2f) {
            stopReported_ = true;
            reportStop();
        }
    } else {
        phase = Phase::Flight;
    }
}

// Called on the first frame the wheels touch the ground: evaluates the landing
void Aircraft::groundContact(bool fromAuto) {
    using namespace Airport;
    pos.y = groundHeight(pos.x, pos.z);
    if (onGround) return;
    onGround = true;
    hasTouchdown = true;
    stopReported_ = false;
    touchdownSpeed = airspeed();
    groundRoll = 0.0f;

    float sink = -vel.y;                                       // m/s
    char buf[200] = "";
    // a bad landing destroys the aircraft: hitting the ground too fast (> 3 m/s down), with the
    // wings very inclined, nose first, or much too fast
    if (vel.y < -3.0f) std::snprintf(buf, sizeof buf, "descida a %.1f m/s", sink);
    else if (std::fabs(roll) > glm::radians(25.0f)) std::snprintf(buf, sizeof buf, "asas inclinadas %.0f°", glm::degrees(std::fabs(roll)));
    else if (pitch < glm::radians(-6.0f)) std::snprintf(buf, sizeof buf, "tocou de nariz");
    else if (touchdownSpeed > 50.0f) std::snprintf(buf, sizeof buf, "velocidade de %.0f km/h", touchdownSpeed * KMH);
    if (buf[0]) { crash(buf); return; }
    const char* quality = sink < 1.0f ? "suave" : sink < 2.0f ? "boa" : "firme";
    bool runway = onRunway(pos.x, pos.z);
    std::snprintf(buf, sizeof buf, "TOQUE %.1f m/s (%s), %.0f km/h, %.0f m depois do início da pista%s",
                  sink, quality, touchdownSpeed * KMH, pos.z - THRESHOLD_Z, runway ? "" : " - FORA DA PISTA!");
    setMessage(buf, 10.0f);
    vel.y = 0.0f;
    phase = fromAuto ? Phase::Rollout : Phase::Ground;
}

void Aircraft::crash(const std::string& reason) {
    if (phase == Phase::Crashed) return;
    phase = Phase::Crashed;
    vel = glm::vec3(0.0f);
    throttle = 0.0f;
    setMessage("ACIDENTE! O avião explodiu (" + reason + ") - R para recomeçar", 1e9f);
}

// ------------------------------------------------------------------------------------
void Aircraft::attitudeFromEuler() {
    glm::mat4 R(1.0f);
    R = glm::rotate(R, heading, glm::vec3(0.0f, 1.0f, 0.0f));   // yaw
    R = glm::rotate(R, -pitch,  glm::vec3(1.0f, 0.0f, 0.0f));   // pitch (nose up)
    R = glm::rotate(R, roll,    glm::vec3(0.0f, 0.0f, 1.0f));   // roll (right wing down)
    attitude = glm::mat3(R);
}

void Aircraft::eulerFromAttitude() {
    const glm::vec3 nose = attitude[2], left = attitude[0], top = attitude[1];
    pitch = std::asin(glm::clamp(nose.y, -1.0f, 1.0f));
    // nose pointing straight up/down: take the heading from the top of the aircraft
    heading = (std::fabs(nose.y) < 0.999f) ? std::atan2(nose.x, nose.z)
                                           : std::atan2(-top.x * nose.y, -top.z * nose.y);
    roll = std::atan2(left.y, top.y);
}

float Aircraft::angleOfAttack() const {
    if (airspeed() < 1.0f) return 0.0f;
    return std::atan2(-glm::dot(vel, up()), glm::dot(vel, forward()));
}

glm::mat4 Aircraft::modelMatrix() const {
    glm::mat4 M = glm::translate(glm::mat4(1.0f), pos) * rotation();
    M = glm::scale(M, glm::vec3(MODEL_SCALE));
    return glm::translate(M, -modelPivot());
}

float Aircraft::agl() const { return pos.y - Airport::groundHeight(pos.x, pos.z); }

float Aircraft::headingDeg() const {
    float h = Airport::RUNWAY_HEADING - glm::degrees(heading);
    while (h < 0.0f) h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;
    return h;
}

float Aircraft::distanceToThreshold() const { return Airport::THRESHOLD_Z - pos.z; }

float Aircraft::glideslopeDeviationDots() const {
    float dist = Airport::AIM_POINT_Z - pos.z;
    if (dist < 10.0f) return 0.0f;
    float angle = glm::degrees(std::atan2(pos.y, dist));
    return glm::clamp((angle - Airport::GLIDE_SLOPE_DEG) / 0.35f, -2.5f, 2.5f);   // 0.35 deg per dot
}

float Aircraft::localizerDeviationDots() const {
    float dist = Airport::LOCALIZER_Z - pos.z;
    if (dist < 10.0f) return 0.0f;
    float angle = glm::degrees(std::atan2(pos.x, dist));
    return glm::clamp(angle / 1.25f, -2.5f, 2.5f);                                // 1.25 deg per dot
}

const char* Aircraft::phaseName() const {
    switch (phase) {
        case Phase::Approach: return "Aproximação";
        case Phase::Flare:    return "Arredondamento";
        case Phase::Rollout:  return "Travagem na pista";
        case Phase::Stopped:  return "Parado (motor desligado)";
        case Phase::Flight:   return "Em voo";
        case Phase::Ground:   return "No solo";
        case Phase::Crashed:  return "Acidente";
    }
    return "";
}
