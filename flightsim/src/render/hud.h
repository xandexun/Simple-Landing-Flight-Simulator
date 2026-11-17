// hud.h - 2D overlay: flight data, ILS indicators, cockpit instrument panel and help.
// Everything is drawn as coloured triangles (shapes) or textured quads (text, font atlas).
#pragma once
#include <string>
#include <vector>
#include "engine/gl_utils.h"

class Aircraft;
class Camera;

struct HudInfo {
    float fps = 0.0f;
    float timeScale = 1.0f;
    bool paused = false;
    bool showHelp = false;
    bool showHud = true;
    bool showApproach = true;   // approach guide panel (right side)
    bool abbrev = false;        // show aviation abbreviations (IAS, ALT, VS...) instead of full names
};

// list of commands: shown on the screen with H and printed in the terminal at startup
extern const std::vector<std::string> HELP_LINES;

class Hud {
public:
    bool init(const std::string& rootDir);
    void draw(const Aircraft& ac, const Camera& cam, const HudInfo& info, int width, int height);

private:
    Shader shader_;
    GLuint fontTex_ = 0, vao_ = 0, vbo_ = 0;
    std::vector<float> verts_;
    int w_ = 0, h_ = 0;
    float s_ = 1.0f;   // UI scale (depends on the window height)
    bool ab_ = false;  // abbreviations instead of full names
    const char* L(const char* full, const char* abbr) const { return ab_ ? abbr : full; }

    // primitives (angles in radians, clockwise from 12 o'clock)
    void vertex(float x, float y, float u, float v, const glm::vec4& c);
    void triangle(glm::vec2 a, glm::vec2 b, glm::vec2 c, const glm::vec4& col);
    void rect(float x, float y, float w, float h, const glm::vec4& c);
    void line(glm::vec2 a, glm::vec2 b, float thick, const glm::vec4& c);
    void arc(float cx, float cy, float r, float a0, float a1, float thick, const glm::vec4& c);
    void circle(float cx, float cy, float r, float thick, const glm::vec4& c);
    void disc(float cx, float cy, float r, const glm::vec4& c);
    void polygon(const std::vector<glm::vec2>& convex, const glm::vec4& c);
    void text(float x, float y, const std::string& str, const glm::vec4& c, float scale);
    void centeredText(float cx, float y, const std::string& str, const glm::vec4& c, float scale);
    float textWidth(const std::string& str, float scale) const;
    void panel(float x, float y, float w, float h);
    void dial(float cx, float cy, float r, const std::string& value);
    void tick(float cx, float cy, float r, float a, float inner, float thick);
    void tickLabel(float cx, float cy, float r, float a, const std::string& l, float scale, float dy,
                   const glm::vec4& c = glm::vec4(1.0f));

    // panels and instruments
    void statusPanel(const Aircraft& ac, const Camera& cam, const HudInfo& info);
    void flightDataPanel(const Aircraft& ac);
    void approachPanel(const Aircraft& ac);
    void messageLine(const Aircraft& ac);
    void helpOverlay();
    void cockpitHud(const Aircraft& ac, const Camera& cam);
    void cockpitPanel(const Aircraft& ac);
    void gaugeAirspeed(float cx, float cy, float r, float kmh);
    void gaugeAttitude(float cx, float cy, float r, float pitchDeg, float rollDeg);
    void gaugeAltimeter(float cx, float cy, float r, float m);
    void gaugeVSI(float cx, float cy, float r, float ms);
    void gaugeHeading(float cx, float cy, float r, float hdg);
};
