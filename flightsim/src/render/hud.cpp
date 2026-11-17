// hud.cpp
#include "render/hud.h"
#include "sim/aircraft.h"
#include "world/airport.h"
#include "sim/camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>

namespace {
const float PI = 3.14159265f;
const float CELL_W = 16.0f, CELL_H = 32.0f, ATLAS_W = 256.0f, ATLAS_H = 448.0f, ADVANCE = 15.0f;
const glm::vec4 WHITE(1.0f), GREEN(0.35f, 1.0f, 0.45f, 1.0f), YELLOW(1.0f, 0.85f, 0.25f, 1.0f),
                RED(1.0f, 0.3f, 0.25f, 1.0f), CYAN(0.4f, 0.9f, 1.0f, 1.0f), GREY(0.7f, 0.7f, 0.7f, 1.0f),
                MAGENTA(1.0f, 0.4f, 1.0f, 1.0f);
const float KMH = 3.6f;   // m/s -> km/h

// Portuguese number format: decimal comma ("3,5" instead of "3.5")
std::string decimalComma(std::string s) {
    for (size_t i = 1; i + 1 < s.size(); ++i)
        if (s[i] == '.' && std::isdigit(static_cast<unsigned char>(s[i - 1])) && std::isdigit(static_cast<unsigned char>(s[i + 1])))
            s[i] = ',';
    return s;
}
std::string fmt(const char* f, double a) { char b[128]; std::snprintf(b, sizeof b, f, a); return decimalComma(b); }

// UTF-8 -> character codes of the font atlas (Latin-1: 32..255, so Portuguese accents work)
std::vector<int> decodeUtf8(const std::string& s) {
    std::vector<int> out;
    for (size_t i = 0; i < s.size();) {
        unsigned char c = static_cast<unsigned char>(s[i]);
        int code = '?', len = 1;
        if (c < 0x80) code = c;
        else if ((c & 0xE0) == 0xC0 && i + 1 < s.size()) { code = ((c & 0x1F) << 6) | (s[i + 1] & 0x3F); len = 2; }
        else if ((c & 0xF0) == 0xE0) len = 3;
        else if ((c & 0xF8) == 0xF0) len = 4;
        out.push_back((code >= 32 && code <= 255 && !(code >= 127 && code < 160)) ? code : '?');
        i += len;
    }
    return out;
}

// keeps the part of a convex polygon where dot(p - o, n) >= 0
std::vector<glm::vec2> clipHalfPlane(const std::vector<glm::vec2>& in, glm::vec2 o, glm::vec2 n) {
    std::vector<glm::vec2> out;
    for (size_t i = 0; i < in.size(); ++i) {
        glm::vec2 a = in[i], b = in[(i + 1) % in.size()];
        float da = glm::dot(a - o, n), db = glm::dot(b - o, n);
        if (da >= 0.0f) out.push_back(a);
        if ((da >= 0.0f) != (db >= 0.0f)) out.push_back(a + (b - a) * (da / (da - db)));
    }
    return out;
}

// point at distance r from the centre, angle a measured clockwise from 12 o'clock
glm::vec2 polar(float cx, float cy, float r, float a) { return {cx + r * std::sin(a), cy - r * std::cos(a)}; }
} // namespace

bool Hud::init(const std::string& root) {
    if (!shader_.load(root + "shaders/hud.vert", root + "shaders/hud.frag")) return false;
    Image img;
    if (!loadPNM(root + "assets/font.pgm", img)) return false;
    fontTex_ = createTexture(img, false, false);
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    vertexAttribs({2, 2, 4});   // position, uv, colour
    glBindVertexArray(0);
    return true;
}

// ------------------------------------------------------------------ primitives
void Hud::vertex(float x, float y, float u, float v, const glm::vec4& c) {
    verts_.insert(verts_.end(), {x, y, u, v, c.r, c.g, c.b, c.a});
}

void Hud::triangle(glm::vec2 a, glm::vec2 b, glm::vec2 c, const glm::vec4& col) {   // uv = -1: no texture
    vertex(a.x, a.y, -1, -1, col); vertex(b.x, b.y, -1, -1, col); vertex(c.x, c.y, -1, -1, col);
}

void Hud::rect(float x, float y, float w, float h, const glm::vec4& c) {
    triangle({x, y}, {x + w, y}, {x + w, y + h}, c);
    triangle({x, y}, {x + w, y + h}, {x, y + h}, c);
}

void Hud::line(glm::vec2 a, glm::vec2 b, float thick, const glm::vec4& c) {
    glm::vec2 d = b - a;
    float len = glm::length(d);
    if (len < 1e-4f) return;
    glm::vec2 n = glm::vec2(-d.y, d.x) / len * (thick * 0.5f);
    triangle(a + n, b + n, b - n, c);
    triangle(a + n, b - n, a - n, c);
}

void Hud::arc(float cx, float cy, float r, float a0, float a1, float thick, const glm::vec4& c) {
    int seg = std::max(2, static_cast<int>(std::fabs(a1 - a0) / (2.0f * PI) * 64.0f));
    for (int i = 0; i < seg; ++i)
        line(polar(cx, cy, r, a0 + (a1 - a0) * i / seg), polar(cx, cy, r, a0 + (a1 - a0) * (i + 1) / seg), thick, c);
}

void Hud::circle(float cx, float cy, float r, float thick, const glm::vec4& c) { arc(cx, cy, r, 0.0f, 2.0f * PI, thick, c); }

void Hud::disc(float cx, float cy, float r, const glm::vec4& c) {
    const int seg = 32;
    for (int i = 0; i < seg; ++i) {
        float a0 = 2.0f * PI * i / seg, a1 = 2.0f * PI * (i + 1) / seg;
        triangle({cx, cy}, {cx + r * std::cos(a0), cy + r * std::sin(a0)}, {cx + r * std::cos(a1), cy + r * std::sin(a1)}, c);
    }
}

void Hud::polygon(const std::vector<glm::vec2>& p, const glm::vec4& c) {
    for (size_t i = 1; i + 1 < p.size(); ++i) triangle(p[0], p[i], p[i + 1], c);
}

void Hud::text(float x, float y, const std::string& str, const glm::vec4& c, float scale) {
    float cw = CELL_W * scale * s_, ch = CELL_H * scale * s_, adv = ADVANCE * scale * s_;
    for (int code : decodeUtf8(str)) {
        int i = code - 32;
        if (i != 0) {
            float u0 = (i % 16) * CELL_W / ATLAS_W, v0 = (i / 16) * CELL_H / ATLAS_H;
            float u1 = u0 + CELL_W / ATLAS_W, v1 = v0 + CELL_H / ATLAS_H;
            vertex(x, y, u0, v0, c); vertex(x + cw, y, u1, v0, c); vertex(x + cw, y + ch, u1, v1, c);
            vertex(x, y, u0, v0, c); vertex(x + cw, y + ch, u1, v1, c); vertex(x, y + ch, u0, v1, c);
        }
        x += adv;
    }
}

float Hud::textWidth(const std::string& str, float scale) const {
    return static_cast<float>(decodeUtf8(str).size()) * ADVANCE * scale * s_;
}

void Hud::centeredText(float cx, float y, const std::string& str, const glm::vec4& c, float scale) {
    text(cx - textWidth(str, scale) * 0.5f, y, str, c, scale);
}

void Hud::panel(float x, float y, float w, float h) {
    rect(x, y, w, h, glm::vec4(0.0f, 0.02f, 0.05f, 0.55f));
    glm::vec4 border(1.0f, 1.0f, 1.0f, 0.25f);
    line({x, y}, {x + w, y}, 1.5f, border); line({x, y + h}, {x + w, y + h}, 1.5f, border);
    line({x, y}, {x, y + h}, 1.5f, border); line({x + w, y}, {x + w, y + h}, 1.5f, border);
}

// ------------------------------------------------------------------ main entry
void Hud::draw(const Aircraft& ac, const Camera& cam, const HudInfo& info, int width, int height) {
    w_ = width;
    h_ = height;
    s_ = height / 900.0f;
    ab_ = info.abbrev;
    verts_.clear();

    if (info.showHud) {
        if (cam.mode == CameraMode::Cockpit) {
            cockpitHud(ac, cam);      // green head-up display in the windscreen
            cockpitPanel(ac);
        }
        statusPanel(ac, cam, info);
        flightDataPanel(ac);
        if (info.showApproach) approachPanel(ac);
    }
    if (info.showHelp) helpOverlay();
    else messageLine(ac);

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    shader_.use();
    shader_.set("uOrtho", glm::ortho(0.0f, float(width), float(height), 0.0f));
    shader_.set("uFont", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, fontTex_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts_.size() * sizeof(float)), verts_.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(verts_.size() / 8));
    glBindVertexArray(0);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

// ------------------------------------------------------------------ panels
void Hud::statusPanel(const Aircraft& ac, const Camera& cam, const HudInfo& info) {
    float x = 14 * s_, y = 14 * s_, lh = 22 * s_, sc = 0.6f;
    panel(x, y, 400 * s_, 150 * s_);
    x += 12 * s_; y += 8 * s_;
    text(x, y, "SIMULADOR DE ATERRAGEM - PISTA 27", YELLOW, sc); y += lh;
    bool autoMode = ac.mode == FlightMode::Auto;
    text(x, y, ab_ ? std::string("MODE ") + (autoMode ? "AUTOLAND" : "MANUAL")
                   : std::string("Modo: ") + (autoMode ? "aterragem automática" : "pilotagem manual"),
         autoMode ? CYAN : MAGENTA, sc); y += lh;
    text(x, y, std::string(L("Fase do voo: ", "PHASE ")) + ac.phaseName(), ac.phase == Phase::Crashed ? RED : WHITE, sc); y += lh;
    text(x, y, std::string(L("Câmara: ", "CAM ")) + cam.name(), WHITE, sc); y += lh;
    text(x, y, info.paused ? std::string("SIMULAÇÃO EM PAUSA") : fmt(L("Velocidade da simulação: x%.2g", "SIM x%.2g"), info.timeScale),
         info.paused ? YELLOW : WHITE, sc); y += lh;
    text(x, y, fmt("FPS %.0f", info.fps), GREY, sc);
    text(x + 120 * s_, y, L("N = siglas", "N = nomes"), GREY, sc);
    text(x + 270 * s_, y, "H = ajuda", GREY, sc);
}

void Hud::flightDataPanel(const Aircraft& ac) {
    float x = 14 * s_, y = 176 * s_, lh = 24 * s_, sc = 0.62f, vx = 245 * s_;
    panel(x, y, 400 * s_, 11 * lh + 16 * s_);
    x += 12 * s_; y += 8 * s_;
    auto row = [&](const char* label, const std::string& value, const glm::vec4& c) {
        text(x, y, label, GREY, sc);
        text(x + vx, y, value, c, sc);
        y += lh;
    };
    float kmh = ac.airspeed() * KMH, vs = ac.verticalSpeed();
    row(L("Velocidade", "IAS"), fmt("%.0f km/h", kmh), (kmh < 85.0f && !ac.onGround) ? RED : GREEN);
    row(L("Altura acima do solo", "ALT AGL"), fmt("%.0f m", ac.agl()), GREEN);
    row(L("Velocidade vertical", "VS"), fmt("%+.1f m/s", vs), (vs < -5.0f) ? RED : GREEN);
    row(L("Rumo (bússola)", "HDG"), fmt("%03.0f°", std::round(ac.headingDeg()) >= 360.0f ? 0.0 : std::round(ac.headingDeg())), GREEN);
    row(L("Inclinação do nariz", "PITCH"), fmt("%+.1f°", glm::degrees(ac.pitch)), GREEN);
    row(L("Inclinação das asas", "BANK"), fmt("%+.0f°", glm::degrees(ac.roll)), std::fabs(ac.roll) > glm::radians(90.0f) ? YELLOW : GREEN);
    row(L("Ângulo de ataque", "AOA"), fmt("%+.1f°", glm::degrees(ac.angleOfAttack())), GREEN);
    row(L("Potência do motor", "THR"), fmt("%.0f %%", ac.throttle * 100.0f), WHITE);
    row(L("Rotações da hélice", "RPM"), fmt("%.0f rpm", ac.propRpm), WHITE);
    row(L("Flaps", "FLAPS"), fmt("%.0f°", ac.flaps * 10.0f), WHITE);
    row(L("Travões", "BRK"), ac.brakesOn ? L("ligados", "ON") : L("desligados", "OFF"), ac.brakesOn ? YELLOW : GREY);
}

void Hud::approachPanel(const Aircraft& ac) {
    float pw = 440 * s_, ph = 318 * s_;
    float x = w_ - pw - 14 * s_, y = 14 * s_, sc = 0.58f, lh = 21 * s_;
    panel(x, y, pw, ph);
    float tx = x + 12 * s_, ty = y + 8 * s_;
    text(tx, ty, L("GUIA DE APROXIMAÇÃO - PISTA 27", "ILS RWY 27"), YELLOW, sc); ty += lh;
    text(tx, ty, L("Rampa de descida de 3,0° (sistema ILS)", "GS 3,0°"), WHITE, sc); ty += lh;
    // distance to the threshold = the line where the landing part of the runway begins
    float d = ac.distanceToThreshold();
    if (d > 0.0f) text(tx, ty, d >= 1000.0f ? fmt(L("Distância ao início da pista: %.2f km", "THR %.2f km"), d / 1000.0)
                                           : fmt(L("Distância ao início da pista: %.0f m", "THR %.0f m"), d), WHITE, sc);
    else text(tx, ty, fmt(L("Já passou o início da pista há %.0f m", "PAST THR %.0f m"), -d), WHITE, sc);
    ty += lh;

    // glide slope (vertical) and localizer (horizontal) deviation scales
    float cx = x + pw * 0.42f, cy = ty + 82 * s_, dot = 22 * s_;
    for (int i = -2; i <= 2; ++i) {
        if (i == 0) continue;
        circle(cx + 110 * s_, cy + i * dot, 4 * s_, 1.5f, WHITE);    // glide slope dots
        circle(cx + i * dot, cy + 80 * s_, 4 * s_, 1.5f, WHITE);     // localizer dots
    }
    line({cx + 98 * s_, cy}, {cx + 122 * s_, cy}, 2.0f, YELLOW);
    line({cx, cy + 68 * s_}, {cx, cy + 92 * s_}, 2.0f, YELLOW);
    if (ab_) {
        text(cx + 132 * s_, cy - 2 * dot, "GS", GREY, 0.48f);
        text(cx + 2.9f * dot, cy + 72 * s_, "LOC", GREY, 0.48f);
    } else {
        text(cx + 132 * s_, cy - 2 * dot, "rampa", GREY, 0.48f);
        text(cx + 132 * s_, cy - 2 * dot + 15 * s_, "de descida", GREY, 0.48f);
        text(cx + 132 * s_, cy + 1.2f * dot, "(acima:", GREY, 0.42f);
        text(cx + 132 * s_, cy + 1.2f * dot + 13 * s_, "descer)", GREY, 0.42f);
        text(cx + 2.9f * dot, cy + 72 * s_, "eixo da pista", GREY, 0.48f);
    }
    // diamonds: where the glide slope / centre line is (aircraft above -> diamond below,
    // aircraft left of the centre line -> diamond to the right)
    auto diamond = [&](float px, float py) {
        polygon({{px, py - 8 * s_}, {px + 8 * s_, py}, {px, py + 8 * s_}, {px - 8 * s_, py}}, MAGENTA);
    };
    if ((Airport::AIM_POINT_Z - ac.pos.z) > 10.0f && !ac.onGround) {
        diamond(cx + 110 * s_, cy + glm::clamp(ac.glideslopeDeviationDots(), -2.5f, 2.5f) * dot);
        diamond(cx + glm::clamp(ac.localizerDeviationDots(), -2.5f, 2.5f) * dot, cy + 80 * s_);
    } else {
        text(cx - 50 * s_, cy - 30 * s_, L("sem sinal", "NO GS"), RED, 0.55f);
    }
    // aircraft symbol at the centre
    line({cx - 30 * s_, cy}, {cx - 8 * s_, cy}, 3.0f, YELLOW);
    line({cx + 8 * s_, cy}, {cx + 30 * s_, cy}, 3.0f, YELLOW);

    // PAPI (as seen from the aircraft), shown left (outer) to right (inner)
    ty = cy + 110 * s_;
    text(tx, ty, L("Luzes indicadoras de rampa (PAPI)", "PAPI"), GREY, sc); ty += lh;
    int whites = World::papiWhiteCount(ac.pos);
    for (int i = 0; i < 4; ++i)
        disc(tx + 12 * s_ + i * 26 * s_, ty + 10 * s_, 8 * s_, whites < 0 ? GREY : (i < whites ? WHITE : RED));
    const char* papiTxt = whites < 0 ? "" : whites == 2 ? L("na rampa certa", "ON SLOPE") : whites < 2 ? L("demasiado baixo", "LOW") : L("demasiado alto", "HIGH");
    text(tx + 122 * s_, ty, papiTxt, whites == 2 ? GREEN : YELLOW, sc);
}

void Hud::messageLine(const Aircraft& ac) {
    if (ac.messageTime <= 0.0f || ac.message.empty()) return;
    float sc = 0.8f;
    std::string msg = decimalComma(ac.message);
    float tw = textWidth(msg, sc);
    float x = (w_ - tw) * 0.5f, y = h_ * 0.62f;
    auto has = [&](const char* word) { return msg.find(word) != std::string::npos; };
    glm::vec4 c = WHITE;
    if (has("ACIDENTE") || has("PERDA") || has("FORA DA PISTA") || has("CAUDA")) c = RED;
    else if (has("TOQUE") || has("Parado")) c = GREEN;
    rect(x - 14 * s_, y - 6 * s_, tw + 28 * s_, 38 * s_, glm::vec4(0.0f, 0.0f, 0.0f, 0.55f));
    text(x, y, msg, c, sc);
}

const std::vector<std::string> HELP_LINES = {
    "COMANDOS",
    "",
    "1/2/3/4 ou C       câmara: seguir o avião / cockpit / livre / torre",
    "Arrastar o rato    rodar a câmara (no cockpit: olhar à volta)",
    "Espaço             no cockpit: voltar a olhar em frente",
    "Roda do rato       zoom / campo de visão",
    "",
    "M                  aterragem automática <-> pilotagem manual",
    "Seta para baixo    puxar o manche: o nariz sobe",
    "Seta para cima     empurrar o manche: o nariz desce",
    "Setas esq. / dir.  rodar o avião sobre si próprio (360°)",
    "Z / X              leme de direção: virar à esquerda / direita",
    "T / G              aumentar / reduzir a potência do motor",
    "F / V              baixar / recolher os flaps",
    "B (manter)         travões das rodas",
    "",
    "I / K              rodar o sol num sentido / no outro (dia e noite)",
    "Tab                esconder / mostrar todos os painéis",
    "N                  siglas de aviação / nomes por extenso",
    "O                  esconder / mostrar o guia de aproximação",
    "+ / -              velocidade da simulação     P    pausa",
    "R                  recomeçar a aproximação",
    "F12                captura de ecrã (.ppm)      Esc  sair",
};

void Hud::helpOverlay() {
    float sc = 0.62f, lh = 24 * s_;
    float pw = 720 * s_, ph = HELP_LINES.size() * lh + 30 * s_;
    float x = (w_ - pw) * 0.5f, y = (h_ - ph) * 0.5f;
    rect(x, y, pw, ph, glm::vec4(0.0f, 0.0f, 0.0f, 0.82f));
    float ty = y + 15 * s_;
    for (size_t i = 0; i < HELP_LINES.size(); ++i) {
        text(x + 24 * s_, ty, HELP_LINES[i], i == 0 ? YELLOW : WHITE, sc);
        ty += lh;
    }
}

// ------------------------------------------------------------------ cockpit
// Head-up display projected in the windscreen: pitch ladder, horizon, glide path and
// flight path vector (where the aircraft is really going). It is fixed to the aircraft, so
// when the pilot looks around it moves on the screen and disappears when it leaves the view.
void Hud::cockpitHud(const Aircraft& ac, const Camera& cam) {
    const glm::vec4 hudGreen(0.3f, 1.0f, 0.4f, 0.9f);
    float ppr = (h_ * 0.5f) / std::tan(glm::radians(cam.fovDeg) * 0.5f);   // pixels per radian (small angles)
    if (std::fabs(cam.headYaw) > glm::radians(70.0f) || std::fabs(cam.headPitch) > glm::radians(60.0f)) return;
    float phi = ac.roll, theta = ac.pitch;
    glm::vec2 B(w_ * 0.5f + cam.headYaw * ppr, h_ * 0.5f + cam.headPitch * ppr);  // boresight (body nose axis)
    glm::vec2 d(std::cos(phi), -std::sin(phi));                    // horizon direction on screen
    glm::vec2 n(-std::sin(phi), -std::cos(phi));                   // "sky" direction on screen
    const float top = 8.0f * s_, bottom = h_ * 0.74f - 10.0f * s_;  // visible part of the windscreen
    auto inside = [&](glm::vec2 p) { return p.y > top && p.y < bottom && p.x > 0.0f && p.x < w_; };
    auto segment = [&](glm::vec2 a, glm::vec2 b, const glm::vec4& c) {
        if (inside(a) || inside(b)) line(a, b, 2.0f * s_, c);
    };

    // boresight symbol
    segment({B.x - 14 * s_, B.y}, {B.x - 5 * s_, B.y}, hudGreen);
    segment({B.x + 5 * s_, B.y}, {B.x + 14 * s_, B.y}, hudGreen);
    segment({B.x, B.y + 5 * s_}, {B.x, B.y + 12 * s_}, hudGreen);

    // pitch ladder every 5 degrees (only the steps near the current pitch)
    int k0 = static_cast<int>(std::floor(glm::degrees(theta) / 5.0f)) * 5;
    for (int k = std::max(k0 - 25, -90); k <= std::min(k0 + 30, 90); k += 5) {
        glm::vec2 c = B - n * ((theta - glm::radians(float(k))) * ppr);
        if (!inside(c)) continue;
        float half = (k == 0 ? 260.0f : 70.0f) * s_;
        if (k == 0) {
            segment(c - d * half, c + d * half, hudGreen);
            continue;
        }
        float gap = 22 * s_;
        glm::vec4 col = hudGreen;
        if (k < 0) col.a = 0.6f;
        segment(c - d * half, c - d * gap, col);
        segment(c + d * gap, c + d * half, col);
        glm::vec2 b2 = c + d * half;
        text(b2.x + 6 * s_, b2.y - 9 * s_, fmt("%.0f", std::fabs(double(k))), col, 0.5f);
    }
    // -3 degree glide path reference (dashed)
    glm::vec2 gp = B - n * ((theta + glm::radians(Airport::GLIDE_SLOPE_DEG)) * ppr);
    for (int i = -3; i <= 3; ++i) {
        if (i == 0) continue;
        glm::vec2 a = gp + d * (i * 22.0f * s_), b = gp + d * (i * 22.0f * s_ + (i > 0 ? 12.0f : -12.0f) * s_);
        segment(a, b, glm::vec4(1.0f, 0.5f, 1.0f, 0.85f));
    }
    // flight path vector
    if (ac.airspeed() > 5.0f && std::fabs(phi) < glm::radians(90.0f)) {
        float gamma = std::asin(glm::clamp(ac.vel.y / ac.airspeed(), -1.0f, 1.0f));
        glm::vec2 f = B - n * ((theta - gamma) * ppr);
        if (inside(f)) {
            circle(f.x, f.y, 7 * s_, 2.0f * s_, hudGreen);
            segment({f.x - 20 * s_, f.y}, {f.x - 7 * s_, f.y}, hudGreen);
            segment({f.x + 7 * s_, f.y}, {f.x + 20 * s_, f.y}, hudGreen);
            segment({f.x, f.y - 7 * s_}, {f.x, f.y - 15 * s_}, hudGreen);
        }
    }
    // speed and height (numbers)
    if (inside({B.x - 280 * s_, B.y})) {
        text(B.x - 300 * s_, B.y - 12 * s_, fmt("%3.0f", ac.airspeed() * KMH), hudGreen, 0.8f);
        text(B.x - 300 * s_, B.y + 14 * s_, "km/h", hudGreen, 0.5f);
    }
    if (inside({B.x + 260 * s_, B.y})) {
        text(B.x + 220 * s_, B.y - 12 * s_, fmt("%4.0f", ac.agl()), hudGreen, 0.8f);
        text(B.x + 250 * s_, B.y + 14 * s_, "m", hudGreen, 0.5f);
    }
}

void Hud::cockpitPanel(const Aircraft& ac) {
    float top = h_ * 0.74f;
    rect(0.0f, top, float(w_), h_ - top, glm::vec4(0.09f, 0.09f, 0.10f, 1.0f));      // instrument panel
    rect(0.0f, top - 6 * s_, float(w_), 6 * s_, glm::vec4(0.03f, 0.03f, 0.03f, 1.0f)); // glare shield edge
    float r = std::min(h_ * 0.085f, w_ * 0.062f);
    float cy = top + (h_ - top) * 0.53f;
    const float xs[5] = {0.27f, 0.39f, 0.51f, 0.63f, 0.75f};
    const char* names[5] = {L("Velocidade", "IAS"), L("Horizonte", "ATT"), L("Altímetro", "ALT"),
                            L("Subida/descida", "VSI"), L("Bússola", "HDG")};
    for (int i = 0; i < 5; ++i) centeredText(w_ * xs[i], cy - r - 22 * s_, names[i], GREY, 0.45f);
    gaugeAirspeed(w_ * xs[0], cy, r, ac.airspeed() * KMH);
    gaugeAttitude(w_ * xs[1], cy, r, glm::degrees(ac.pitch), glm::degrees(ac.roll));
    gaugeAltimeter(w_ * xs[2], cy, r, ac.agl());
    gaugeVSI(w_ * xs[3], cy, r, ac.verticalSpeed());
    gaugeHeading(w_ * xs[4], cy, r, ac.headingDeg());
    // engine / configuration column
    float tx = w_ * 0.025f, ty = top + 16 * s_, lh = 23 * s_;
    text(tx, ty, fmt(L("Potência: %3.0f %%", "THR %3.0f %%"), ac.throttle * 100.0f), WHITE, 0.55f); ty += lh;
    rect(tx, ty, 150 * s_, 10 * s_, glm::vec4(0.25f, 0.25f, 0.25f, 1.0f));
    rect(tx, ty, 150 * s_ * ac.throttle, 10 * s_, GREEN); ty += lh * 0.8f;
    text(tx, ty, fmt(L("Flaps: %.0f°", "FLAPS %.0f"), ac.flaps * 10.0f), WHITE, 0.55f); ty += lh;
    text(tx, ty, ac.brakesOn ? L("Travões ligados", "BRK ON") : L("Travões desligados", "BRK OFF"), ac.brakesOn ? YELLOW : GREY, 0.55f); ty += lh;
    text(tx, ty, ac.onGround ? L("Rodas no solo", "WOW GND") : L("Rodas no ar", "WOW AIR"), GREY, 0.55f);
    float rx = w_ * 0.835f;
    bool autoMode = ac.mode == FlightMode::Auto;
    text(rx, top + 16 * s_, L("Piloto automático:", "AP"), GREY, 0.55f);
    text(rx, top + 39 * s_, autoMode ? L("a aterrar", "LAND") : L("desligado", "OFF"), autoMode ? CYAN : GREY, 0.55f);
    text(rx, top + 70 * s_, ac.phaseName(), WHITE, 0.55f);
}

// round dial with a value written below it
void Hud::dial(float cx, float cy, float r, const std::string& value) {
    disc(cx, cy, r, glm::vec4(0.02f, 0.02f, 0.02f, 1.0f));
    circle(cx, cy, r, 3.0f, GREY);
    centeredText(cx, cy + r + 4 * s_, value, WHITE, 0.55f);
}

// a scale mark from radius r*inner to r, and a label centred at radius r (dy pixels up)
void Hud::tick(float cx, float cy, float r, float a, float inner, float thick) {
    line(polar(cx, cy, r * inner, a), polar(cx, cy, r, a), thick, WHITE);
}
void Hud::tickLabel(float cx, float cy, float r, float a, const std::string& l, float scale, float dy, const glm::vec4& c) {
    glm::vec2 p = polar(cx, cy, r, a);
    centeredText(p.x, p.y - dy * s_, l, c, scale);
}

void Hud::gaugeAirspeed(float cx, float cy, float r, float kmh) {
    dial(cx, cy, r, fmt("%.0f km/h", kmh));
    auto ang = [](float v) { return glm::radians(-150.0f + glm::clamp(v, 0.0f, 350.0f) / 350.0f * 300.0f); };
    arc(cx, cy, r * 0.86f, ang(74), ang(157), 4.0f * s_, WHITE);                            // flaps range
    arc(cx, cy, r * 0.92f, ang(89), ang(239), 4.0f * s_, glm::vec4(0.2f, 0.9f, 0.2f, 1.0f)); // normal
    arc(cx, cy, r * 0.92f, ang(239), ang(302), 4.0f * s_, YELLOW);                           // caution
    for (int v = 0; v <= 350; v += 50) {
        tick(cx, cy, r, ang(float(v)), 0.78f, 2.0f);
        if (v % 100 == 0) tickLabel(cx, cy, r * 0.58f, ang(float(v)), fmt("%.0f", v), 0.38f, 7);
    }
    line({cx, cy}, polar(cx, cy, r * 0.85f, ang(kmh)), 3.0f * s_, WHITE);
    centeredText(cx, cy + r * 0.3f, "km/h", GREY, 0.42f);
}

void Hud::gaugeAttitude(float cx, float cy, float r, float pitchDeg, float rollDeg) {
    // circle polygon, then the ground part clipped by the horizon line
    std::vector<glm::vec2> circ;
    for (int i = 0; i < 48; ++i) {
        float a = 2.0f * PI * i / 48;
        circ.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    polygon(circ, glm::vec4(0.15f, 0.45f, 0.85f, 1.0f));
    float phi = glm::radians(rollDeg);
    glm::vec2 C(cx, cy), d(std::cos(phi), -std::sin(phi)), n(-std::sin(phi), -std::cos(phi));
    float pxPerDeg = r / 25.0f;
    glm::vec2 hc = C - n * (pitchDeg * pxPerDeg);
    polygon(clipHalfPlane(circ, hc, -n), glm::vec4(0.55f, 0.35f, 0.15f, 1.0f));
    // horizon line clipped to the circle
    float dist = glm::dot(hc - C, n);
    if (std::fabs(dist) < r) {
        float half = std::sqrt(r * r - dist * dist);
        glm::vec2 m = C + n * dist;
        line(m - d * half, m + d * half, 2.0f, WHITE);
    }
    for (int k = -90; k <= 90; k += 10) {               // pitch marks
        glm::vec2 c = hc + n * (k * pxPerDeg);
        if (k != 0 && glm::length(c - C) <= r * 0.8f) line(c - d * r * 0.2f, c + d * r * 0.2f, 1.5f, WHITE);
    }
    circle(cx, cy, r, 3.0f, GREY);
    // fixed aircraft symbol
    line({cx - r * 0.55f, cy}, {cx - r * 0.15f, cy}, 4.0f * s_, YELLOW);
    line({cx + r * 0.15f, cy}, {cx + r * 0.55f, cy}, 4.0f * s_, YELLOW);
    disc(cx, cy, 3.0f * s_, YELLOW);
    centeredText(cx, cy + r + 4 * s_, fmt("asas %+.0f°", rollDeg), WHITE, 0.5f);
}

// long hand: 100 m per turn, short hand: 1000 m per turn
void Hud::gaugeAltimeter(float cx, float cy, float r, float m) {
    dial(cx, cy, r, fmt("%.0f m", m));
    for (int i = 0; i < 10; ++i) {
        float a = 2.0f * PI * i / 10.0f;
        tick(cx, cy, r, a, 0.78f, 2.0f);
        glm::vec2 p = polar(cx, cy, r * 0.6f, a);
        text(p.x - 5 * s_, p.y - 9 * s_, fmt("%.0f", double(i)), WHITE, 0.45f);
    }
    float a100 = 2.0f * PI * std::fmod(std::fmax(m, 0.0f), 100.0f) / 100.0f;
    float a1000 = 2.0f * PI * std::fmod(std::fmax(m, 0.0f), 1000.0f) / 1000.0f;
    line({cx, cy}, polar(cx, cy, r * 0.5f, a1000), 5.0f * s_, WHITE);
    line({cx, cy}, polar(cx, cy, r * 0.85f, a100), 2.5f * s_, WHITE);
}

// vertical speed indicator: 0 at 9 o'clock, +10 m/s up, -10 m/s down
void Hud::gaugeVSI(float cx, float cy, float r, float ms) {
    dial(cx, cy, r, fmt("%+.1f m/s", ms));
    auto ang = [](float v) { return glm::radians(-90.0f + glm::clamp(v, -10.0f, 10.0f) / 10.0f * 170.0f); };
    for (int v = -10; v <= 10; v += 5) {
        tick(cx, cy, r, ang(float(v)), 0.8f, 2.0f);
        tickLabel(cx, cy, r * 0.6f, ang(float(v)), fmt("%.0f", std::fabs(double(v))), 0.38f, 7);
    }
    line({cx, cy}, polar(cx, cy, r * 0.85f, ang(ms)), 3.0f * s_, WHITE);
    text(cx + r * 0.05f, cy - r * 0.42f, "sobe", GREY, 0.38f);
    text(cx + r * 0.05f, cy + r * 0.28f, "desce", GREY, 0.38f);
}

void Hud::gaugeHeading(float cx, float cy, float r, float hdg) {
    dial(cx, cy, r, fmt("rumo %03.0f°", std::fmod(std::round(hdg), 360.0f)));
    const char* cards[] = {"N", "3", "6", "E", "12", "15", "S", "21", "24", "O", "30", "33"};
    for (int i = 0; i < 36; ++i) {
        float a = glm::radians(i * 10.0f - hdg);          // the card rotates, lubber line on top
        tick(cx, cy, r, a, (i % 3 == 0) ? 0.78f : 0.88f, 1.5f);
        if (i % 3 == 0) {
            std::string l = cards[i / 3];
            tickLabel(cx, cy, r * 0.6f, a, l, 0.45f, 8, l.size() == 1 ? YELLOW : WHITE);
        }
    }
    line({cx, cy - r}, {cx, cy - r * 0.7f}, 3.0f, glm::vec4(1.0f, 0.5f, 0.1f, 1.0f));   // lubber line
    line({cx, cy - r * 0.3f}, {cx, cy + r * 0.3f}, 2.5f, YELLOW);                       // aircraft
    line({cx - r * 0.25f, cy - r * 0.05f}, {cx + r * 0.25f, cy - r * 0.05f}, 2.5f, YELLOW);
}
