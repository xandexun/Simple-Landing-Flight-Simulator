// main.cpp - Aircraft Flight Simulator: LANDING procedure (Computer Graphics, Assignment 2, Exercise 2)
//
// Loads runway.obj and airplane.obj, renders them with Phong shading and a movable point light,
// and simulates the landing on runway 27 (automatic ILS approach or manual flight).
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "sim/aircraft.h"
#include "world/airport.h"
#include "sim/camera.h"
#include "sim/environment.h"
#include "sim/explosion.h"
#include "engine/gl_utils.h"
#include "render/hud.h"
#include "render/renderer.h"

#ifndef ASSET_ROOT
#define ASSET_ROOT "./"
#endif

namespace {

struct App {
    GLFWwindow* window = nullptr;
    Aircraft aircraft;
    Camera camera;
    Environment env;
    World world;
    Explosion explosion;
    Renderer renderer;
    Hud hud;
    HudInfo info;
    bool dragging = false;
    double lastX = 0.0, lastY = 0.0;
    bool screenshotRequested = false;
    int screenshotCount = 0;
};

App* gApp = nullptr;

// ------------------------------------------------------------------ GLFW callbacks
void keyCallback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
    if (action != GLFW_PRESS) return;
    App& a = *gApp;
    switch (key) {
    case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
    case GLFW_KEY_1: a.camera.mode = CameraMode::Follow; break;
    case GLFW_KEY_2:
        if (a.camera.mode == CameraMode::Cockpit) a.camera.centerHead();   // pressed again: look ahead
        a.camera.mode = CameraMode::Cockpit;
        break;
    case GLFW_KEY_SPACE: a.camera.centerHead(); break;
    case GLFW_KEY_3: a.camera.mode = CameraMode::Free; break;
    case GLFW_KEY_4: a.camera.mode = CameraMode::Tower; break;
    case GLFW_KEY_C: a.camera.mode = CameraMode((int(a.camera.mode) + 1) % 4); break;   // next camera
    case GLFW_KEY_M:
        a.aircraft.setMode(a.aircraft.mode == FlightMode::Auto ? FlightMode::Manual : FlightMode::Auto);
        break;
    case GLFW_KEY_R: {
        CameraMode keep = a.camera.mode;
        a.aircraft.reset(a.aircraft.mode);
        a.explosion.clear();
        a.camera.reset();
        a.camera.mode = keep;
        break;
    }
    case GLFW_KEY_P: a.info.paused = !a.info.paused; break;
    case GLFW_KEY_H: a.info.showHelp = !a.info.showHelp; break;
    case GLFW_KEY_TAB: a.info.showHud = !a.info.showHud; break;
    case GLFW_KEY_O: a.info.showApproach = !a.info.showApproach; break;
    case GLFW_KEY_N: a.info.abbrev = !a.info.abbrev; break;
    case GLFW_KEY_F: a.aircraft.changeFlaps(+1); break;
    case GLFW_KEY_V: a.aircraft.changeFlaps(-1); break;
    case GLFW_KEY_EQUAL:
    case GLFW_KEY_KP_ADD: a.info.timeScale = std::fmin(a.info.timeScale * 2.0f, 8.0f); break;
    case GLFW_KEY_MINUS:
    case GLFW_KEY_KP_SUBTRACT: a.info.timeScale = std::fmax(a.info.timeScale * 0.5f, 0.25f); break;
    case GLFW_KEY_F12: a.screenshotRequested = true; break;
    default: break;
    }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int /*mods*/) {
    if (button != GLFW_MOUSE_BUTTON_LEFT && button != GLFW_MOUSE_BUTTON_RIGHT) return;
    gApp->dragging = (action == GLFW_PRESS);
    glfwGetCursorPos(window, &gApp->lastX, &gApp->lastY);
}

void cursorCallback(GLFWwindow* /*window*/, double x, double y) {
    if (gApp->dragging) gApp->camera.mouseDrag(float(x - gApp->lastX), float(y - gApp->lastY));
    gApp->lastX = x;
    gApp->lastY = y;
}

void scrollCallback(GLFWwindow* /*window*/, double /*dx*/, double dy) {
    gApp->camera.scroll(float(dy));
}

// ------------------------------------------------------------------ continuous keyboard input
bool down(GLFWwindow* w, int key) { return glfwGetKey(w, key) == GLFW_PRESS; }
// +1 while the "plus" key is held, -1 for the "minus" key, 0 for none or both
float axis(GLFWwindow* w, int plus, int minus) { return (down(w, plus) ? 1.0f : 0.0f) - (down(w, minus) ? 1.0f : 0.0f); }

Controls processInput(App& a, float dt) {
    GLFWwindow* w = a.window;
    Controls c;   // flight controls (used in manual mode)
    c.elevator = axis(w, GLFW_KEY_DOWN, GLFW_KEY_UP);
    c.aileron  = axis(w, GLFW_KEY_RIGHT, GLFW_KEY_LEFT);
    c.rudder   = axis(w, GLFW_KEY_X, GLFW_KEY_Z);
    c.brakes   = down(w, GLFW_KEY_B);
    a.aircraft.changeThrottle(axis(w, GLFW_KEY_T, GLFW_KEY_G) * 0.4f * dt);

    // light (sun): turns around the scene, I one way and K the other (45 degrees per second)
    float sun = axis(w, GLFW_KEY_I, GLFW_KEY_K);
    if (sun != 0.0f) a.env.rotateSun(sun * glm::radians(45.0f) * dt);
    return c;
}

struct Options {
    int msaa = -1;             // -1 = automatic (4x on a real GPU, off on software rendering), 0 = off, 1 = on
    int width = 1600, height = 900;
    std::string root;          // optional path of the project folder
    bool hasCam = false;       // --cam x,y,z,yawDeg,pitchDeg : start with the free camera there
    float cam[5] = {0, 0, 0, 0, 0};
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--no-msaa") o.msaa = 0;
        else if (a == "--msaa") o.msaa = 1;
        else if (a == "--size" && i + 1 < argc) std::sscanf(argv[++i], "%dx%d", &o.width, &o.height);
        else if (a == "--cam" && i + 1 < argc)
            o.hasCam = std::sscanf(argv[++i], "%f,%f,%f,%f,%f", &o.cam[0], &o.cam[1], &o.cam[2], &o.cam[3], &o.cam[4]) == 5;
        else if (a == "--help" || a == "-h")
            std::cout << "usage: flightsim [project_folder] [--size WxH] [--msaa | --no-msaa] [--cam x,y,z,yaw,pitch]\n";
        else if (a.rfind("--", 0) != 0) o.root = a;
    }
    return o;
}

bool isSoftwareRenderer(const char* renderer) {
    std::string r = renderer ? renderer : "";
    for (char& c : r) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return r.find("llvmpipe") != std::string::npos || r.find("softpipe") != std::string::npos ||
           r.find("swrast") != std::string::npos || r.find("software") != std::string::npos;
}

std::string findRoot(const std::string& given) {
    std::string candidates[] = {given.empty() ? std::string() : given + "/", ASSET_ROOT, "./", "../", "../../"};
    for (const std::string& r : candidates)
        if (!r.empty() && std::ifstream(r + "assets/runway.obj").good()) return r;
    return "./";
}

void printHelp() {
    std::cout << "\n";
    for (const std::string& line : HELP_LINES) std::cout << "  " << line << "\n";
    std::cout << "\n";
}

int fail(const char* message) {
    std::cerr << message << "\n";
    glfwTerminate();
    return -1;
}

} // namespace

int main(int argc, char** argv) {
    App app;
    gApp = &app;
    const Options opts = parseArgs(argc, argv);
    const std::string root = findRoot(opts.root);

    // GLEW (GLX build) does not work on a pure Wayland surface: prefer X11/XWayland if available
#ifdef GLFW_PLATFORM_X11
    if (glfwPlatformSupported(GLFW_PLATFORM_X11)) glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
    if (!glfwInit()) return fail("Failed to initialise GLFW");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    auto openWindow = [&](int samples) -> GLFWwindow* {
        glfwWindowHint(GLFW_SAMPLES, samples);
        GLFWwindow* w = glfwCreateWindow(opts.width, opts.height, "Flight Simulator - Landing (Exercise 2)", nullptr, nullptr);
        if (w) {
            glfwMakeContextCurrent(w);
            glfwSwapInterval(1);
        }
        return w;
    };
    auto initGlew = [&]() -> bool {
        glewExperimental = GL_TRUE;
        GLenum err = glewInit();
        glGetError();   // glewInit may leave a harmless GL_INVALID_ENUM in core profile
        if (err != GLEW_OK) {
            std::cerr << "glewInit warning: " << glewGetErrorString(err) << "\n";
            if (!glGenVertexArrays) { std::cerr << "OpenGL functions not available\n"; return false; }
        }
        return true;
    };

    app.window = openWindow(opts.msaa == 0 ? 0 : 4);
    if (!app.window) return fail("Failed to open a window with OpenGL 3.3 core");
    if (!initGlew()) return -1;

    // Software rendering (virtual machine, no GPU driver) is very slow with multisampling:
    // reopen the window without MSAA unless the user asked for it explicitly.
    if (isSoftwareRenderer(reinterpret_cast<const char*>(glGetString(GL_RENDERER)))) {
        std::cout << "WARNING: software OpenGL renderer detected (no GPU acceleration) - the simulator will be slow.\n"
                     "         Install the graphics driver / enable 3D acceleration in the virtual machine.\n";
        if (opts.msaa == -1) {
            glfwDestroyWindow(app.window);
            app.window = openWindow(0);
            if (!app.window || !initGlew()) return -1;
            std::cout << "MSAA disabled to save performance (use --msaa to force it)\n";
        }
    }
    std::cout << "OpenGL " << glGetString(GL_VERSION) << " - " << glGetString(GL_RENDERER) << "\n";
    std::cout << "Assets: " << root << "\n";

    glfwSetKeyCallback(app.window, keyCallback);
    glfwSetMouseButtonCallback(app.window, mouseButtonCallback);
    glfwSetCursorPosCallback(app.window, cursorCallback);
    glfwSetScrollCallback(app.window, scrollCallback);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    app.env.reset();
    app.world.build();
    if (!app.renderer.init(root, app.world) || !app.hud.init(root))
        return fail("Initialisation failed (are the assets/ and shaders/ folders next to the program?)");
    app.aircraft.reset(FlightMode::Auto);
    app.camera.reset();
    if (opts.hasCam) {                       // free camera at a given place (degrees; yaw 0 = looking along +Z)
        app.camera.mode = CameraMode::Free;
        app.camera.freePos = glm::vec3(opts.cam[0], opts.cam[1], opts.cam[2]);
        app.camera.freeYaw = glm::radians(opts.cam[3]);
        app.camera.freePitch = glm::radians(opts.cam[4]);
    }
    printHelp();

    double last = glfwGetTime();
    float fpsAccum = 0.0f;
    int fpsFrames = 0;
    while (!glfwWindowShouldClose(app.window)) {
        double now = glfwGetTime();
        float dt = static_cast<float>(std::fmin(now - last, 0.1));
        last = now;
        fpsAccum += dt;
        ++fpsFrames;
        if (fpsAccum >= 0.5f) { app.info.fps = fpsFrames / fpsAccum; fpsAccum = 0.0f; fpsFrames = 0; }

        Controls controls = processInput(app, dt);

        // physics with fixed small steps (stable at any frame rate / simulation speed)
        if (!app.info.paused) {
            float simDt = dt * app.info.timeScale;
            int steps = static_cast<int>(std::ceil(simDt / (1.0f / 120.0f)));
            for (int i = 0; i < steps; ++i) app.aircraft.update(simDt / steps, controls);
            // flying into a building (or a tree) destroys the aircraft
            if (app.aircraft.phase != Phase::Crashed && !app.aircraft.onGround &&
                app.world.hitsBuilding(app.aircraft.pos + app.aircraft.up() * 1.2f, 3.0f))
                app.aircraft.crash("bateu num edifício");
            if (app.aircraft.phase == Phase::Crashed && !app.explosion.started) app.explosion.start(app.aircraft.pos);
            app.explosion.update(simDt);
        }
        app.camera.update(dt, app.aircraft);

        int w, h;
        glfwGetFramebufferSize(app.window, &w, &h);
        if (w > 0 && h > 0) {
            app.renderer.render(app.camera, app.env, app.aircraft, app.world, app.explosion, w, h, static_cast<float>(now));
            app.hud.draw(app.aircraft, app.camera, app.info, w, h);
            if (app.screenshotRequested) {
                char name[64];
                std::snprintf(name, sizeof name, "screenshot_%03d.ppm", app.screenshotCount++);
                if (saveScreenshot(name, w, h)) std::cout << "Saved " << name << "\n";
                app.screenshotRequested = false;
            }
        }
        glfwSwapBuffers(app.window);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}
