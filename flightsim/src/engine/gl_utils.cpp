// gl_utils.cpp
#include "engine/gl_utils.h"
#include <cstdio>
#include <cctype>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iostream>

std::string readTextFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "Cannot open file: " << path << "\n";
        return "";
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static GLuint compileShader(GLenum type, const std::string& src, const std::string& name) {
    GLuint s = glCreateShader(type);
    const char* c = src.c_str();
    glShaderSource(s, 1, &c, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        std::cerr << "Shader compile error in " << name << ":\n" << log << "\n";
    }
    return s;
}

bool Shader::load(const std::string& vertPath, const std::string& fragPath) {
    std::string vs = readTextFile(vertPath), fs = readTextFile(fragPath);
    if (vs.empty() || fs.empty()) return false;
    GLuint v = compileShader(GL_VERTEX_SHADER, vs, vertPath);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fs, fragPath);
    id = glCreateProgram();
    glAttachShader(id, v);
    glAttachShader(id, f);
    glLinkProgram(id);
    GLint ok = 0;
    glGetProgramiv(id, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[4096];
        glGetProgramInfoLog(id, sizeof log, nullptr, log);
        std::cerr << "Program link error (" << vertPath << "):\n" << log << "\n";
    }
    glDeleteShader(v);
    glDeleteShader(f);
    cache_.clear();
    return ok != 0;
}

GLint Shader::loc(const char* name) const {
    auto it = cache_.find(name);
    if (it == cache_.end()) it = cache_.emplace(name, glGetUniformLocation(id, name)).first;
    return it->second;
}

void vertexAttribs(std::initializer_list<int> sizes) {
    GLsizei stride = 0;
    for (int n : sizes) stride += n * sizeof(float);
    size_t offset = 0;
    GLuint index = 0;
    for (int n : sizes) {
        glEnableVertexAttribArray(index);
        glVertexAttribPointer(index++, n, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(offset));
        offset += n * sizeof(float);
    }
}

// ---------------------------------------------------------------- images
// next word of a PNM header (skipping spaces and # comments)
static bool readToken(std::istream& in, std::string& tok) {
    tok.clear();
    char c;
    while (in.get(c)) {
        if (c == '#') { std::string line; std::getline(in, line); continue; }
        if (std::isspace(static_cast<unsigned char>(c))) { if (!tok.empty()) return true; continue; }
        tok += c;
    }
    return !tok.empty();
}

bool loadPNM(const std::string& path, Image& img) {
    std::ifstream in(path, std::ios::binary);
    if (!in) { std::cerr << "Cannot open image: " << path << "\n"; return false; }
    std::string magic, w, h, maxv;
    if (!readToken(in, magic) || !readToken(in, w) || !readToken(in, h) || !readToken(in, maxv)) return false;
    if (magic != "P6" && magic != "P5") { std::cerr << path << ": only binary PPM/PGM supported\n"; return false; }
    img.channels = (magic == "P6") ? 3 : 1;
    img.width = std::stoi(w);
    img.height = std::stoi(h);
    img.pixels.resize(static_cast<size_t>(img.width) * img.height * img.channels);
    in.read(reinterpret_cast<char*>(img.pixels.data()), static_cast<std::streamsize>(img.pixels.size()));
    return static_cast<size_t>(in.gcount()) == img.pixels.size();
}

GLuint createTexture(const Image& img, bool mipmaps, bool flipY) {
    std::vector<unsigned char> data = img.pixels;
    const size_t row = static_cast<size_t>(img.width) * img.channels;
    if (flipY)
        for (int y = 0; y < img.height / 2; ++y)
            std::swap_ranges(data.begin() + y * row, data.begin() + (y + 1) * row, data.begin() + (img.height - 1 - y) * row);

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    bool rgb = img.channels == 3;
    glTexImage2D(GL_TEXTURE_2D, 0, rgb ? GL_RGB8 : GL_R8, img.width, img.height, 0, rgb ? GL_RGB : GL_RED, GL_UNSIGNED_BYTE, data.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, mipmaps ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    if (mipmaps) glGenerateMipmap(GL_TEXTURE_2D);
    return tex;
}

bool saveScreenshot(const std::string& path, int width, int height) {
    std::vector<unsigned char> px(static_cast<size_t>(width) * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", width, height);
    for (int y = height - 1; y >= 0; --y)                     // OpenGL rows start at the bottom
        std::fwrite(&px[static_cast<size_t>(y) * width * 3], 1, static_cast<size_t>(width) * 3, f);
    std::fclose(f);
    return true;
}
