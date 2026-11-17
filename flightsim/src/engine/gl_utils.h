// gl_utils.h - shaders, vertex attributes, textures (PPM/PGM loader) and screenshots
#pragma once
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <initializer_list>
#include <string>
#include <vector>
#include <unordered_map>

std::string readTextFile(const std::string& path);

class Shader {
public:
    GLuint id = 0;
    bool load(const std::string& vertPath, const std::string& fragPath);
    void use() const { glUseProgram(id); }
    GLint loc(const char* name) const;
    void set(const char* name, int v) const              { glUniform1i(loc(name), v); }
    void set(const char* name, bool v) const             { glUniform1i(loc(name), v ? 1 : 0); }
    void set(const char* name, float v) const            { glUniform1f(loc(name), v); }
    void set(const char* name, const glm::vec3& v) const { glUniform3f(loc(name), v.x, v.y, v.z); }
    void set(const char* name, const glm::mat3& m) const { glUniformMatrix3fv(loc(name), 1, GL_FALSE, glm::value_ptr(m)); }
    void set(const char* name, const glm::mat4& m) const { glUniformMatrix4fv(loc(name), 1, GL_FALSE, glm::value_ptr(m)); }
private:
    mutable std::unordered_map<std::string, GLint> cache_;
};

// Describes the vertex buffer bound to GL_ARRAY_BUFFER: consecutive float attributes 0, 1, 2...
// with the given number of components each, e.g. {3, 3, 2} = position, normal, uv.
void vertexAttribs(std::initializer_list<int> sizes);

struct Image {
    int width = 0, height = 0, channels = 0;   // channels: 1 (PGM) or 3 (PPM)
    std::vector<unsigned char> pixels;          // first row = top of the image
};

// Loads binary PPM (P6, RGB) or PGM (P5, grey) images
bool loadPNM(const std::string& path, Image& img);
// Creates an OpenGL texture. flipY = true puts the first row at v = 1 (OBJ convention).
GLuint createTexture(const Image& img, bool mipmaps, bool flipY);
// Saves the current framebuffer to a PPM file
bool saveScreenshot(const std::string& path, int width, int height);
