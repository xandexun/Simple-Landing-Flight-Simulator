// mesh.h - triangle meshes on the GPU (VAO + VBO) and the .obj / .mtl loader
#pragma once
#include <GL/glew.h>
#include <functional>
#include <string>
#include <vector>
#include "engine/vertex.h"
#include "engine/gl_utils.h"
#include "engine/frustum.h"

struct Material {
    std::string name = "default";
    glm::vec3 Ka{0.3f}, Kd{0.8f}, Ks{0.2f};
    float Ns = 32.0f;          // Phong shininess exponent
    GLuint texture = 0;        // diffuse texture (map_Kd), 0 = none
};

// A group of triangles inside one axis-aligned box. Chunks that are outside the view frustum
// are not drawn at all (frustum culling): this is what keeps the huge terrain and runway fast.
struct Chunk {
    GLsizei first = 0;         // first index in the element buffer
    GLsizei count = 0;         // number of indices
    glm::vec3 bmin{0.0f}, bmax{0.0f};
};

struct SubMesh {               // a range of triangles that share one material
    // Before upload(): first/count are vertex positions in the non-indexed input.
    // After upload():  first/count refer to the element (index) buffer.
    GLint first = 0;
    GLsizei count = 0;
    int material = 0;
    std::vector<Chunk> chunks;
};

class Mesh {
public:
    std::vector<SubMesh> parts;
    std::vector<Material> materials;

    // Uploads non-indexed triangles: identical vertices are merged and an index buffer is built
    // (far fewer vertices for the GPU to transform). Triangles are grouped into square chunks of
    // chunkSize (in the xz plane, model units) for frustum culling; 0 = one chunk per material.
    // If parts is empty a single part with material 0 is created.
    void upload(const std::vector<Vertex>& verts, float chunkSize = 0.0f);
    // One part per group of triangles; group i uses materials[i]
    void uploadGroups(const std::vector<std::vector<Vertex>>& groups, float chunkSize = 0.0f);
    // Sets the material uniforms (uKa, uKd, uKs, uShininess, uUseTexture) and draws one part.
    // If fr is given, chunks outside the frustum are skipped (fr must be in model space).
    void drawPart(const Shader& sh, size_t i, const Frustum* fr = nullptr) const;
    void draw(const Shader& sh, const Frustum* fr = nullptr) const;   // all parts with their materials

private:
    GLuint vao_ = 0, vbo_ = 0, ebo_ = 0;
};

// Loads a Wavefront .obj file (v, vt, vn, f with any polygon size, usemtl, mtllib).
// Polygons are triangulated as a fan; missing normals are computed per face.
// keep (optional): only the triangles whose centroid passes the test are loaded
bool loadOBJ(const std::string& path, Mesh& mesh, float chunkSize = 0.0f,
             const std::function<bool(const glm::vec3&)>& keep = nullptr);
