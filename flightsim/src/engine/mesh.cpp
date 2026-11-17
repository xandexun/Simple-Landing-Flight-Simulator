// mesh.cpp - GPU meshes and Wavefront OBJ/MTL loader
#include "engine/mesh.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <map>
#include <sstream>
#include <unordered_map>

// ------------------------------------------------------------------ GPU side
static_assert(sizeof(Vertex) == 13 * sizeof(float), "Vertex must be 13 tightly packed floats");

void Mesh::upload(const std::vector<Vertex>& in, float chunkSize) {
    if (materials.empty()) materials.push_back(Material());
    if (parts.empty()) parts.push_back({0, static_cast<GLsizei>(in.size()), 0, {}});

    // 1) merge identical vertices (same bytes) -> unique vertex array + index of every input vertex
    std::vector<Vertex> unique;
    std::vector<uint32_t> remap(in.size());
    std::unordered_map<std::string, uint32_t> seen;
    for (size_t i = 0; i < in.size(); ++i) {
        std::string key(reinterpret_cast<const char*>(&in[i]), sizeof(Vertex));
        auto it = seen.emplace(key, static_cast<uint32_t>(unique.size())).first;
        if (it->second == unique.size()) unique.push_back(in[i]);   // first time this vertex appears
        remap[i] = it->second;
    }

    // 2) per material: group the triangles in chunks and write the index buffer chunk by chunk
    std::vector<uint32_t> indices;
    indices.reserve(in.size());
    for (SubMesh& part : parts) {
        std::map<std::pair<int, int>, std::vector<uint32_t>> bins;
        const size_t t0 = static_cast<size_t>(part.first) / 3, t1 = (static_cast<size_t>(part.first) + part.count) / 3;
        for (size_t t = t0; t < t1; ++t) {
            const uint32_t a = remap[3 * t], b = remap[3 * t + 1], c = remap[3 * t + 2];
            int bx = 0, bz = 0;
            if (chunkSize > 0.0f) {
                glm::vec3 centre = (unique[a].pos + unique[b].pos + unique[c].pos) / 3.0f;
                bx = static_cast<int>(std::floor(centre.x / chunkSize));
                bz = static_cast<int>(std::floor(centre.z / chunkSize));
            }
            bins[{bx, bz}].insert(bins[{bx, bz}].end(), {a, b, c});
        }
        part.chunks.clear();
        part.first = static_cast<GLint>(indices.size());
        for (auto& kv : bins) {
            Chunk ch{static_cast<GLsizei>(indices.size()), static_cast<GLsizei>(kv.second.size()), glm::vec3(1e30f), glm::vec3(-1e30f)};
            for (uint32_t idx : kv.second) {
                ch.bmin = glm::min(ch.bmin, unique[idx].pos);
                ch.bmax = glm::max(ch.bmax, unique[idx].pos);
            }
            indices.insert(indices.end(), kv.second.begin(), kv.second.end());
            part.chunks.push_back(ch);
        }
        part.count = static_cast<GLsizei>(indices.size()) - part.first;
    }

    // 3) GPU buffers
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(unique.size() * sizeof(Vertex)), unique.data(), GL_STATIC_DRAW);
    glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);          // stays bound to the VAO
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)), indices.data(), GL_STATIC_DRAW);
    vertexAttribs({3, 3, 2, 3, 2});   // position, normal, uv, colour, extra (see vertex.h)
    glBindVertexArray(0);
}

void Mesh::uploadGroups(const std::vector<std::vector<Vertex>>& groups, float chunkSize) {
    std::vector<Vertex> all;
    parts.clear();
    for (size_t g = 0; g < groups.size(); ++g) {
        parts.push_back({static_cast<GLint>(all.size()), static_cast<GLsizei>(groups[g].size()), static_cast<int>(g), {}});
        all.insert(all.end(), groups[g].begin(), groups[g].end());
    }
    upload(all, chunkSize);
}

void Mesh::drawPart(const Shader& sh, size_t i, const Frustum* fr) const {
    const SubMesh& p = parts[i];
    const Material& m = materials[p.material];
    sh.set("uKa", m.Ka);
    sh.set("uKd", m.Kd);
    sh.set("uKs", m.Ks);
    sh.set("uShininess", m.Ns);
    sh.set("uUseTexture", m.texture != 0);
    if (m.texture) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m.texture);
        sh.set("uTexture", 0);
    }
    glBindVertexArray(vao_);

    // draw the visible chunks; neighbouring visible chunks are merged into one draw call
    GLsizei runFirst = 0, runCount = 0;
    auto flush = [&]() {
        if (runCount > 0)
            glDrawElements(GL_TRIANGLES, runCount, GL_UNSIGNED_INT,
                           reinterpret_cast<const void*>(static_cast<uintptr_t>(runFirst) * sizeof(uint32_t)));
        runCount = 0;
    };
    for (const Chunk& c : p.chunks) {
        if (fr && !fr->intersectsAABB(c.bmin, c.bmax)) { flush(); continue; }
        if (runCount > 0 && runFirst + runCount == c.first) runCount += c.count;
        else { flush(); runFirst = c.first; runCount = c.count; }
    }
    flush();
}

void Mesh::draw(const Shader& sh, const Frustum* fr) const {
    for (size_t i = 0; i < parts.size(); ++i) drawPart(sh, i, fr);
}

// ------------------------------------------------------------------ OBJ loader
namespace {

std::string directoryOf(const std::string& path) {
    size_t p = path.find_last_of("/\\");
    return (p == std::string::npos) ? std::string() : path.substr(0, p + 1);
}

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return (a == std::string::npos) ? std::string() : s.substr(a, b - a + 1);
}

void loadMTL(const std::string& path, std::vector<Material>& out) {
    std::string text = readTextFile(path);
    if (text.empty()) { std::cerr << "Warning: material file not found: " << path << "\n"; return; }
    const std::string dir = directoryOf(path);
    std::istringstream in(text);
    std::string line;
    Material* cur = nullptr;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ls(line);
        std::string key;
        ls >> key;
        if (key == "newmtl") {
            out.push_back(Material());
            cur = &out.back();
            cur->name = trim(line.substr(6));
        } else if (!cur) {
            continue;
        }
        else if (key == "Ka") ls >> cur->Ka.x >> cur->Ka.y >> cur->Ka.z;
        else if (key == "Kd") ls >> cur->Kd.x >> cur->Kd.y >> cur->Kd.z;
        else if (key == "Ks") ls >> cur->Ks.x >> cur->Ks.y >> cur->Ks.z;
        else if (key == "Ns") ls >> cur->Ns;
        else if (key == "map_Kd") {
            std::string file = trim(line.substr(6));
            size_t slash = file.find_last_of("/\\");               // ignore absolute paths
            if (slash != std::string::npos) file = file.substr(slash + 1);
            Image img;
            if (loadPNM(dir + file, img)) cur->texture = createTexture(img, true, true);
            else std::cerr << "Warning: texture not loaded (use binary .ppm): " << dir + file << "\n";
        }
    }
}

// Parses an OBJ face index (1-based, negative = relative to the end). Returns -1 if absent.
int resolveIndex(long idx, size_t count) {
    if (idx > 0) return static_cast<int>(idx - 1);
    if (idx < 0) return static_cast<int>(static_cast<long>(count) + idx);
    return -1;
}

void readFloats(const char* p, float* out, int n) {
    char* next = nullptr;
    for (int i = 0; i < n; ++i) { out[i] = std::strtof(p, &next); p = next; }
}

} // namespace

bool loadOBJ(const std::string& path, Mesh& mesh, float chunkSize, const std::function<bool(const glm::vec3&)>& keep) {
    std::string text = readTextFile(path);
    if (text.empty()) return false;

    std::vector<glm::vec3> P, N;
    std::vector<glm::vec2> T;
    std::vector<Material> mats;
    std::map<std::string, int> matIndex;
    std::vector<std::vector<Vertex>> perMat;      // triangles grouped by material
    int current = -1;

    auto useMaterial = [&](const std::string& name) {
        current = matIndex.emplace(name, static_cast<int>(perMat.size())).first->second;
        if (current == static_cast<int>(perMat.size())) perMat.emplace_back();   // new material
    };

    const char* s = text.c_str();
    const char* end = s + text.size();
    std::vector<int> fv, ft, fn;
    while (s < end) {
        const char* eol = static_cast<const char*>(std::memchr(s, '\n', static_cast<size_t>(end - s)));
        if (!eol) eol = end;
        std::string line(s, eol);
        s = eol + 1;
        if (line.size() < 2 || line[0] == '#') continue;
        const char* c = line.c_str();
        char* next = nullptr;

        float f[3];
        if (c[0] == 'v' && c[1] == ' ') { readFloats(c + 2, f, 3); P.emplace_back(f[0], f[1], f[2]); }
        else if (c[0] == 'v' && c[1] == 't') { readFloats(c + 3, f, 2); T.emplace_back(f[0], f[1]); }
        else if (c[0] == 'v' && c[1] == 'n') { readFloats(c + 3, f, 3); N.emplace_back(f[0], f[1], f[2]); }
        else if (c[0] == 'f' && c[1] == ' ') {
            if (current < 0) useMaterial("default");
            fv.clear(); ft.clear(); fn.clear();
            const char* p = c + 2;
            while (*p) {
                while (*p == ' ' || *p == '\t' || *p == '\r') ++p;
                if (!*p) break;
                long vi = std::strtol(p, &next, 10), ti = 0, ni = 0;
                p = next;
                if (*p == '/') {
                    ++p;
                    if (*p != '/') { ti = std::strtol(p, &next, 10); p = next; }
                    if (*p == '/') { ++p; ni = std::strtol(p, &next, 10); p = next; }
                }
                while (*p && *p != ' ' && *p != '\t' && *p != '\r') ++p;
                fv.push_back(resolveIndex(vi, P.size()));
                ft.push_back(resolveIndex(ti, T.size()));
                fn.push_back(resolveIndex(ni, N.size()));
            }
            // fan triangulation: (0, i, i+1)
            for (size_t i = 1; i + 1 < fv.size(); ++i) {
                size_t idx[3] = {0, i, i + 1};
                glm::vec3 a = P[fv[idx[0]]], b = P[fv[idx[1]]], cc = P[fv[idx[2]]];
                if (keep && !keep((a + b + cc) / 3.0f)) continue;
                glm::vec3 faceN = glm::cross(b - a, cc - a);
                float len = glm::length(faceN);
                faceN = (len > 1e-12f) ? faceN / len : glm::vec3(0.0f, 1.0f, 0.0f);
                for (size_t k : idx)
                    perMat[current].push_back({P[fv[k]], (fn[k] >= 0) ? N[fn[k]] : faceN,
                                               (ft[k] >= 0) ? T[ft[k]] : glm::vec2(0.0f), glm::vec3(1.0f)});
            }
        } else if (line.compare(0, 6, "usemtl") == 0) {
            useMaterial(trim(line.substr(6)));
        } else if (line.compare(0, 6, "mtllib") == 0) {
            loadMTL(directoryOf(path) + trim(line.substr(6)), mats);
        }
    }

    // one part per material (in the order they were first used)
    mesh.materials.assign(perMat.size(), Material());
    size_t triangles = 0;
    for (auto& kv : matIndex) {
        Material& mat = mesh.materials[kv.second];
        mat.name = kv.first;
        for (const Material& x : mats) if (x.name == kv.first) mat = x;
        triangles += perMat[kv.second].size() / 3;
    }
    mesh.uploadGroups(perMat, chunkSize);
    std::cout << "Loaded " << path << ": " << P.size() << " vertices, " << triangles
              << " triangles, " << mesh.parts.size() << " materials\n";
    return triangles > 0;
}
