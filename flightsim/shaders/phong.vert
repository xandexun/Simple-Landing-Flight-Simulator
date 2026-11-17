#version 330 core
// Phong shading: the vertex shader only transforms; lighting is computed per fragment.
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUV;
layout(location = 3) in vec3 aColor;
layout(location = 4) in vec2 aExtra;

uniform mat4 uModel, uView, uProj;
uniform mat3 uNormalMatrix;   // transpose(inverse(mat3(uModel)))

out vec3 vWorldPos, vNormal, vColor;
out vec3 vObjNormal;          // object-space normal (only walls get windows)
out vec2 vUV;
flat out vec2 vExtra;

void main() {
    vec4 world = uModel * vec4(aPos, 1.0);
    vWorldPos = world.xyz;
    vNormal = uNormalMatrix * aNormal;
    vUV = aUV;
    vColor = aColor;
    vExtra = aExtra;
    vObjNormal = aNormal;
    gl_Position = uProj * uView * world;
}
