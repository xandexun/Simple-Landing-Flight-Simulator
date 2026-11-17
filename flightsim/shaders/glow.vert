#version 330 core
// Point sprites: runway / approach lights, and the explosion particles (fire, sparks, smoke)
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
layout(location = 2) in float aSize;
layout(location = 3) in float aAlpha;
uniform mat4 uView, uProj;
uniform float uViewportHeight;
uniform float uFovScale;     // 1 / tan(fov/2)
uniform float uSizeScale;    // 2.5 for lights (halo), 1.0 for particles
uniform float uMaxSize;      // pixels
out vec3 vColor;
out float vAlpha;
void main() {
    vec4 viewPos = uView * vec4(aPos, 1.0);
    gl_Position = uProj * viewPos;
    float dist = max(-viewPos.z, 0.1);
    gl_PointSize = clamp(aSize * uViewportHeight * 0.5 * uFovScale / dist * uSizeScale, 2.0, uMaxSize);
    vColor = aColor;
    vAlpha = aAlpha;
}
