#version 330 core
in vec3 vColor;
in float vAlpha;
out vec4 FragColor;
uniform float uIntensity;
uniform int uMode;           // 0 = light (halo + core, additive), 1 = fire (additive), 2 = smoke (alpha blended)
void main() {
    vec2 c = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(c, c);
    if (r2 > 1.0) discard;
    if (uMode == 0) {
        float glow = exp(-r2 * 4.0) + (1.0 - smoothstep(0.0, 0.12, r2)) * 1.5;   // halo + bright core
        FragColor = vec4(vColor * glow * uIntensity, 1.0);
    } else if (uMode == 1) {
        float glow = exp(-r2 * 3.0) * (1.0 - r2);
        FragColor = vec4(vColor * glow * vAlpha * 1.6, 1.0);
    } else {
        float a = vAlpha * pow(1.0 - r2, 1.5);
        FragColor = vec4(vColor, a);
    }
}
