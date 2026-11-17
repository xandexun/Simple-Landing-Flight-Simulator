#version 330 core
// Procedural sky: gradient horizon -> zenith, sun disc and glow, stars at night.
in vec2 vNdc;
out vec4 FragColor;
uniform mat4 uInvViewProj;
uniform vec3 uViewPos, uSkyZenith, uSkyHorizon, uSunDir, uSunColor;
uniform float uNight;

float hash(vec3 p) { return fract(sin(dot(p, vec3(127.1, 311.7, 74.7))) * 43758.5453); }

void main() {
    vec4 farP = uInvViewProj * vec4(vNdc, 1.0, 1.0);
    vec3 dir = normalize(farP.xyz / farP.w - uViewPos);

    float h = dir.y;
    vec3 col = mix(uSkyHorizon, uSkyZenith, pow(clamp(h, 0.0, 1.0), 0.55));
    if (h < 0.0) col = uSkyHorizon * (1.0 + h * 0.6);           // below the horizon

    float s = max(dot(dir, uSunDir), 0.0);
    col += uSunColor * (pow(s, 8.0) * 0.35 + pow(s, 64.0) * 0.5) * (1.0 - uNight * 0.8);
    col += uSunColor * smoothstep(0.9995, 0.9998, s) * 3.0;     // sun disc

    // stars
    if (uNight > 0.01 && h > 0.0) {
        vec3 cell = floor(dir * 300.0);
        float star = step(0.9975, hash(cell));
        col += vec3(star) * uNight * h * 1.2;
    }
    FragColor = vec4(col, 1.0);
}
