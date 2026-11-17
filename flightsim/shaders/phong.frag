#version 330 core
// Phong illumination model with one point light (the sun):
//   I = Ka*Ia + Kd*Il*max(N.L,0) + Ks*Il*max(R.V,0)^n
// plus distance fog that blends with the sky colour.
in vec3 vWorldPos, vNormal, vColor, vObjNormal;
in vec2 vUV;
flat in vec2 vExtra;      // buildings: x = wall height (m), y = window style (integer part) + seed (fraction)
out vec4 FragColor;

uniform vec3 uLightPos, uLightColor, uAmbient, uViewPos;       // light and camera
uniform vec3 uKa, uKd, uKs;                                     // material
uniform float uShininess;
uniform bool uUseTexture, uUseVertexColor;
uniform sampler2D uTexture;
uniform vec3 uTint;          // multiplies the base colour (the burnt wreck)
// procedural patterns: 0 = none, 1 = ground detail (terrain, asphalt), 3 = building facades (windows)
uniform int uPattern;
uniform float uNight;
uniform vec3 uSkyHorizon, uSunDir, uSunColor;                   // fog / sky
uniform float uFogDensity;
uniform float uAlpha;        // < 1 for the blurred spinning propeller

float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
float noise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y);
}

void main() {
    float dist = length(uViewPos - vWorldPos);
    vec3 base = uKd;
    if (uUseTexture) base *= texture(uTexture, vUV).rgb;
    if (uUseVertexColor) base *= vColor;
    base *= uTint;

    vec3 ka = uKa, ks = uKs;
    float shininess = uShininess;
    vec3 emissive = vec3(0.0);

    if (uPattern == 1) {
        // terrain: small scale variation so the grass does not look flat
        // (only within 4 km: further away the detail is invisible and costs fill rate)
        float k = 1.0 - smoothstep(1500.0, 4000.0, dist);
        if (k > 0.0) {
            float n = noise(vWorldPos.xz * 0.08) * 0.6 + noise(vWorldPos.xz * 0.6) * 0.4;
            base *= mix(1.0, 0.82 + 0.36 * n, k);
        }
    } else if (uPattern == 3 && vExtra.y >= 1.0 && abs(vObjNormal.y) < 0.5 && dist < 3500.0) {
        // building facade; vUV = (metres along the wall, metres above the ground)
        float style = floor(vExtra.y + 0.001);          // 1 = punched windows, 2 = glass curtain wall, 3 = house
        float seed = fract(vExtra.y);
        float H = vExtra.x;
        float u = vUV.x, v = vUV.y;
        vec2 cell, lo, hi;
        float v0 = min(3.0, H * 0.3);                   // plain ground floor (lobby)
        bool glass = false;
        // few, large windows: one every 7-8 m along the wall and one per floor
        if (style > 2.5)      { cell = vec2(u / 7.0, v / 3.2); lo = vec2(0.36, 0.30); hi = vec2(0.64, 0.80); v0 = 0.7; }
        else if (style > 1.5) { cell = vec2(u / 7.5, v / 4.2); lo = vec2(0.03, 0.05); hi = vec2(0.97, 0.95); glass = true; }
        else                  { cell = vec2(u / 8.0, v / 4.2); lo = vec2(0.30, 0.28); hi = vec2(0.70, 0.80); }
        vec2 f = fract(cell);
        bool win = f.x > lo.x && f.x < hi.x && f.y > lo.y && f.y < hi.y && v > v0 && v < H - (glass ? 0.4 : 0.9);
        // some places have plain wall instead of a window (not on glass towers)
        if (win && !glass && hash(floor(cell) + vec2(seed * 53.0, 7.0)) < 0.3) win = false;
        if (win) {
            float rnd = hash(floor(cell) + vec2(seed * 97.0, 3.0));
            if (glass) {
                // tinted glass that reflects the sky: darker at the bottom, brighter at the top
                float gv = clamp(v / max(H, 1.0), 0.0, 1.0);
                base = base * mix(0.30, 0.85, gv) + vec3(0.02, 0.04, 0.07) + 0.04 * rnd;
                ks = vec3(0.75);
                shininess = 110.0;
            } else {
                base = vec3(0.09, 0.12, 0.16) * (0.8 + 0.5 * rnd);
                ks = vec3(0.6);
                shininess = 80.0;
            }
            float lit = step(glass ? 0.75 : 0.6, hash(floor(cell) + vec2(seed * 97.0, 11.0)));
            emissive += (glass ? vec3(0.75, 0.70, 0.55) : vec3(1.0, 0.82, 0.45)) * lit * uNight * 0.8;
        }
    }

    vec3 N = normalize(vNormal);
    if (!gl_FrontFacing) N = -N;                       // double sided surfaces
    vec3 L = normalize(uLightPos - vWorldPos);         // point light direction
    vec3 V = normalize(uViewPos - vWorldPos);
    vec3 R = reflect(-L, N);

    float diff = max(dot(N, L), 0.0);
    bool shiny = (ks.r + ks.g + ks.b) > 0.0;          // terrain has no specular: skip the pow()
    float spec = (diff > 0.0 && shiny) ? pow(max(dot(R, V), 0.0), max(shininess, 1.0)) : 0.0;

    vec3 color = ka * uAmbient * base           // ambient
               + base * uLightColor * diff       // diffuse (Kd already in base)
               + ks * uLightColor * spec         // specular
               + emissive;

    // exponential squared fog towards the horizon colour (with sun glow)
    float fogT = dist * uFogDensity;
    float fog = exp(-fogT * fogT);
    vec3 viewDir = normalize(vWorldPos - uViewPos);
    vec3 fogColor = uSkyHorizon + uSunColor * pow(max(dot(viewDir, uSunDir), 0.0), 8.0) * 0.35 * (1.0 - uNight);
    color = mix(fogColor, color, fog);

    FragColor = vec4(color, uAlpha);
}
