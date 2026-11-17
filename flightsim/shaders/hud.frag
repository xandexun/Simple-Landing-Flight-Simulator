#version 330 core
in vec2 vUV;
in vec4 vColor;
out vec4 FragColor;
uniform sampler2D uFont;
void main() {
    float a = 1.0;
    if (vUV.x > -0.5) a = texture(uFont, vUV).r;     // text: font atlas alpha; shapes: uv = -1
    FragColor = vec4(vColor.rgb, vColor.a * a);
}
