#version 430 core
in vec2 vUV;
uniform sampler2D uAccum;
uniform int       uSPP;
out vec4 fragColor;

void main() {
    vec3 c = texture(uAccum, vUV).rgb / float(max(uSPP, 1));
    // Reinhard + gamma-2 (sqrt)
    c = c / (1.0 + c);
    c = sqrt(clamp(c, 0.0, 1.0));
    fragColor = vec4(c, 1.0);
}
