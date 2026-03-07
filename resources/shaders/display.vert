#version 430 core
out vec2 vUV;
void main() {
    float x = (gl_VertexID == 1) ?  3.0 : -1.0;
    float y = (gl_VertexID == 2) ?  3.0 : -1.0;
    gl_Position = vec4(x, y, 0.0, 1.0);
    // UV in [0,1] over the visible triangle; flip Y to correct image orientation
    vUV = vec2(x * 0.5 + 0.5, 1.0 - (y * 0.5 + 0.5));
}
