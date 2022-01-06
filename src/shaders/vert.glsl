#version 330

layout (location = 0) in vec2 position;

layout (std140) uniform scene_data_t {
    mat4 mvp;
} scene_data;

void main() {
    vec4 p = vec4(position, 0, 1);
    gl_Position = scene_data.mvp * p;
}