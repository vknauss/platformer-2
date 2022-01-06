#version 330

layout (location = 0) in vec2 position;

layout (std140) uniform scene_data {
    mat4 mvp;
};

layout (std140) uniform obj_data {
    mat4 tfm;
    vec3 color;
};

void main() {
    vec4 p = tfm * vec4(position, 0, 1);
    gl_Position = mvp * p;
}