#version 330

layout (location = 0) out vec4 f_color;

layout (std140) uniform obj_data {
    mat4 tfm;
    vec3 color;
};

void main() {
    f_color = vec4(color, 1); //texture(tex, v_uv);
}