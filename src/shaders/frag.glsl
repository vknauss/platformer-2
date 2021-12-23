#version 330

layout (location = 0) out vec4 f_color;

in vec2 v_uv;

uniform vec3 color;

void main() {
    f_color = vec4(color, 1); //texture(tex, v_uv);
}