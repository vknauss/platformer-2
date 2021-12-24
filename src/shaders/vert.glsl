#version 330

// vec2 vertices[4] = vec2[4](
//     vec2(-0.5, -0.5), vec2(0.5, -0.5), vec2(-0.5, 0.5), vec2(0.5, 0.5));

// vec2 uvs[4] = vec2[4](
//     vec2(0, 0), vec2(1, 0), vec2(0, 1), vec2(1, 1));

// out vec2 v_uv;

layout (location = 0) in vec2 position;

uniform mat4 mvp;

void main() {
    // vec4 p = vec4(vertices[gl_VertexID], 0, 1);
    vec4 p = vec4(position, 0, 1);
    gl_Position = mvp * p;
    // v_uv = uvs[gl_VertexID];
}