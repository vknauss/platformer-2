#include "render.h"

#include <vvm/matrix_tfm.hpp>

#include "util.h"

#define SHADER_PATH(X) SHADERS_DIR "/" X

// for testing. enabling this will make rendering significantly slower
#define RENDER_IMMEDIATE 0


namespace render {

static ogu::shader create_shader(const std::string& filename, ogu::shader::type type) {
    return {{utils::file_string(filename)}, type};
}

static ogu::vertex_buffer_binding create_vbo_binding(const ogu::buffer& vbo) {
    ogu::vertex_buffer_binding b {vbo};
    b.instanced = false;
    b.stride = sizeof(vvm::v2f);
    b.attribs.push_back({.location = 0, .size = 2, .type = GL_FLOAT});
    return b;
}

static void write_vbo(const ogu::buffer& vbo) {
    vbo.write(0, 0, [] (void* ptr) {
        vvm::v2f* vptr = (vvm::v2f*) ptr;
        *(vptr++) = {-0.5, -0.5};
        *(vptr++) = { 0.5, -0.5};
        *(vptr++) = {-0.5,  0.5};
        *(vptr++) = { 0.5,  0.5};
        *(vptr++) = {-0.5, -0.5};
        *(vptr++) = {-0.5,  0.5};
        *(vptr++) = { 0.5, -0.5};
        *(vptr++) = { 0.5,  0.5};
        *(vptr++) = { 0.0,  0.0};
        *(vptr++) = { 0.5,  0.0};
    });
}

renderer::renderer() :
        _program({ create_shader(SHADER_PATH("vert.glsl"), ogu::shader::type::VERTEX),
                   create_shader(SHADER_PATH("frag.glsl"), ogu::shader::type::FRAGMENT) }),
        _vbo(10 * sizeof(vvm::v2f)),
        _vao({ create_vbo_binding(_vbo) }),
        _scene_ubo(sizeof(vvm::m4f)),
        _objects_ubo(16384) {
    write_vbo(_vbo);
    _vao.bind();

    _program.use();
    _program.addUniformBuffer("scene_data");
    _program.addUniformBuffer("obj_data");
    _program.bindUniformBuffer("scene_data", _scene_ubo);
    _program.bindUniformBuffer("obj_data", _objects_ubo);
}

void renderer::set_camera_matrix(const vvm::m4f& m) {
    _scene_ubo.write(0, 0, [&] (void* ptr) { *(vvm::m4f*) ptr = m; });
    _program.bindUniformBuffer("scene_data", _scene_ubo);
}

static constexpr vvm::m4f model_matrix(const vvm::v2f& size, const vvm::v2f& pos, float angle) {
    return vvm::translate(pos) * vvm::m4f(vvm::rotate(angle)) * vvm::scale(vvm::v3f(size, 0));
}

static void write_obj_data(const ogu::buffer& ubo, const vvm::m4f& model, const vvm::v3f& color) {
    ubo.write(0, sizeof(vvm::m4f) + sizeof(vvm::v4f), [&] (void* ptr) {
        *(vvm::m4f*) ptr = model;
        ptr = (char*) ptr + sizeof(vvm::m4f);
        *(vvm::v3f*) ptr = color;
    });
}

void renderer::draw_quad(const vvm::v2f& size, const vvm::v2f& pos, float angle, const vvm::v3f color) {
    auto model = model_matrix(size, pos, angle);
    #if RENDER_IMMEDIATE == 1
    write_obj_data(_objects_ubo, model_matrix(size, pos, angle), color);
    _program.bindUniformBuffer("obj_data", _objects_ubo);
    glDrawArrays(GL_LINES, 0, 10);
    #else
    _draw_calls.push_back({model, color, GL_LINES, 10});
    #endif
}

void renderer::draw_filled_quad(const vvm::v2f& size, const vvm::v2f& pos, float angle, const vvm::v3f color) {
    auto model = model_matrix(size, pos, angle);
    #if RENDER_IMMEDIATE == 1
    write_obj_data(_objects_ubo, model, color);
    _program.bindUniformBuffer("obj_data", _objects_ubo);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    #else
    _draw_calls.push_back({model, color, GL_TRIANGLE_STRIP, 4});
    #endif
}

void renderer::draw_line(const vvm::v2f& p0, const vvm::v2f& p1, const vvm::v3f& color) {
    auto model = vvm::translate(p0) * vvm::m4f(vvm::m2f(p1 - p0, {0, 0})) * vvm::translate(vvm::v2f{0.5, 0.5});
    #if RENDER_IMMEDIATE == 1
    write_obj_data(_objects_ubo, model, color);
    _program.bindUniformBuffer("obj_data", _objects_ubo);
    glDrawArrays(GL_LINES, 0, 2);
    #else
    _draw_calls.push_back({model, color, GL_LINES, 2});
    #endif
}

void renderer::render() {
    #if RENDER_IMMEDIATE == 1
    return;
    #endif

    auto obj_data_size = sizeof(vvm::m4f) + sizeof(vvm::v4f);

    _objects_ubo.write(0, _draw_calls.size() * obj_data_size, [this] (void* ptr) {
        for (const auto& call : _draw_calls) {
            *(vvm::m4f*) ptr = call.tfm;
            ptr = (char*) ptr + sizeof(vvm::m4f);
            *(vvm::v3f*) ptr = call.color;
            ptr = (char*) ptr + sizeof(vvm::v4f);  // v4f because std140
        }
    });

    for (auto i = 0u; i < _draw_calls.size(); ++i) {
        _program.bindUniformBuffer("obj_data", _objects_ubo, i * obj_data_size, obj_data_size);
        glDrawArrays(_draw_calls[i].mode, 0, _draw_calls[i].count);
    }

    _draw_calls.clear();
}

}