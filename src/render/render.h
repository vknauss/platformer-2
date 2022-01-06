#pragma once

#include <ogu/buffer.h>
#include <ogu/shader.h>
#include <ogu/vertex_array.h>

#include <vvm/vvm.hpp>


namespace render {

class renderer {

    ogu::shader_program _program;

    ogu::buffer _vbo;
    ogu::vertex_array _vao;

    ogu::buffer _scene_ubo, _objects_ubo;


public:

    renderer();

    void set_camera_matrix(const vvm::m4f& m);

    void draw_quad(const vvm::v2f& size, const vvm::v2f& pos, float angle, const vvm::v3f color);

    void draw_filled_quad(const vvm::v2f& size, const vvm::v2f& pos, float angle, const vvm::v3f color);

    void draw_line(const vvm::v2f& p0, const vvm::v2f& p1, const vvm::v3f& color);

};

}