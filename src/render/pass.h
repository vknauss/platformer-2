#pragma once

#include <ogu/buffer.h>
#include <ogu/shader.h>
#include <ogu/vertex_array.h>

#include <string>
#include <vector>


namespace render {

struct vbo_mapping {
    const ogu::buffer& buffer;

    GLintptr stride;

    struct attribute {
        uint32_t index;

        GLenum type;
        GLboolean normalized;
        GLsizeiptr offset;
    };

    std::vector<attribute> attributes;

};


struct pass {
    ogu::shader_program program;

    struct attribute {
        std::string name;
        GLint size;
        bool integer;
    };

    std::vector<attribute> attributes;

    pass(const std::vector<std::string>& attribute_names, const std::string& vertex_shader, const std::string& fragment_shader);

};

struct vao_resource_binding {
    ogu::vertex_array vao;

    vao_resource_binding(const pass& pass, const std::vector<vbo_mapping>& vbo_mappings);
};

}