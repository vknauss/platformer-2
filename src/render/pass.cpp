#include "pass.h"

#include <stdexcept>


namespace render {

static ogu::shader_program create_shader_program(const std::vector<std::string>& attribute_names, const std::string& vertex_shader, const std::string& fragment_shader) {
    std::vector<std::string> vs_sources;
    vs_sources.reserve(2 + attribute_names.size());
    vs_sources.push_back("#version 330");
    for (auto i = 0u; i < attribute_names.size(); ++i) {
        vs_sources.push_back("layout(location=" + std::to_string(i) + ") in " + attribute_names[i] + ";");
    }
    vs_sources.push_back(vertex_shader);
    return ogu::shader_program({
        { vs_sources, ogu::shader::type::VERTEX },
        { {fragment_shader}, ogu::shader::type::FRAGMENT } });
}

static pass::attribute create_attribute(const std::string& name) {
    auto i = 0u;
    for (; i < name.size(); ++i) {
        if (name[i] == ' ') break;
    }
    if (i == name.size()) throw std::runtime_error("attribute names should be like \"float f\" \"vec3 pos\" etc. not like \"" + name + "\"");
    auto typestr = name.substr(0, i);
    auto namestr = name.substr(i+1, name.size());
    switch (typestr[0]) {
    case 'f':
        if (i == 5 && typestr[1] == 'l' && typestr[2] == 'o' && typestr[3] == 'a' && typestr[4] == 't') {
            return { namestr, 1, false};
        }
        break;
    case 'i':
        if (i == 3 && typestr[1] == 'n' && typestr[2] == 't') {
            return { namestr, 1, true};
        } else if (i == 5 && typestr[1] == 'v' && typestr[2] == 'e' && typestr[3] == 'c') {
            auto size = typestr[4] - '0';
            if (size < 2 || size > 4) throw std::runtime_error("\"" + name + "\" is not a valide size of ivec");
            return { namestr, size, true};
        }
        break;
    case 'u':
        if (i == 4 && typestr[1] == 'i' && typestr[2] == 'n' && typestr[3] == 't') {
            return { namestr, 1, true};
        } else if (i == 5 && typestr[1] == 'v' && typestr[2] == 'e' && typestr[3] == 'c') {
            auto size = typestr[4] - '0';
            if (size < 2 || size > 4) throw std::runtime_error("\"" + name + "\" is not a valide size of uvec");
            return { namestr, size, true};
        }
        break;
    case 'v':
        if (i == 4 && typestr[1] == 'e' && typestr[2] == 'c') {
            auto size = typestr[3] - '0';
            if (size < 2 || size > 4) throw std::runtime_error("\"" + name + "\" is not a valide size of vec");
            return { namestr, size, false};
        }
        break;
    }
    throw std::runtime_error("invalid attribute type \"" + typestr + "\"");
}

pass::pass(const std::vector<std::string>& attribute_names, const std::string& vertex_shader, const std::string& fragment_shader) :
        program(create_shader_program(attribute_names, vertex_shader, fragment_shader)) {
    attributes.reserve(attribute_names.size());
    for (const auto& name : attribute_names) {
        attributes.push_back(create_attribute(name));
    }
}

static ogu::vertex_array create_vao(const pass& pass, const std::vector<vbo_mapping>& mappings) {
    std::vector<ogu::vertex_buffer_binding> bindings;
    bindings.reserve(mappings.size());
    for (const auto& mapping : mappings) {
        bindings.push_back({mapping.buffer});
        auto& binding = bindings.back();
        binding.attribs.reserve(mapping.attributes.size());
        for (const auto& attribute : mapping.attributes) {
            binding.attribs.push_back({});
            auto& b_attrib = binding.attribs.back();
            const auto& p_attrib = pass.attributes[attribute.index];
            b_attrib.integer = p_attrib.integer;
            b_attrib.size = p_attrib.size;
            b_attrib.location = attribute.index;
            b_attrib.type = attribute.type;
            b_attrib.normalized = attribute.normalized;
            b_attrib.stride = mapping.stride;
            b_attrib.offset = attribute.offset;
        }
    }
    return ogu::vertex_array(bindings);
}

vao_resource_binding::vao_resource_binding(const pass& pass, const std::vector<vbo_mapping>& vbo_mappings) :
        vao(create_vao(pass, vbo_mappings)) {
}

}