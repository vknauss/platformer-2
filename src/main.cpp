#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <ogu/shader.h>
#include <ogu/vertex_array.h>

#include <GLFW/glfw3.h>

#include <vvm/vvm.hpp>
#include <vvm/matrix_tfm.hpp>

#include "physics/physics.h"


enum ErrorCode {
    NO_ERROR,
    GLFW_INIT_FAILED,
    WINDOW_CREATION_FAILED,
    GLEW_INIT_FAILED
};

struct App {
    GLFWwindow* window;
    bool isRunning;
};

ErrorCode init(App& app) {
    app.isRunning = false;

    if (!glfwInit()) {
        return GLFW_INIT_FAILED;
    }

    app.window = glfwCreateWindow(1280, 800, "Window", nullptr, nullptr);
    if (!app.window) {
       return WINDOW_CREATION_FAILED;
    }

    glfwMakeContextCurrent(app.window);

    if (glewInit() != GLEW_OK) {
        return GLEW_INIT_FAILED;
    }

    app.isRunning = true;

    return NO_ERROR;
}

void printError(ErrorCode err) {
    switch (err) {
    case GLFW_INIT_FAILED:
        std::cerr << "Failed to initialize GLFW." << std::endl;
        break;
    case WINDOW_CREATION_FAILED:
        std::cerr << "Failed to create window." << std::endl;
        break;
    case GLEW_INIT_FAILED:
        std::cerr << "Failed to initialize GLEW." << std::endl;
        break;
    default:
        break;
    }
}

void cleanup(ErrorCode err, App& app) {
    switch (err) {
    case NO_ERROR:
    case GLEW_INIT_FAILED:
        glfwDestroyWindow(app.window);
    case WINDOW_CREATION_FAILED:
        glfwTerminate();
    case GLFW_INIT_FAILED:
    default:
        break;
    }
}

void compileShader(GLuint shader, const char* shaderSource) {
    glShaderSource(shader, 1, &shaderSource, nullptr);
    glCompileShader(shader);

    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (!status) {
        std::cerr << "Shader compile failed." << std::endl;
        GLint infoLogLength;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLogLength);
        char* infoLog = new char[infoLogLength];
        glGetShaderInfoLog(shader, infoLogLength, nullptr, infoLog);
        std::cerr << "Shader info log: " << infoLog << std::endl;
        delete [] infoLog;
    }
}

std::string loadShaderSource(const std::string& filename) {
    auto ifs = std::ifstream(std::string(SHADERS_DIR) + "/" + filename);
    if (!ifs) {
        std::cerr << "Failed to load shader file." << std::endl;
    }

    return std::string {std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()};
}

GLuint createShaderProgram(const std::string& vsFilename, const std::string& fsFilename) {
    GLuint program = glCreateProgram();
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);

    auto vsSource = loadShaderSource(vsFilename);
    auto fsSource = loadShaderSource(fsFilename);

    compileShader(vs, vsSource.c_str());
    compileShader(fs, fsSource.c_str());

    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    glDetachShader(program, vs);
    glDetachShader(program, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint status;
    glGetProgramiv(program, GL_LINK_STATUS, &status);
    if (!status) {
        std::cerr << "Program link failed." << std::endl;
    }

    return program;
}

template<typename T,
    unsigned int E,
    unsigned int I>
struct packed_array {
    static constexpr unsigned int MAX_ELEMENTS = E;
    static constexpr unsigned int MAX_INDICES = I;
    
    T data[MAX_ELEMENTS];
    unsigned int numElements;
    unsigned int elementIndices[MAX_ELEMENTS];

    unsigned int indices[MAX_INDICES];

    packed_array() : numElements(0) {
        std::fill(indices, indices + MAX_INDICES, MAX_ELEMENTS);
    }

    const T& get(unsigned int index) const {
        static_assert(index < MAX_INDICES, "get() (const): index out of bounds");
        return data[indices[index]];
    }

    T& get(unsigned int index) {
        static_assert(index < MAX_INDICES, "get(): index out of bounds");
        return data[indices[index]];
    }

    bool has(unsigned int index) const {
        static_assert(index < MAX_INDICES, "has(): index out of bounds");
        return indices[index] != MAX_ELEMENTS;
    }

    void insert(unsigned int index, const T& t) {
        static_assert(numElements < MAX_ELEMENTS, "too many elements. cant insert");
        indices[index] = numElements;
        elementIndices[numElements] = index;
        data[numElements++] = t;
    }

    void remove(unsigned int index) {
        --numElements;
        if (indices[index] < numElements) {
            data[indices[index]] = data[numElements];
            indices[elementIndices[numElements]] = indices[index];
            indices[index] = MAX_ELEMENTS;
        }
    }

};

typedef unsigned int entity;
constexpr unsigned long MAX_ENTITIES = 1<<16;
struct ecs {
    unsigned char entityAliveBitBuffer[MAX_ENTITIES / 8 + 1];
    unsigned int maxEntity;
    unsigned int numEntities;
    entity freeIndices[64];
    unsigned int firstFreeIndex;
    unsigned int numFreeIndices;
};

void initECS(ecs& ecs) {
    std::fill(ecs.entityAliveBitBuffer, ecs.entityAliveBitBuffer + (MAX_ENTITIES / 8 + 1), 0);
    ecs.maxEntity = 0;
    ecs.numEntities = 0;
    ecs.firstFreeIndex = 0;
    ecs.numFreeIndices = 0;
}

bool entityAlive(entity e, const ecs& ecs) {
    return e < ecs.maxEntity && ecs.entityAliveBitBuffer[e/8] & (1<<(e%8));
}

entity createEntity(ecs& ecs) {
    if (ecs.numEntities == MAX_ENTITIES) {
        return ecs.numEntities - 1;  // oops 
    }

    entity e;
    if (ecs.maxEntity < MAX_ENTITIES) {
        e = ecs.maxEntity++;
    } else {
        e = ecs.freeIndices[ecs.firstFreeIndex];
        ecs.firstFreeIndex = (ecs.firstFreeIndex + 1) % 64;
        --ecs.numFreeIndices;
    }

    ecs.entityAliveBitBuffer[e/8] |= 1 << e%8;
    ++ecs.numEntities;

    return e;
}

void destroyEntity(entity e, ecs& ecs) {
    if (entityAlive(e, ecs)) {
        ecs.entityAliveBitBuffer[e/8] &= 0xFF - (1 << e%8);
        ecs.freeIndices[(ecs.firstFreeIndex + ecs.numFreeIndices) & 64] = e;
        ++ecs.numFreeIndices;
    }
}

void run(App& app) {

    // GLuint vao;
    // glGenVertexArrays(1, &vao);
    // glBindVertexArray(vao);
    // ogu::vertex_array vao;
    
    // ogu::buffer vbo;
    // vbo.allocate(10 * sizeof(vvm::v2f), GL_STATIC_DRAW);



    // GLuint vbo;
    // glGenBuffers(1, &vbo);
    // glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // glBufferData(GL_ARRAY_BUFFER, 10 * sizeof(vvm::v2f), nullptr, GL_STATIC_DRAW);
    // {
    //     // void* ptr = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    //     void* ptr = vbo.map(GL_WRITE_ONLY);
    //     vvm::v2f* vptr = (vvm::v2f*) ptr;
    //     *(vptr++) = {-0.5, -0.5};
    //     *(vptr++) = { 0.5, -0.5};
    //     *(vptr++) = {-0.5,  0.5};
    //     *(vptr++) = { 0.5,  0.5};
    //     *(vptr++) = {-0.5, -0.5};
    //     *(vptr++) = {-0.5,  0.5};
    //     *(vptr++) = { 0.5, -0.5};
    //     *(vptr++) = { 0.5,  0.5};
    //     *(vptr++) = { 0.0,  0.0};
    //     *(vptr++) = { 0.5,  0.0};
    //     // glUnmapBuffer(GL_ARRAY_BUFFER);
    //     vbo.unmap();
    // }

    // vao.add_binding({0, 2, GL_FLOAT, 0, 0, false, false});
    // glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);
    // glEnableVertexAttribArray(0);

    ogu::VertexArray vao(10 * sizeof(vvm::v2f), 0, 0);
    
    vao.getVertexBuffer().write(0, 0, [] (void* ptr) {
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

    vao.addAttribute({0, 2});

    vao.createAttributeBindings();
    vao.enable();

    ogu::ShaderProgram program({
        {std::string(SHADERS_DIR) + "/vert.glsl", ogu::Shader::VERTEX},
        {std::string(SHADERS_DIR) + "/frag.glsl", ogu::Shader::FRAGMENT} });
    program.addUniform("mvp");
    program.addUniform("color");

    // GLuint program = createShaderProgram("vert.glsl", "frag.glsl");

    // auto uMVP = glGetUniformLocation(program, "mvp");
    // auto uColor = glGetUniformLocation(program, "color");

    // init physics
    physics::dynamics_world pworld;

    // create some collision shapes
    auto box_shape_id = pworld.add_collision_shape({.extents = {0.5, 0.5}});
    auto ground_shape_id = pworld.add_collision_shape({.extents = {10, 1}});
    auto table_shape_id = pworld.add_collision_shape({.extents = {1.5, 0.05}});
    auto table_leg_shape_id = pworld.add_collision_shape({.extents = {0.15, 0.5}});

    // 0 mass rigid bodies are static / kinematic: 1-way interaction with dynamic bodies
    auto player_id = pworld.add_rigid_body(1, {.position = {0, 0}, .angle = 0}, box_shape_id);

    // positive mass rigid bodies are dynamic
    pworld.add_rigid_body(1, {.position = {5, 0}}, box_shape_id);
    pworld.add_rigid_body(1, {.position = {5, 2.5}}, box_shape_id);
    pworld.add_rigid_body(1, {.position = {5, 5.0}}, box_shape_id);
    
    // adding collision object equivalent to 0-mass rigid body
    pworld.add_collision_object({.position = {0, -4}, .angle = 0.0}, ground_shape_id);

    // add a table
    pworld.add_rigid_body(1, {.position = {-1.4, -2.5}}, table_leg_shape_id);
    pworld.add_rigid_body(1, {.position = { 1.4, -2.5}}, table_leg_shape_id);
    pworld.add_rigid_body(1, {.position = { 0.0, -1.95}}, table_shape_id);

    // glUseProgram(program);
    program.use();

    glfwSwapInterval(1);

    auto time = glfwGetTimerValue();
    while (app.isRunning) {
        glfwPollEvents();

        if (glfwWindowShouldClose(app.window) || glfwGetKey(app.window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            app.isRunning = false;
        }

        int width, height;
        glfwGetWindowSize(app.window, &width, &height);

        auto lastTime = time;
        time = glfwGetTimerValue();
        float dt = (double) (time - lastTime) / glfwGetTimerFrequency(); 

        vvm::v2f move_dir(0, 0);
        if (glfwGetKey(app.window, GLFW_KEY_W)) {
            move_dir.y += 1;
        }
        if (glfwGetKey(app.window, GLFW_KEY_S)) {
            move_dir.y -= 1;
        }
        if (glfwGetKey(app.window, GLFW_KEY_A)) {
            move_dir.x -= 1;
        }
        if (glfwGetKey(app.window, GLFW_KEY_D)) {
            move_dir.x += 1;
        }
        if (vvm::dot(move_dir, move_dir) > 0)
            pworld.velocities[player_id].linear = vvm::normalize(move_dir) * 5.0f;
        else
            pworld.velocities[player_id].linear = {0, 0};

        pworld.step(1.0 / 60.0, 4);

        // for (int i = 0; i < n_bodies; ++i) {
        //     bodies[i].velocity += gravity * dt;
        //     bodies[i].position += bodies[i].velocity * dt;
        // }

        auto proj = vvm::ortho(0.1f, (float) width / height);
        // auto view = vvm::translate(vvm::v3f(-player.position, 0));

        auto viewProj = proj /* * view */;

        glViewport(0, 0, width, height);

        glClear(GL_COLOR_BUFFER_BIT);

        static const auto setUniforms = [&] (const vvm::m4f& m, const vvm::v3f color) {
            auto mvp = viewProj * m;
            // glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp.data);
            // glUniform3fv(uColor, 1, color.data);
            glUniformMatrix4fv(program.getUniformLocation("mvp"), 1, GL_FALSE, mvp.data);
            glUniform3fv(program.getUniformLocation("color"), 1, color.data);
            // program.setUniform("mvp", mvp);
            // program.setUniform("color", color);
        };

        static const auto drawQuad = [&] (const vvm::v2f& size, const vvm::v2f& pos, float angle, const vvm::v3f color) {
            auto model = vvm::translate(pos) * vvm::m4f(vvm::rotate(angle)) * vvm::scale(vvm::v3f(size, 0));
            setUniforms(model, color);
            glDrawArrays(GL_LINES, 0, 10);
        };

        static const auto drawSolidQuad = [&] (const vvm::v2f& size, const vvm::v2f& pos, float angle, const vvm::v3f color) {
            auto model = vvm::translate(pos) * vvm::m4f(vvm::rotate(angle)) * vvm::scale(vvm::v3f(size, 0));
            setUniforms(model, color);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        };

        static const auto drawLine = [&] (const vvm::v2f& p0, const vvm::v2f& p1, const vvm::v3f& color) {
            auto model = vvm::translate(p0) * vvm::m4f(vvm::m2f(p1 - p0, {0, 0})) * vvm::translate(vvm::v2f{0.5, 0.5});
            setUniforms(model, color);
            glDrawArrays(GL_LINES, 0, 2);
        };
        
        for (int i = 0; i < pworld.num_bodies; ++i) {
            const auto& t = pworld.transforms[i];
            const auto& s = pworld.collision_shapes[pworld.shape_ids[i]];
            drawQuad(2.0f * s.extents, t.position, t.angle, {1, 1, 1});
        }

        for (const auto& pair : pworld.pairs) {
            using fid = physics::collision_world::intersecting_pair::feature_id;
            
            const auto& t0 = pworld.transforms[pair.b0];
            const auto& t1 = pworld.transforms[pair.b1];
            const auto& s0 = pworld.collision_shapes[pworld.shape_ids[pair.b0]];
            const auto& s1 = pworld.collision_shapes[pworld.shape_ids[pair.b1]];
            
            vvm::v2f p1, p2;
            switch (pair.feature) {
            case fid::b0_pos_x:
            case fid::b0_neg_x:
                p1 = t0.position + pair.axis * s0.extents.x;
                p2 = p1 + pair.axis;
                break;
            case fid::b0_pos_y:
            case fid::b0_neg_y:
                p1 = t0.position + pair.axis * s0.extents.y;
                p2 = p1 + pair.axis;
                break;
            case fid::b1_pos_x:
            case fid::b1_neg_x:
                p1 = t1.position - pair.axis * s1.extents.x;
                p2 = p1 - pair.axis;
                break;
            case fid::b1_pos_y:
            case fid::b1_neg_y:
                p1 = t1.position - pair.axis * s1.extents.y;
                p2 = p1 - pair.axis;
                break;
            }

            auto color = pair.num_contacts > 0 ? vvm::v3f(1, 0, 0) : vvm::v3f(0, 1, 0);
            drawLine(p1, p2, color);

            for (int i = 0; i < pair.num_contacts; ++i) {
                auto p = pworld.contacts[pair.contact_ids[i]].position;
                drawSolidQuad({0.1, 0.1}, p, 0, {1, 0, 1});
            }
        }

        glfwSwapBuffers(app.window);
    }

    // glDeleteProgram(program);
}

int main() {
    App app;

    auto err = init(app);
    
    printError(err);

    run(app);

    cleanup(err, app);

    return 0;
}