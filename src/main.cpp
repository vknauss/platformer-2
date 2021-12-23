#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <GL/glew.h>
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

// physics

// struct body {
//     vvm::v2f position;
//     vvm::v2f velocity;
//     vvm::v2f size;
//     bool dynamic;
// };


// struct physics_world {
//     packed_array<body, 256, MAX_ENTITIES> bodies;
//     vvm::v2f gravity;

//     struct overlapping_pair_entry {
//         unsigned int b0, b1;
//     };
//     std::vector<overlapping_pair_entry> overlappingPairs;

//     physics_world() : gravity(0, -9.8f) { }

//     void calcOverlappingPairs() {
//         std::vector<unsigned int> indices(bodies.numElements);
//         std::iota(indices.begin(), indices.end(), 0);
//         std::sort(indices.begin(), indices.end(), [this] (auto i0, auto i1) {
//             return bodies.data[i0].position.x < bodies.data[i1].position.x;
//         });

//         overlappingPairs.clear();
        
//         std::vector<unsigned int> activeIntervals;
//         for (auto i = 0u; i < indices.size(); ++i) {
//             auto i0 = indices[i];
//             float w = bodies.data[i0].position.x;
//             auto numActiveIntervals = 0u;
//             for (auto j = 0u; j < activeIntervals.size(); ++j) {
//                 auto i1 = activeIntervals[j];
//                 if (w > bodies.data[i1].position.x + bodies.data[i1].size.x) {
                    
//                     activeIntervals[numActiveIntervals++] = activeIntervals[j];
//                 }
//             }
//         }

        
//         auto numOverlappingPairs = overlappingPairs.size();




//         for (int i = overlappingPairs.size() - 1; i >= 0; --i) {
//             auto& p = overlappingPairs[i];
//             const auto& b0 = bodies.data[p.b0];
//             const auto& b1 = bodies.data[p.b1];
//             if (b0.position.x > b1.position.x + b1.size.x ||
//                 b0.position.x + b0.size.x < b1.position.x ||
//                 b0.position.y > b1.position.y + b1.size.y ||
//                 b0.position.y + b0.size.y < b1.position.y)
//             {
//                 --numOverlappingPairs;
//                 overlappingPairs[i] = overlappingPairs[numOverlappingPairs];
//             }
//         }

//     }

//     void tick(float dt) {
//         for (auto i = 0u; i < dynamicBodies.numElements; ++i) {
//             auto& b = dynamicBodies.data[i];
//             b.velocity += gravity * dt;
//             b.position += b.velocity * dt;
//         }
//     }

// };


void run(App& app) {

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint program = createShaderProgram("vert.glsl", "frag.glsl");

    auto uMVP = glGetUniformLocation(program, "mvp");
    auto uColor = glGetUniformLocation(program, "color");

    physics::collision_world collision_world;

    collision_world.collision_shapes.push_back({.extents = {1, 1}});

    collision_world.transforms.push_back({.position = {0, 0}, .angle = 0});
    collision_world.shape_ids.push_back(0);
    collision_world.num_bodies = 1;

    collision_world.transforms.push_back({.position = {5, 0}, .angle = 1.2});
    collision_world.shape_ids.push_back(0);
    ++collision_world.num_bodies;

    glUseProgram(program);

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
            collision_world.transforms[0].position += vvm::normalize(move_dir) * dt * 5.0f;

        collision_world.find_intersecting_pairs();
        collision_world.find_contacts();

        // for (int i = 0; i < n_bodies; ++i) {
        //     bodies[i].velocity += gravity * dt;
        //     bodies[i].position += bodies[i].velocity * dt;
        // }

        auto proj = vvm::ortho(0.1f, (float) width / height);
        // auto view = vvm::translate(vvm::v3f(-player.position, 0));

        auto viewProj = proj /* * view */;

        glViewport(0, 0, width, height);

        glClear(GL_COLOR_BUFFER_BIT);
        
        glUniform3fv(uColor, 1, vvm::v3f(1).data);
        for (int i = 0; i < collision_world.num_bodies; ++i) {
            const auto& t = collision_world.transforms[i];
            const auto& s = collision_world.collision_shapes[collision_world.shape_ids[i]];
            auto model = vvm::translate(vvm::v3f(t.position, 0)) *
                vvm::m4f(vvm::rotate(t.angle)) *
                vvm::scale(vvm::v3f(collision_world.collision_shapes[collision_world.shape_ids[i]].extents * 2.0f, 0));
            auto mvp = viewProj * model;
            glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp.data);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        }
        
        for (const auto& pair : collision_world.pairs) {
            using fid = physics::collision_world::intersecting_pair::feature_id;
            vvm::v2f p;
            const auto& t0 = collision_world.transforms[pair.b0];
            const auto& t1 = collision_world.transforms[pair.b1];
            const auto& s0 = collision_world.collision_shapes[collision_world.shape_ids[pair.b0]];
            const auto& s1 = collision_world.collision_shapes[collision_world.shape_ids[pair.b1]];
            switch (pair.feature) {
            case fid::b0_pos_x:
                p = t0.position + vvm::rotate(t0.angle) * vvm::v2f(s0.extents.x + 0.5, 0);
                break;
            case fid::b0_neg_x:
                p = t0.position + vvm::rotate(t0.angle) * vvm::v2f(-s0.extents.x - 0.5, 0);
                break;
            case fid::b0_pos_y:
                p = t0.position + vvm::rotate(t0.angle) * vvm::v2f(0, s0.extents.y + 0.5);
                break;
            case fid::b0_neg_y:
                p = t0.position + vvm::rotate(t0.angle) * vvm::v2f(0, -s0.extents.y - 0.5);
                break;
            case fid::b1_pos_x:
                p = t1.position + vvm::rotate(t1.angle) * vvm::v2f(s1.extents.x + 0.5, 0);
                break;
            case fid::b1_neg_x:
                p = t1.position + vvm::rotate(t1.angle) * vvm::v2f(-s1.extents.x - 0.5, 0);
                break;
            case fid::b1_pos_y:
                p = t1.position + vvm::rotate(t1.angle) * vvm::v2f(0, s1.extents.y + 0.5);
                break;
            case fid::b1_neg_y:
                p = t1.position + vvm::rotate(t1.angle) * vvm::v2f(0, -s1.extents.y - 0.5);
                break;
            }
            auto model = vvm::translate(vvm::v3f(p, 0)) *
                vvm::m4f(vvm::m2f({pair.axis.y, -pair.axis.x}, pair.axis)) *
                vvm::scale(vvm::v3f{0.1, 1, 0});
            auto mvp = viewProj * model;
            glUniformMatrix4fv(uMVP, 1, GL_FALSE, mvp.data);
            if (pair.num_contacts > 0) {    
                glUniform3fv(uColor, 1, vvm::v3f(1, 0, 0).data);
            } else {
                glUniform3fv(uColor, 1, vvm::v3f(0, 1, 0).data);
            }
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        }

        glfwSwapBuffers(app.window);
    }

    glDeleteProgram(program);
}

int main() {
    App app;

    auto err = init(app);
    
    printError(err);

    run(app);

    cleanup(err, app);

    return 0;
}