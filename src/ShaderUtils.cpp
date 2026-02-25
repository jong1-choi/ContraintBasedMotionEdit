//
// ShaderUtils.cpp
// ConstraintBasedMotionEdit
//
// OpenGL shader loading, uniform helpers, and primitive mesh rendering.
//

#include "ShaderUtils.h"

#include <fstream>
#include <iostream>
#include <vector>

// ---- Platform helpers -------------------------------------------------------

#ifdef WIN32
std::wstring utf82Unicode(const std::string& s) {
    int len = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, nullptr, 0);
    std::wstring ws(len, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), -1, ws.data(), len);
    return ws;
}
#endif

// ---- Internal helpers -------------------------------------------------------

static void printShaderLog(GLuint obj) {
    int len = 0;
    glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &len);
    if (len <= 0) return;
    std::string log(len, '\0');
    glGetShaderInfoLog(obj, len, nullptr, log.data());
    std::cerr << "Shader Info:\n" << log << "\n";
}

static void printProgramLog(GLuint obj) {
    int len = 0;
    glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &len);
    if (len <= 0) return;
    std::string log(len, '\0');
    glGetProgramInfoLog(obj, len, nullptr, log.data());
    std::cerr << "Program Info:\n" << log << "\n";
}

static std::string readText(const std::string& fn) {
    std::ifstream t(utf82Unicode(fn));
    if (!t.is_open()) {
        std::cerr << "[ERROR] File not found: " << fn << "\n";
        return "";
    }
    return std::string(std::istreambuf_iterator<char>(t),
                       std::istreambuf_iterator<char>());
}

// ---- Shader loading ---------------------------------------------------------

GLuint loadShader(const std::string& fn, GLuint shaderType) {
    std::string code = readText(fn);
    GLuint shader = glCreateShader(shaderType);
    const char* src = code.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    printShaderLog(shader);
    return shader;
}

GLuint buildProgram(GLuint vert, GLuint frag) {
    GLuint prog = glCreateProgram();
    glAttachShader(prog, vert);
    glAttachShader(prog, frag);
    glLinkProgram(prog);
    glUseProgram(prog);
    printProgramLog(prog);
    return prog;
}

std::tuple<GLuint, GLuint, GLuint> loadProgram(const std::string& vertFn,
                                                const std::string& fragFn) {
    GLuint vert = loadShader(vertFn, GL_VERTEX_SHADER);
    GLuint frag = loadShader(fragFn, GL_FRAGMENT_SHADER);
    GLuint prog = buildProgram(vert, frag);
    return { prog, vert, frag };
}

// ---- Uniform setters --------------------------------------------------------

void setUniform(GLuint prog, const std::string& n, int v)              { glUniform1i(glGetUniformLocation(prog, n.c_str()), v); }
void setUniform(GLuint prog, const std::string& n, float v)            { glUniform1f(glGetUniformLocation(prog, n.c_str()), v); }
void setUniform(GLuint prog, const std::string& n, const glm::ivec2& v){ glUniform2iv(glGetUniformLocation(prog, n.c_str()), 1, glm::value_ptr(v)); }
void setUniform(GLuint prog, const std::string& n, const glm::ivec3& v){ glUniform3iv(glGetUniformLocation(prog, n.c_str()), 1, glm::value_ptr(v)); }
void setUniform(GLuint prog, const std::string& n, const glm::vec2& v) { glUniform2fv(glGetUniformLocation(prog, n.c_str()), 1, glm::value_ptr(v)); }
void setUniform(GLuint prog, const std::string& n, const glm::vec3& v) { glUniform3fv(glGetUniformLocation(prog, n.c_str()), 1, glm::value_ptr(v)); }
void setUniform(GLuint prog, const std::string& n, const glm::vec4& v) { glUniform4fv(glGetUniformLocation(prog, n.c_str()), 1, glm::value_ptr(v)); }
void setUniform(GLuint prog, const std::string& n, const glm::mat3& v) { glUniformMatrix3fv(glGetUniformLocation(prog, n.c_str()), 1, 0, glm::value_ptr(v)); }
void setUniform(GLuint prog, const std::string& n, const glm::mat4& v) { glUniformMatrix4fv(glGetUniformLocation(prog, n.c_str()), 1, 0, glm::value_ptr(v)); }
void setUniform(GLuint prog, const std::string& n, const glm::vec3* v, int c) { glUniform3fv(glGetUniformLocation(prog, n.c_str()), c, (const GLfloat*)v); }

// ---- Renderable mesh helper -------------------------------------------------

struct RenderableMesh {
    GLuint va    = 0;
    GLuint vBuf  = 0;
    GLuint nBuf  = 0;
    GLuint eBuf  = 0;
    unsigned int nFaces = 0;

    void create(const std::vector<glm::vec3>& verts,
                const std::vector<glm::vec3>& norms,
                const std::vector<glm::uvec3>& faces) {
        if (va) return;
        glGenVertexArrays(1, &va);
        glBindVertexArray(va);

        glGenBuffers(1, &vBuf);
        glBindBuffer(GL_ARRAY_BUFFER, vBuf);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * verts.size(), verts.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glGenBuffers(1, &nBuf);
        glBindBuffer(GL_ARRAY_BUFFER, nBuf);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * norms.size(), norms.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_TRUE, 0, nullptr);

        glGenBuffers(1, &eBuf);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eBuf);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(glm::uvec3) * faces.size(), faces.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        nFaces = (unsigned int)faces.size() * 3;
    }

    void render() {
        glBindVertexArray(va);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eBuf);
        glDrawElements(GL_TRIANGLES, nFaces, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
};

// ---- Unit primitive meshes --------------------------------------------------

static constexpr int   k_nStrip = 15;
static constexpr int   k_nSlice = 30;
static constexpr float k_pi     = 3.14159265f;

void drawQuad() {
    static RenderableMesh mesh;
    if (!mesh.va) {
        const std::vector<glm::vec3> v = { {-1,1,0},{-1,-1,0},{1,1,0},{1,-1,0} };
        const std::vector<glm::vec3> n = { {0,0,1},{0,0,1},{0,0,1},{0,0,1} };
        const std::vector<glm::uvec3> e = { {0,1,2},{2,1,3} };
        mesh.create(v, n, e);
    }
    mesh.render();
}

void drawSphere() {
    static RenderableMesh mesh;
    if (!mesh.va) {
        std::vector<glm::vec3> v;
        std::vector<glm::uvec3> e;
        v.push_back({ 0, 1, 0 });
        for (int s = 1; s < k_nStrip; s++) {
            float y = cosf(s * k_pi / k_nStrip);
            float r = sinf(s * k_pi / k_nStrip);
            for (int l = 0; l < k_nSlice; l++)
                v.push_back({ sinf(l * k_pi * 2 / k_nSlice) * r, y, cosf(l * k_pi * 2 / k_nSlice) * r });
        }
        v.push_back({ 0, -1, 0 });
        // Top cap
        for (int l = 0; l < k_nSlice; l++)
            e.push_back({ 0u, (unsigned)(l + 1), (unsigned)((l + 1) % k_nSlice + 1) });
        // Strips
        for (int s = 1; s < k_nStrip; s++) {
            int s0 = (s - 1) * k_nSlice + 1, s1 = s * k_nSlice + 1;
            for (int l = 0; l < k_nSlice; l++) {
                e.push_back({ (unsigned)(l + s0), (unsigned)(l + s1), (unsigned)((l + 1) % k_nSlice + s0) });
                e.push_back({ (unsigned)((l + 1) % k_nSlice + s0), (unsigned)(l + s1), (unsigned)((l + 1) % k_nSlice + s1) });
            }
        }
        // Bottom cap
        int s0 = (k_nStrip - 2) * k_nSlice + 1, s1 = (k_nStrip - 1) * k_nSlice + 1;
        for (int l = 0; l < k_nSlice; l++)
            e.push_back({ (unsigned)s1, (unsigned)((l + 1) % k_nSlice + s0), (unsigned)(l + s0) });
        mesh.create(v, v, e);
    }
    mesh.render();
}

void drawCylinder() {
    static RenderableMesh mesh;
    if (!mesh.va) {
        std::vector<glm::vec3> v, n;
        std::vector<glm::uvec3> e;
        // Top cap center
        v.push_back({ 0, .5f, 0 }); n.push_back({ 0, 1, 0 });
        for (int l = 0; l < k_nSlice; l++) { v.push_back({ sinf(l * k_pi * 2 / k_nSlice), .5f, cosf(l * k_pi * 2 / k_nSlice) }); n.push_back({ 0, 1, 0 }); }
        // Side top
        for (int l = 0; l < k_nSlice; l++) { glm::vec2 p = { sinf(l * k_pi * 2 / k_nSlice), cosf(l * k_pi * 2 / k_nSlice) }; v.push_back({ p.x, .5f, p.y }); n.push_back({ p.x, 0, p.y }); }
        // Side bottom
        for (int l = 0; l < k_nSlice; l++) { glm::vec2 p = { sinf(l * k_pi * 2 / k_nSlice), cosf(l * k_pi * 2 / k_nSlice) }; v.push_back({ p.x, -.5f, p.y }); n.push_back({ p.x, 0, p.y }); }
        // Bottom cap ring
        for (int l = 0; l < k_nSlice; l++) { v.push_back({ sinf(l * k_pi * 2 / k_nSlice), -.5f, cosf(l * k_pi * 2 / k_nSlice) }); n.push_back({ 0, -1, 0 }); }
        // Bottom cap center
        v.push_back({ 0, -.5f, 0 }); n.push_back({ 0, -1, 0 });
        // Top cap faces
        for (int l = 0; l < k_nSlice; l++) e.push_back({ 0u, (unsigned)(l + 1), (unsigned)((l + 1) % k_nSlice + 1) });
        // Side faces
        int s0 = k_nSlice + 1, s1 = k_nSlice * 2 + 1;
        for (int l = 0; l < k_nSlice; l++) {
            e.push_back({ (unsigned)(l + s0), (unsigned)(l + s1), (unsigned)((l + 1) % k_nSlice + s0) });
            e.push_back({ (unsigned)((l + 1) % k_nSlice + s0), (unsigned)(l + s1), (unsigned)((l + 1) % k_nSlice + s1) });
        }
        // Bottom cap faces
        s0 = k_nSlice * 3 + 1; s1 = k_nSlice * 4 + 1;
        for (int l = 0; l < k_nSlice; l++) e.push_back({ (unsigned)s1, (unsigned)((l + 1) % k_nSlice + s0), (unsigned)(l + s0) });
        mesh.create(v, n, e);
    }
    mesh.render();
}

// ---- High-level drawing helpers --------------------------------------------

static GLuint getCurProgram() {
    GLint prog = 0;
    glGetIntegerv(GL_CURRENT_PROGRAM, &prog);
    return (GLuint)prog;
}

void drawQuad(const glm::vec3& p, const glm::vec3& n, const glm::vec2& sz, const glm::vec4 color) {
    glm::vec3 axis  = glm::cross(n, glm::vec3(0, 0, 1));
    float     len   = glm::length(axis);
    float     angle = atan2f(len, n.z);
    glm::mat4 m = len > 1e-7f
        ? glm::translate(glm::mat4(1), p) * glm::rotate(glm::mat4(1), angle, axis) * glm::scale(glm::mat4(1), glm::vec3(sz.x, sz.y, 1))
        : glm::translate(glm::mat4(1), p)                                           * glm::scale(glm::mat4(1), glm::vec3(sz.x, sz.y, 1));
    GLuint prog = getCurProgram();
    setUniform(prog, "modelMat", m);
    setUniform(prog, "color", color);
    drawQuad();
}

void drawSphere(const glm::vec3& p, float r, const glm::vec4 color) {
    GLuint prog = getCurProgram();
    setUniform(prog, "modelMat", glm::translate(glm::mat4(1), p) * glm::scale(glm::mat4(1), glm::vec3(r)));
    setUniform(prog, "color", color);
    drawSphere();
}

void drawCylinder(const glm::vec3& p1, const glm::vec3& p2, float r, const glm::vec4 color) {
    glm::vec3 axis  = glm::cross(glm::vec3(0, 1, 0), p1 - p2);
    float     len   = glm::length(axis);
    float     angle = atan2f(len, (p1 - p2).y);
    glm::vec3 s     = { r, glm::length(p1 - p2), r };
    glm::mat4 m = len > 1e-7f
        ? glm::translate(glm::mat4(1), (p1 + p2) * .5f) * glm::rotate(glm::mat4(1), angle, axis) * glm::scale(glm::mat4(1), s)
        : glm::translate(glm::mat4(1), (p1 + p2) * .5f)                                           * glm::scale(glm::mat4(1), s);
    GLuint prog = getCurProgram();
    setUniform(prog, "modelMat", m);
    setUniform(prog, "color", color);
    drawCylinder();
}
