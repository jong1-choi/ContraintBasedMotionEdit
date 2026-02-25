//
// ShaderUtils.h
// ConstraintBasedMotionEdit
//
// OpenGL shader loading and primitive drawing utilities.
//

#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <tuple>

#ifdef WIN32
#include <windows.h>
std::wstring utf82Unicode(const std::string& s);
#else
inline std::string utf82Unicode(const std::string& s) { return s; }
#endif

// --- Shader loading ---
GLuint loadShader(const std::string& filename, GLuint shaderType);
GLuint buildProgram(GLuint vertShader, GLuint fragShader);
std::tuple<GLuint, GLuint, GLuint> loadProgram(const std::string& vertFn, const std::string& fragFn);

// --- Uniform setters ---
void setUniform(GLuint prog, const std::string& name, int v);
void setUniform(GLuint prog, const std::string& name, float v);
void setUniform(GLuint prog, const std::string& name, const glm::ivec2& v);
void setUniform(GLuint prog, const std::string& name, const glm::ivec3& v);
void setUniform(GLuint prog, const std::string& name, const glm::vec2& v);
void setUniform(GLuint prog, const std::string& name, const glm::vec3& v);
void setUniform(GLuint prog, const std::string& name, const glm::vec4& v);
void setUniform(GLuint prog, const std::string& name, const glm::mat3& v);
void setUniform(GLuint prog, const std::string& name, const glm::mat4& v);
void setUniform(GLuint prog, const std::string& name, const glm::vec3* v, int n);

// --- Primitive drawing ---
// Low-level (unit shapes, uses current program's modelMat/color uniforms)
void drawQuad();
void drawSphere();
void drawCylinder();

// High-level (sets transform and color automatically)
void drawQuad(const glm::vec3& p, const glm::vec3& n, const glm::vec2& sz,
              const glm::vec4 color = glm::vec4(0, 0, .4f, 1));
void drawSphere(const glm::vec3& p, float r,
                const glm::vec4 color = glm::vec4(1, .4f, 0, 1));
void drawCylinder(const glm::vec3& p1, const glm::vec3& p2, float r,
                  const glm::vec4 color = glm::vec4(1, 0, 0, 1));
