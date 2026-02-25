//
// Renderer.h
// ConstraintBasedMotionEdit
//
// 3D camera, shadow-mapped rendering, and depth-based unprojection.
// Replaces the JGL-based ModelView / AnimView hierarchy.
//

#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <functional>
#include <string>

// ---------------------------------------------------------------------------
// FB  — OpenGL framebuffer helper
// ---------------------------------------------------------------------------
struct FB {
    int    w = 0, h = 0;
    GLuint fbo   = 0;
    GLuint color = 0;
    GLuint depth = 0;

    void create(int width, int height);
    void clearGL();

    void setToTarget();
    void restoreVP();

    void bindColor(GLuint prog, const std::string& name, GLuint slot) const;
    void bindDepth(GLuint prog, const std::string& name, GLuint slot) const;

private:
    GLint m_savedVP[4] = {};
    GLint m_savedFB    = 0;
    GLint m_savedSc    = 0;

    static void setTexParam(GLuint minFilter = GL_LINEAR,
                            GLuint wrap      = GL_CLAMP_TO_EDGE);
};

// ---------------------------------------------------------------------------
// Renderer  — camera state + shadow-mapped 3D rendering
// ---------------------------------------------------------------------------
struct Renderer {
    // Camera
    float     m_dist        = 250.f;
    float     m_yaw         = 0.f;
    float     m_pitch       = 0.3f;
    float     m_fov         = 0.8f;
    glm::vec3 m_sceneCenter = {0, 0, 0};

    // Lighting / IBL
    glm::vec3 m_lightPos    = {0, 200, 250};
    bool      m_enableShadow = true;
    glm::vec3 m_iblCoeffs[9];

    // Window dimensions (must be kept in sync with actual GLFW window)
    int m_width  = 800;
    int m_height = 600;

    // Callbacks set by main.cpp
    std::function<void()> renderFunction = [](){};
    std::function<void()> wireFunction   = [](){};

    Renderer();

    // Call once per frame from the main loop
    void drawGL();

    // Camera matrices
    glm::mat4 getViewMat() const;
    glm::mat4 getProjMat() const;

    // Unproject screen point to world (reads depth from framebuffer)
    std::pair<glm::vec3, float> unproject(const glm::vec2& screenPt,
                                           float pixelRatio = 1.f) const;
    // Unproject with known depth value
    glm::vec3 unprojectAtDepth(const glm::vec2& screenPt, float d,
                                float pixelRatio = 1.f) const;

private:
    GLuint m_renderProg = 0, m_renderVert = 0, m_renderFrag = 0;
    GLuint m_constProg  = 0, m_constVert  = 0, m_constFrag  = 0;
    FB     m_shadowMap;

    void setViewProj(GLuint prog) const;
    void ensureShaders();
};
