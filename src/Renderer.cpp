//
// Renderer.cpp
// ConstraintBasedMotionEdit
//
// Implements shadow-mapped 3D rendering and camera unprojection.
//

#include "Renderer.h"
#include "ShaderUtils.h"

#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

// ---------------------------------------------------------------------------
// FB
// ---------------------------------------------------------------------------

void FB::setTexParam(GLuint minFilter, GLuint wrap) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap);
}

void FB::clearGL() {
    if (color) { glDeleteTextures(1, &color);     color = 0; }
    if (depth) { glDeleteTextures(1, &depth);     depth = 0; }
    if (fbo)   { glDeleteFramebuffers(1, &fbo);   fbo   = 0; }
}

void FB::create(int width, int height) {
    if (width == w && height == h) return;
    w = width; h = height;
    clearGL();

    glGenTextures(1, &color);
    glBindTexture(GL_TEXTURE_2D, color);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    setTexParam();

    glGenTextures(1, &depth);
    glBindTexture(GL_TEXTURE_2D, depth);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    setTexParam();

    glGenFramebuffers(1, &fbo);
    GLint savedFB = 0;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &savedFB);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color, 0);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,  depth, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cerr << "[FB] Framebuffer incomplete\n";
    glBindFramebuffer(GL_FRAMEBUFFER, savedFB);
}

void FB::setToTarget() {
    glGetIntegerv(GL_VIEWPORT, m_savedVP);
    m_savedSc = glIsEnabled(GL_SCISSOR_TEST);
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &m_savedFB);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
    glViewport(0, 0, w, h);
    glDisable(GL_SCISSOR_TEST);
}

void FB::restoreVP() {
    glViewport(m_savedVP[0], m_savedVP[1], m_savedVP[2], m_savedVP[3]);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_savedFB);
    if (m_savedSc) glEnable(GL_SCISSOR_TEST);
}

void FB::bindColor(GLuint prog, const std::string& name, GLuint slot) const {
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, color);
    setUniform(prog, name, (int)slot);
}

void FB::bindDepth(GLuint prog, const std::string& name, GLuint slot) const {
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, depth);
    setUniform(prog, name, (int)slot);
}

// ---------------------------------------------------------------------------
// Renderer
// ---------------------------------------------------------------------------

Renderer::Renderer() {
    // IBL coefficients (uniform hemisphere lighting)
    m_iblCoeffs[0] = { 2.136610f,  2.136610f,  2.136610f };
    m_iblCoeffs[1] = { 1.200665f,  1.200665f,  1.200665f };
    m_iblCoeffs[2] = {-0.319614f, -0.319614f, -0.319614f };
    m_iblCoeffs[3] = {-0.718034f, -0.718034f, -0.718034f };
    m_iblCoeffs[4] = {-0.101370f, -0.101370f, -0.101370f };
    m_iblCoeffs[5] = {-0.183365f, -0.183365f, -0.183365f };
    m_iblCoeffs[6] = {-0.191558f, -0.191558f, -0.191558f };
    m_iblCoeffs[7] = { 0.387255f,  0.387255f,  0.387255f };
    m_iblCoeffs[8] = {-0.021659f, -0.021659f, -0.021659f };
}

glm::mat4 Renderer::getViewMat() const {
    return glm::translate(glm::mat4(1), glm::vec3(0, 0, -m_dist))
         * glm::rotate(glm::mat4(1), m_pitch, glm::vec3(1, 0, 0))
         * glm::rotate(glm::mat4(1), m_yaw,   glm::vec3(0, 1, 0))
         * glm::translate(glm::mat4(1), -m_sceneCenter);
}

glm::mat4 Renderer::getProjMat() const {
    return glm::perspective(m_fov,
                            (float)m_width / (float)m_height,
                            10.f, 1000.f);
}

void Renderer::setViewProj(GLuint prog) const {
    setUniform(prog, "projMat", getProjMat());
    setUniform(prog, "viewMat", getViewMat());
}

void Renderer::ensureShaders() {
    if (m_renderProg != 0) return;
    std::tie(m_renderProg, m_renderVert, m_renderFrag) = loadProgram("Res/shader.vert", "Res/shader.frag");
    std::tie(m_constProg,  m_constVert,  m_constFrag)  = loadProgram("Res/const.vert",  "Res/const.frag");
}

void Renderer::drawGL() {
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ensureShaders();

    glm::mat4 shadowV, shadowP;
    constexpr float k_shadowZNear = 100.f;
    constexpr float k_shadowZFar  = 10000.f;
    constexpr float k_shadowFov   = 1.0f;

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    // Shadow pass
    if (m_enableShadow) {
        m_shadowMap.create(1024, 1024);
        m_shadowMap.setToTarget();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        shadowV = glm::lookAt(m_lightPos, m_sceneCenter, glm::vec3(0, 1, 0));
        shadowP = glm::perspective(k_shadowFov, 1.f, k_shadowZNear, k_shadowZFar);
        glUseProgram(m_constProg);
        setUniform(m_constProg, "modelMat", glm::mat4(1));
        setUniform(m_constProg, "projMat",  shadowP);
        setUniform(m_constProg, "viewMat",  shadowV);
        renderFunction();
        m_shadowMap.restoreVP();
    }

    // Main render pass
    glUseProgram(m_renderProg);
    setUniform(m_renderProg, "color",    glm::vec4(.8f, .8f, .8f, 1.f));
    setUniform(m_renderProg, "modelMat", glm::mat4(1));
    setViewProj(m_renderProg);
    setUniform(m_renderProg, "lightPos",   m_lightPos);
    setUniform(m_renderProg, "iblCoeffs",  m_iblCoeffs, 9);

    if (m_enableShadow) {
        setUniform(m_renderProg, "shadowEnabled", 1);
        m_shadowMap.bindDepth(m_renderProg, "shadowMap", 0);
        setUniform(m_renderProg, "shadowProj",    shadowP);
        setUniform(m_renderProg, "shadowZNear",   k_shadowZNear);
        setUniform(m_renderProg, "shadowZFar",    k_shadowZFar);
        setUniform(m_renderProg, "lightDir",      glm::normalize(m_sceneCenter - m_lightPos));
        setUniform(m_renderProg, "cosLightFov",   cosf(k_shadowFov / 2.f));
        setUniform(m_renderProg, "shadowBiasedVP",
                   glm::translate(glm::mat4(1), glm::vec3(.5f)) * glm::scale(glm::mat4(1), glm::vec3(.5f)) * shadowP * shadowV);
    }
    else {
        setUniform(m_renderProg, "shadowEnabled", 0);
    }
    renderFunction();

    // Wireframe pass
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glLineWidth(.2f);
    glUseProgram(m_constProg);
    setUniform(m_constProg, "color",    glm::vec4(0, 0, 0, .2f));
    setUniform(m_constProg, "modelMat", glm::mat4(1));
    setViewProj(m_constProg);
    wireFunction();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

std::pair<glm::vec3, float> Renderer::unproject(const glm::vec2& pt,
                                                  float pixelRatio) const {
    float d = 0.f;
    // Read depth at the clicked pixel
    glReadPixels((GLint)(pt.x * pixelRatio),
                 (GLint)((m_height - pt.y) * pixelRatio),
                 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &d);
    return { unprojectAtDepth(pt, d, pixelRatio), d };
}

glm::vec3 Renderer::unprojectAtDepth(const glm::vec2& pt, float d,
                                      float /*pixelRatio*/) const {
    glm::vec3 ndc = glm::vec3(pt / glm::vec2(m_width, m_height), d) * 2.f - glm::vec3(1.f);
    ndc.y = -ndc.y;
    glm::vec4 world = glm::inverse(getProjMat() * getViewMat()) * glm::vec4(ndc, 1.f);
    return glm::vec3(world) / world.w;
}
