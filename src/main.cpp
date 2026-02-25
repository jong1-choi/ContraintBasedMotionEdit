//
// main.cpp
// ConstraintBasedMotionEdit
//
// GLFW window, input callbacks, animation loop, and motion editing entry point.
// Implements constraint-based motion retargeting via IK + cubic B-spline fitting.
//
// Reference: "Retargetting Motion to New Characters" — Gleicher et al.
//            Cubic uniform B-spline: knot interval = 5 frames
//

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "IK.h"
#include "BVH.h"
#include "Renderer.h"
#include "ShaderUtils.h"

#include <iostream>
#include <vector>
#include <cmath>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr int   WINDOW_W      = 800;
static constexpr int   WINDOW_H      = 600;
static constexpr int   k_jointCount  = 31;   // number of joints to hit-test
static constexpr float k_pickRadius  = 1.5f; // world-space picking radius

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
static Renderer         g_renderer;
static BVH*             g_bvh      = nullptr;
static std::vector<Body> g_newBody;
static std::vector<Body> g_oldBody;

static int   g_totalFrame = 0;
static int   g_frameNum   = 0;
static float g_frameTime  = 0.f;
static bool  g_animating  = false;
static float g_lastTime   = 0.f;

static int       g_picked   = -1;
static glm::vec2 g_oldPt2;
static glm::vec3 g_oldPt3;
static glm::vec3 g_pickPt;
static glm::vec3 g_targetPt;
static float     g_oldDepth = 0.f;

// ---------------------------------------------------------------------------
// Simulation helpers
// ---------------------------------------------------------------------------
static void loadBVH(const std::string& path) {
    g_newBody.clear();
    g_oldBody.clear();
    g_frameNum  = 0;
    g_frameTime = 0.f;
    g_bvh->Clear();
    g_bvh->Load(path.c_str());

    g_totalFrame = g_bvh->num_frame;
    Body temp;
    for (int i = 0; i < g_totalFrame; i++) {
        temp.clear();
        g_bvh->UpdatePose(i, temp, 5);
        g_newBody.push_back(temp);
        g_oldBody.push_back(temp);

        g_newBody[i].getDisplacement(g_newBody[i], g_newBody[i]);
        g_oldBody[i].getDisplacement(g_oldBody[i], g_oldBody[i]);

        g_oldBody[i].updatePos(0);
        g_newBody[i].updatePos(0);
    }
}

static void init() {
    // Default BVH path — drag-and-drop a .bvh file to change it
    loadBVH("BVH/WalkStartA.bvh");
}

static void frame(float dt) {
    g_frameTime += dt;
    if (g_frameTime > 0.03f) {
        g_frameNum  = (g_frameNum + 1) % g_bvh->num_frame;
        g_frameTime = 0.f;
        g_newBody[g_frameNum].updatePos(0);
    }
}

// Cubic uniform B-spline constraint-based motion editing.
// For each joint, fits a B-spline through the constrained displacement frames,
// then applies the curve to all frames to produce smooth motion.
static void motionEdit() {
    std::vector<int> cons;
    for (int i = 0; i < g_totalFrame; i++) {
        if (g_newBody[i].constraint) {
            cons.push_back(i);
            g_newBody[i].constraint = false;
        }
    }
    if (cons.empty()) return;

    const int   space    = 5;
    const int   controlN = g_totalFrame / space + 1;

    for (int joint = 1; joint < (int)g_bvh->joints.size() + 1; joint++) {
        Eigen::MatrixXf basis = Eigen::MatrixXf::Zero(controlN, (int)cons.size());
        Eigen::MatrixXf p     = Eigen::MatrixXf::Zero(3, (int)cons.size());

        for (int j = 0; j < (int)cons.size(); j++) {
            int   f = cons[j];
            float t = (f % space) / (float)space;
            int   k = f / space;

            // Skip boundary cases where B-spline stencil is incomplete
            if (k < 1 || k > controlN - 4) continue;

            // Cubic uniform B-spline basis (de Boor)
            basis(k - 1, j) = (1.f/6.f) * (1-t)*(1-t)*(1-t);
            basis(k,     j) = (1.f/6.f) * (3*t*t*t - 6*t*t + 4);
            basis(k + 1, j) = (1.f/6.f) * (-3*t*t*t + 3*t*t + 3*t + 1);
            basis(k + 2, j) = (1.f/6.f) * t*t*t;

            p(0, j) = g_newBody[f].displacement(joint, 0);
            p(1, j) = g_newBody[f].displacement(joint, 1);
            p(2, j) = g_newBody[f].displacement(joint, 2);
        }

        // Solve for B-spline control points via SVD pseudo-inverse: b = p * B^+
        auto solver = basis.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        solver.setThreshold(0.01f);
        Eigen::MatrixXf b = p * solver.solve(Eigen::MatrixXf::Identity(controlN, controlN));

        // Apply B-spline curve to all frames
        for (int f = 0; f < g_totalFrame; f++) {
            float t = (f % space) / (float)space;
            int   k = f / space;
            if (k < 1 || k > controlN - 3) continue;

            glm::vec3 bspline =
                glm::vec3(b(0, k-1), b(1, k-1), b(2, k-1)) * (1.f/6.f) * (1-t)*(1-t)*(1-t) +
                glm::vec3(b(0, k  ), b(1, k  ), b(2, k  )) * (1.f/6.f) * (3*t*t*t - 6*t*t + 4) +
                glm::vec3(b(0, k+1), b(1, k+1), b(2, k+1)) * (1.f/6.f) * (-3*t*t*t + 3*t*t + 3*t + 1) +
                glm::vec3(b(0, k+2), b(1, k+2), b(2, k+2)) * (1.f/6.f) * t*t*t;

            glm::quat dq = glm::quat(1.f, bspline.x, bspline.y, bspline.z);
            g_newBody[f].links[joint - 1].q = g_oldBody[f].links[joint - 1].q * dq;
        }
    }

    for (int f = 0; f < g_totalFrame; f++)
        g_newBody[f].updatePos(0);

    std::cout << "[motionEdit] Done. " << cons.size() << " constraint(s) applied.\n";
}

// ---------------------------------------------------------------------------
// Render callback (called from Renderer::drawGL)
// ---------------------------------------------------------------------------
static void renderScene() {
    if (!g_bvh || g_bvh->joints.empty()) return;

    g_newBody[g_frameNum].render();
    for (int i = 0; i < g_bvh->num_frame; i++)
        g_newBody[i].shapeRender();

    if (g_picked >= 0)
        drawSphere(g_targetPt, 1.5f, glm::vec4(1, 1, 0, .1f));

    drawQuad(glm::vec3(0), glm::vec3(0, 1, 0), glm::vec2(2000));
}

// ---------------------------------------------------------------------------
// GLFW callbacks
// ---------------------------------------------------------------------------
static void onMouseButton(GLFWwindow*, int button, int action, int) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;

    double mx, my;
    glfwGetCursorPos(glfwGetCurrentContext(), &mx, &my);
    glm::vec2 pt2((float)mx, (float)my);

    if (action == GLFW_PRESS) {
        g_oldPt2 = pt2;
        auto [pt3, d] = g_renderer.unproject(pt2);
        g_oldPt3  = pt3;
        g_oldDepth = d;

        // Hit-test joints
        g_picked = -1;
        if (g_bvh && !g_newBody.empty()) {
            for (int i = 0; i < k_jointCount; i++) {
                if (glm::length(pt3 - g_newBody[g_frameNum].links[i].getPos()) < k_pickRadius) {
                    g_picked  = i;
                    g_pickPt  = g_newBody[g_frameNum].links[i].getPos();
                    g_targetPt = g_pickPt;
                    break;
                }
            }
        }
    }
    else if (action == GLFW_RELEASE) {
        g_picked = -1;
    }
}

static void onCursorPos(GLFWwindow*, double x, double y) {
    if (ImGui::GetIO().WantCaptureMouse) return;

    glm::vec2 pt2((float)x, (float)y);

    if (g_picked >= 0 && glfwGetMouseButton(glfwGetCurrentContext(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        // IK drag
        g_targetPt = g_pickPt + g_renderer.unprojectAtDepth(pt2, g_oldDepth) - g_oldPt3;
        g_newBody[g_frameNum].solveIK(g_picked, g_targetPt);

        int condition = 0;
        if      (g_picked > 0  && g_picked < 7)                                  condition = 1;
        else if (g_picked >= 7 && g_picked < 13)                                 condition = 2;
        else if (g_picked >= 13 && g_picked < (int)g_newBody[g_frameNum].links.size()) condition = 3;

        g_newBody[g_frameNum].updatePos(condition);
        g_newBody[g_frameNum].getDisplacement(g_oldBody[g_frameNum], g_newBody[g_frameNum]);
        g_newBody[g_frameNum].constraint = true;
    }
    else if (glfwGetMouseButton(glfwGetCurrentContext(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        // Camera orbit
        glm::vec2 delta = pt2 - g_oldPt2;
        g_renderer.m_yaw   += delta.x / (float)WINDOW_W * 3.14159f;
        g_renderer.m_pitch += delta.y / (float)WINDOW_H * 3.14159f;
        g_oldPt2 = pt2;
    }
    else {
        g_oldPt2 = pt2;
    }
}

static void onScroll(GLFWwindow*, double, double yOffset) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    g_renderer.m_dist *= std::pow(0.8f, (float)yOffset);
}

static void onKey(GLFWwindow*, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    if (ImGui::GetIO().WantCaptureKeyboard) return;

    switch (key) {
    case GLFW_KEY_SPACE:
        g_animating = !g_animating;
        if (g_animating) g_lastTime = (float)glfwGetTime();
        break;
    case GLFW_KEY_0:
        g_animating = false;
        init();
        break;
    case GLFW_KEY_1:
        motionEdit();
        break;
    default:
        break;
    }
}

static void onDrop(GLFWwindow*, int count, const char** paths) {
    if (count > 0) loadBVH(paths[0]);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main() {
    if (!glfwInit()) return -1;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WINDOW_W, WINDOW_H,
                                          "Constraint-Based Motion Edit", nullptr, nullptr);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // V-Sync

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { glfwTerminate(); return -1; }

    // Register callbacks
    glfwSetMouseButtonCallback(window, onMouseButton);
    glfwSetCursorPosCallback  (window, onCursorPos);
    glfwSetScrollCallback     (window, onScroll);
    glfwSetKeyCallback        (window, onKey);
    glfwSetDropCallback       (window, onDrop);

    // ImGui setup
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 410");

    // Renderer setup
    g_renderer.m_width  = WINDOW_W;
    g_renderer.m_height = WINDOW_H;
    g_renderer.renderFunction = renderScene;

    // BVH + scene init
    g_bvh = new BVH();
    init();

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Animation step
        if (g_animating) {
            float now = (float)glfwGetTime();
            frame(now - g_lastTime);
            g_lastTime = now;
        }

        // ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Info panel
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(220, 130), ImGuiCond_Always);
        ImGui::Begin("Info", nullptr,
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse);
        ImGui::Text("Frame: %d / %d", g_frameNum, g_totalFrame);
        ImGui::Text("Animating: %s", g_animating ? "Yes" : "No");
        ImGui::Separator();
        ImGui::Text("[Space]  Toggle animation");
        ImGui::Text("[0]      Reset");
        ImGui::Text("[1]      Apply motion edit");
        ImGui::Text("Drag .bvh file to load");
        ImGui::End();

        ImGui::Render();

        // 3D render
        glViewport(0, 0, WINDOW_W, WINDOW_H);
        g_renderer.drawGL();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    delete g_bvh;
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
