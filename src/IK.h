//
// IK.h
// ConstraintBasedMotionEdit
//
// Link/Body data structures and inverse kinematics solver.
// Implements a Jacobian-based IK with SVD pseudo-inverse (Eigen).
//

#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

#include "ShaderUtils.h"

// ---------------------------------------------------------------------------
// Link  — one bone in the kinematic chain
// ---------------------------------------------------------------------------
struct Link {
    glm::vec3 l  = glm::vec3(0);              // local offset from parent joint
    glm::quat q  = glm::quat(1, 0, 0, 0);    // local orientation

    glm::vec3 m_parentGlobalP = glm::vec3(0);
    glm::quat m_parentGlobalQ = glm::quat(1, 0, 0, 0);

    bool isEnd      = false;
    int  parentIndex = -1;
    int  childIndex  = -1;

    Link(int parI, int chiI,
         const glm::vec3& ll, const glm::quat& qq,
         const glm::vec3& pl, const glm::quat& pq,
         bool end)
        : parentIndex(parI), childIndex(chiI),
          l(ll), q(qq),
          m_parentGlobalP(pl), m_parentGlobalQ(pq),
          isEnd(end) {}

    // World position of this joint
    glm::vec3 getPos() const { return m_parentGlobalQ * l + m_parentGlobalP; }
    // World orientation of this joint
    glm::quat getOri() const { return m_parentGlobalQ * q; }

    void rotate(const glm::quat& rot)                   { q = rot * q; }
    void updatePose(const glm::vec3& pos, const glm::quat& ori) {
        m_parentGlobalP = pos;
        m_parentGlobalQ = ori;
    }

    void render() const;
    void shapeRender() const;
};

// ---------------------------------------------------------------------------
// Body  — full skeleton (collection of Links)
// ---------------------------------------------------------------------------
struct Body {
    std::vector<Link>  links;
    glm::vec3          globalP      = glm::vec3(0);
    glm::quat          globalQ      = glm::quat(1, 0, 0, 0);
    Eigen::MatrixXf    displacement;
    bool               constraint   = false;

    void clear();
    void add(int parI, int chiI,
             const glm::vec3& ll, const glm::quat& qq,
             const glm::vec3& pl, const glm::quat& pq,
             bool end);

    void render()      const;
    void shapeRender() const;

    // Forward kinematics — propagates transforms down the chain.
    // condition selects which sub-chain to update (0 = all).
    void updatePos(int condition);

    // Returns all ancestor indices from end-effector to root.
    std::vector<int> getAncestors(int end) const;

    // Iterative Jacobian IK: move joint 'target' to 'targetP'.
    void solveIK(int target, const glm::vec3& targetP);

    // Stores the quaternion log-map displacement between origin and edited pose.
    void getDisplacement(const Body& origin, const Body& edited);
};
