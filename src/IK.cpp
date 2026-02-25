//
// IK.cpp
// ConstraintBasedMotionEdit
//
// Link rendering and Body IK solver implementation.
//

#include "IK.h"

// ---------------------------------------------------------------------------
// Link
// ---------------------------------------------------------------------------

void Link::render() const {
    drawSphere(getPos(), 1.f);
    drawSphere(m_parentGlobalP, 1.f);
    drawCylinder(getPos(), m_parentGlobalP, 0.8f);
}

void Link::shapeRender() const {
    drawCylinder(getPos(), m_parentGlobalP, 0.1f);
}

// ---------------------------------------------------------------------------
// Body
// ---------------------------------------------------------------------------

void Body::clear() {
    links.clear();
    globalP = glm::vec3(0);
    globalQ = glm::quat(1, 0, 0, 0);
}

void Body::add(int parI, int chiI,
               const glm::vec3& ll, const glm::quat& qq,
               const glm::vec3& pl, const glm::quat& pq,
               bool end) {
    links.emplace_back(parI, chiI, ll, qq, pl, pq, end);
}

void Body::render() const {
    for (const auto& link : links)
        if (link.parentIndex >= 0)
            link.render();
}

void Body::shapeRender() const {
    for (const auto& link : links)
        if (link.parentIndex >= 0)
            link.shapeRender();
}

void Body::updatePos(int condition) {
    int start = 1, end = (int)links.size();
    if      (condition == 1) { start = 1;  end = 7;  }
    else if (condition == 2) { start = 7;  end = 13; }
    else if (condition == 3) { start = 13; end = (int)links.size(); }

    for (int i = start; i < end; i++) {
        const Link& parent = links[links[i].parentIndex];
        links[i].updatePose(parent.getPos(), parent.getOri());
    }
}

std::vector<int> Body::getAncestors(int end) const {
    std::vector<int> result;
    int idx = end;
    while (links[idx].parentIndex != -1) {
        result.push_back(links[idx].parentIndex);
        idx = links[idx].parentIndex;
    }
    return result;
}

void Body::solveIK(int target, const glm::vec3& targetP) {
    using namespace Eigen;
    using namespace glm;

    auto ancestors = getAncestors(target);
    if (ancestors.empty()) return;

    const vec3 axes[] = { {1,0,0}, {0,1,0}, {0,0,1} };
    const int  dof    = (int)ancestors.size() * 3;

    MatrixXf J(3, dof);
    MatrixXf b(3, 1);

    // Iterative Jacobian IK
    // dTheta = J^+ * dP,  where J^+ is the SVD pseudo-inverse
    for (int iter = 0; iter < 100; iter++) {
        // Early exit when close enough
        vec3 err = targetP - links[target].getPos();
        if (length(err) < 0.01f) break;

        // Build Jacobian: J(i,j) = axis_j × (p_end - p_joint_i)
        for (int i = 0; i < (int)ancestors.size(); i++) {
            vec3 p = links[target].getPos() - links[ancestors[i]].getPos();
            for (int j = 0; j < 3; j++) {
                vec3 v = cross(axes[j], p);
                J.col(i * 3 + j) << v.x, v.y, v.z;
            }
        }

        b << err.x, err.y, err.z;

        // Solve via SVD with threshold to handle near-singular cases
        auto solver = J.bdcSvd(ComputeThinU | ComputeThinV);
        solver.setThreshold(0.01f);
        auto x = solver.solve(b);

        // Apply incremental rotations (exponential map for smooth quaternion interp)
        for (int i = 0; i < (int)ancestors.size(); i++) {
            if (links[ancestors[i]].parentIndex < 0) break;
            for (int j = 0; j < 3; j++)
                links[ancestors[i]].rotate(glm::exp(quat(0.f, 0.01f * axes[j] * x(i * 3 + j))));
            links[ancestors[i]].q = glm::normalize(links[ancestors[i]].q);
        }

        // Propagate updated rotations to world positions before next iteration
        updatePos(0);
    }
}

void Body::getDisplacement(const Body& origin, const Body& edited) {
    Eigen::MatrixXf d((int)links.size() + 1, 3);

    // Root translation displacement (in local frame of origin root)
    glm::vec3 t = glm::inverse(origin.links[0].q)
                  * (edited.links[0].getPos() - origin.links[0].getPos())
                  * origin.links[0].q;
    d(0, 0) = t.x;
    d(0, 1) = t.y;
    d(0, 2) = t.z;

    // Per-joint orientation displacement via quaternion log-map
    for (int i = 1; i < (int)links.size() + 1; i++) {
        glm::quat dq  = glm::log(glm::inverse(origin.links[i - 1].q) * edited.links[i - 1].q);
        d(i, 0) = dq.x;
        d(i, 1) = dq.y;
        d(i, 2) = dq.z;
    }

    displacement = d;
}
