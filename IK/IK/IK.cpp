//
//  IK.cpp
//  IK
//
//  Created by 최종원 on 2022/06/27.
//

#include "IK.hpp"

// 선택된 joint와 옮겨진 위치를 받아오는 func
void Body::solveIK(int target, const glm::vec3& targetP) {
    using namespace Eigen;
    using namespace glm;
    auto a = getAncestors(target);
    // Repeat some times
    if (a.size() == 0) return;
    MatrixXf J = MatrixXf(3, a.size() * 3); //constraints 3개, DOF 6개(q의 개수 * 3차원의 자유도)
    MatrixXf b = MatrixXf(3, 1); //constraints 3개 짜리 column vector
    const vec3 axes[] = { {1,0,0}, {0,1,0}, {0,0,1} }; // x, y, z축
    for (auto iter = 0; iter < 100; iter++) {
        // Compute Jacobian 자코비안은 매번 새로 계산해야해서 loop 안에 있음
        // Jacobian(i, j) = dF(i) / dTheta(j)
        // Jacobian((0-6), j) = x축/y축/z축으로 돌리는거를 각각 Theta로해서 자유도를 3개로 줌
        for (int i = 0; i < a.size(); i++) { //각각 joint에 대해서 joint index
            vec3 p = links[target].getPos() - links[a[i]].getPos(); //i번째 link의 parent를 end-effect의 위치에서 빼주기
            for (int j = 0; j < 3; j++) { //각각 축에 대해서 axis index
                // Jacobian((0-6), j) = (axis) X (position)
                vec3 v = cross(axes[j], p);
                J.col(i * 3 + j) << v.x, v.y, v.z;
            }
        }
        
        // solve dTheta = (Jacobian Inverse)(dPosition)
        vec3 d = targetP - links[target].getPos(); // end-effector의 위치가 target방향으로 움직여야함
        b << d.x, d.y, d.z;
        // SVD로 풀자!
        auto solver = J.bdcSvd(ComputeThinU | ComputeThinV); // U, V를 계산할 때 U,V를 얼만큼 정확하게 사용할지 정해주기 Thin은 대충 정해주는거
        solver.setThreshold(0.01); // 0.01보다 작으면 0으로 만들어라
        auto x = solver.solve(b);
        
        // theta = deltaT * dTheta + theta -> forward kinematics 성질이 locally linear하나는 성질을 이용해서
        for (int i = 0; i < a.size(); i++) {
            for (int j = 0; j < 3; j++) {
                if( links[a[i]].parentIndex < 0) break;
                links[a[i]].rotate(exp(quat(0, 0.01f * axes[j] * x(i * 3 + j))));
            }
        }
//        updatePos();
        
    }
}
