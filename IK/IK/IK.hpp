//
//  IK.hpp
//  IK
//
//  Created by 최종원 on 2022/06/27.
//

#ifndef IK_hpp
#define IK_hpp

#include <stdio.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>
#include <string.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "GLTools.hpp"

using namespace glm;
using namespace std;

struct Link {
    glm::vec3 l = glm::vec3(0);
    glm::quat q = glm::quat(1, 0, 0, 0);
    glm::vec3 parentGlobalP = glm::vec3(0);
    glm::quat parentGlobalQ = glm::quat(1, 0, 0, 0);

    bool isEnd = false;
    int parentIndex = -1;
    int childIndex = -1;

    void render() const {
        drawSphere( getPos(), 1 );
        drawSphere( parentGlobalP, 1);
        drawCylinder( getPos(), parentGlobalP, 0.8 );
    }
    void shapeRender() const {
        drawCylinder( getPos(), parentGlobalP, 0.1);
    }
    
    glm::vec3 getPos() const {
        return parentGlobalQ * l + parentGlobalP;
    }
    glm::quat getOri() const {
        return parentGlobalQ * q;
    }
    void rotate( const glm::quat& rot ) {
        q = rot * q;
    }
    void updatePose( const glm::vec3& pos, const glm::quat& ori ) {
        parentGlobalP = pos;
        parentGlobalQ = ori;
    }
    Link( int parI, int chiN, const glm::vec3& ll, const glm::quat& qq, const glm::vec3& pl, const glm::quat& pq, bool end) : parentIndex(parI), childIndex(chiN), l(ll), q(qq) , parentGlobalP(pl), parentGlobalQ(pq), isEnd(end){}

};


struct Body {
    std::vector<Link> links;
    glm::vec3 globalP = glm::vec3(0);
    glm::quat globalQ = glm::quat(1, 0, 0, 0);
    Eigen::MatrixXf displacement;
    bool constraint = false;
    void clear() {
        links.clear();
        globalP = glm::vec3(0);
        globalQ = glm::quat(1, 0, 0, 0);
    }
    void add( int parI, int chiI, const glm::vec3& ll, const glm::quat& qq, const glm::vec3& pl, const glm::quat& pq, bool end ) {
        links.push_back(Link(parI, chiI, ll, qq, pl, pq, end));
    }
    void render() const {
        for (auto& l : links) {
            if(l.parentIndex >= 0)
                l.render();
        }
    }
    
    void shapeRender() const {
        for (auto& l : links) {
            if(l.parentIndex >= 0)
                l.shapeRender();
        }
    }
    
    
    void updatePos(int condition) {

        if(condition == 0){
            for (int i = 1; i < links.size(); i++) {
                Link pLink = links[links[i].parentIndex];
                links[i].updatePose(pLink.getPos(), pLink.getOri());
            }
        }
        else if(condition == 1){
            for (int i = 1; i < 7; i++) {
                Link pLink = links[links[i].parentIndex];
                links[i].updatePose(pLink.getPos(), pLink.getOri());
            }
        }
        else if(condition == 2){
            for (int i = 7; i < 13; i++) {
                Link pLink = links[links[i].parentIndex];
                links[i].updatePose(pLink.getPos(), pLink.getOri());
            }
        }
        else if(condition == 3){
            for (int i = 13; i < links.size(); i++) {
                Link pLink = links[links[i].parentIndex];
                links[i].updatePose(pLink.getPos(), pLink.getOri());
            }
        }
    }
    const std::vector<int> getAncestors( int end ) {
        std::vector<int> ret;
        int index = end;
        bool endCheck = 1;
        
        while(endCheck){
            if(links[index].parentIndex == -1)
                endCheck = false;
            else{
                ret.push_back(links[index].parentIndex);
                index = links[index].parentIndex;
            }
        }
        
        return ret;
    }
    
    void solveIK(int target, const glm::vec3& targetP);
    
    void getDisplacement(const Body& origin, const Body& edited){
        Eigen::MatrixXf d = Eigen::MatrixXf(links.size() + 1,3);
        d(0,0) =
        (inverse(origin.links[0].q)*(edited.links[0].getPos() - origin.links[0].getPos())*origin.links[0].q).x;
        d(0,1) =
        (inverse(origin.links[0].q)*(edited.links[0].getPos() - origin.links[0].getPos())*origin.links[0].q).y;
        d(0,2) =
        (inverse(origin.links[0].q)*(edited.links[0].getPos() - origin.links[0].getPos())*origin.links[0].q).z;
        
        for(int i = 1; i < links.size() + 1; i++){
            d(i,0) =
            (log(inverse(origin.links[i-1].q) * edited.links[i-1].q)).x;
            d(i,1) =
            (log(inverse(origin.links[i-1].q) * edited.links[i-1].q)).y;
            d(i,2) =
            (log(inverse(origin.links[i-1].q) * edited.links[i-1].q)).z;
            
        }
        displacement = d;
    }
};

#endif /* IK_hpp */
