////
////  body.hpp
////  IK
////
////  Created by Hyun Joon Shin on 2021/06/14.
////
//
//#ifndef body_h
//#define body_h
//#include <glm/glm.hpp>
//#include <glm/gtx/quaternion.hpp>
//#include <vector>
//#include "GLTools.hpp"
//#include "readBVH.hpp"
//
//#include <Eigen/core>
//#include <Eigen/dense>
//
//struct Link {
//	glm::vec3 l = glm::vec3(0);
//	glm::quat q = glm::quat(1, 0, 0, 0);
//	glm::quat parentGlobalQ = glm::quat(1, 0, 0, 0);
//	glm::vec3 parentGlobalP = glm::vec3(0);
//
//	int parent = -1;
//
//	void render() const {
//		drawSphere( getPos(), 1 );
//		drawSphere( parentGlobalP, 1);
//		drawCylinder( getPos(), parentGlobalP, 0.8 );
//	}
//	glm::vec3 getPos() const {
//		return glm::rotate(getOrientation(), l) + parentGlobalP;
//	}
//	glm::quat getOrientation() const {
//		return parentGlobalQ * q;
//	}
//	void rotate( const glm::quat& rot ) {
//		q = rot * q;
//	}
//	void updatePose( const glm::vec3& pos, const glm::quat& ori ) {
//		parentGlobalQ = ori;
//		parentGlobalP = pos;
//	}
//	Link( int par, const glm::vec3& ll ) : parent(par), l(ll) {}
//};
//struct Joint {
//    glm::vec3 l = glm::vec3(0);
//    glm::quat q = glm::quat(1, 0, 0, 0);
//    glm::quat parentGlobalQ = glm::quat(1, 0, 0, 0);
//    glm::vec3 parentGlobalP = glm::vec3(0);
//    
//    
//    
//    BVH bvh;
//    
//};
//
//struct Body {
//	std::vector<Link> links;
//	glm::vec3 globalP = glm::vec3(0);
//	glm::quat globalQ = glm::quat(1, 0, 0, 0);
//
//	void clear() {
//		links.clear();
//		globalP = glm::vec3(0);
//		globalQ = glm::quat(1, 0, 0, 0);
//	}
//	void add(int parent, const glm::vec3& l ) {
//		links.push_back(Link(parent, l));
//	}
//	void render() const {
//		for (auto& l : links) l.render();
//	}
//	void updatePos() {
//        for (auto& l : links) {
//			if (l.parent < 0) {
//				l.updatePose(globalP, globalQ);
//			}
//			else {
//				l.updatePose(links[l.parent].getPos(), links[l.parent].getOrientation());
//			}
//		}
//	}
//	const std::vector<int> getAncestors( int end ) {
//		std::vector<int> ret;
//
//		for (int i = 0; i <= end; i++) {
//			ret.push_back(i);
//		}
//		return ret;
//	}
//	void solveIK(int target, const glm::vec3& targetP);
//};
//
////struct Link {
////    glm::vec3 l = glm::vec3(0);
////    glm::quat q = glm::quat(1, 0, 0, 0);
////    glm::quat parentGlobalQ = glm::quat(1, 0, 0, 0);
////    glm::vec3 parentGlobalP = glm::vec3(0);
////
////    int parent = -1;
////
////    void render() const {
////        drawSphere( getPos(), 1 );
////        drawSphere( parentGlobalP, 1);
////        drawCylinder( getPos(), parentGlobalP, 0.8 );
////    }
////    glm::vec3 getPos() const {
////        return glm::rotate(getOrientation(), l) + parentGlobalP;
////    }
////    glm::quat getOrientation() const {
////        return parentGlobalQ * q;
////    }
////    void rotate( const glm::quat& rot ) {
////        q = rot * q;
////    }
////    void updatePose( const glm::vec3& pos, const glm::quat& ori ) {
////        parentGlobalQ = ori;
////        parentGlobalP = pos;
////    }
////    Link( int par, const glm::vec3& ll ) : parent(par), l(ll) {}
////};
////
////
////struct Body {
////    std::vector<Link> links;
////    glm::vec3 globalP = glm::vec3(0);
////    glm::quat globalQ = glm::quat(1, 0, 0, 0);
////
////    void clear() {
////        links.clear();
////        globalP = glm::vec3(0);
////        globalQ = glm::quat(1, 0, 0, 0);
////    }
////    void add(int parent, const glm::vec3& l ) {
////        links.push_back(Link(parent, l));
////    }
////    void render() const {
////        for (auto& l : links) l.render();
////    }
////    void updatePos() {
////        for (auto& l : links) {
////            if (l.parent < 0) {
////                l.updatePose(globalP, globalQ);
////            }
////            else {
////                l.updatePose(links[l.parent].getPos(), links[l.parent].getOrientation());
////            }
////        }
////    }
////    const std::vector<int> getAncestors( int end ) {
////        std::vector<int> ret;
////
////        for (int i = 0; i <= end; i++) {
////            ret.push_back(i);
////        }
////        return ret;
////    }
////    void solveIK(int target, const glm::vec3& targetP);
////};
//
//#endif /* body_h */
