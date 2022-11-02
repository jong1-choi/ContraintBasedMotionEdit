//
//  main.cpp
//  IK
//
//  Created by Hyun Joon Shin on 2021/06/09.
//

#include <iostream>
#include <JGL/JGL_Window.hpp>
#include "AnimView.hpp"
#include "readBVH.hpp"
#include "GLTools.hpp"

glm::vec3 oldPt3;
glm::vec2 oldPt2;
glm::vec3 pickPt;
glm::vec3 targetPt;
BVH *bvh;
int picked = -1;
float oldD;
vector<Body> newBody;
vector<Body> oldBody;

AnimView* animView;
int totalFrame = 0;
int frameNum = 0;
float frameTime = 0;
vector<int> cstr;


void init(){
    newBody.clear();
    oldBody.clear();
    bvh->Load("/Users/choijongwon/Desktop/ContraintBasedMotionEdit/IK/BVH/WalkStartA.bvh");
    Body temp;
    totalFrame = bvh->num_frame;
    for(int i = 0; i < bvh->num_frame; i++){
        temp.clear();
        bvh->UpdatePose(i,temp, 5);
        
        newBody.push_back(temp);
        oldBody.push_back(temp);
        
        newBody[i].getDisplacement(newBody[i], newBody[i]);
        oldBody[i].getDisplacement(newBody[i], newBody[i]);
        
        oldBody[i].updatePos(0);
        newBody[i].updatePos(0);
    }
    
    frameNum = 0;
    frameTime = 0;
    
}

void frame(float dt) {
    frameTime += dt;

    if(frameTime > 0.03f){
        frameNum++;
        frameTime = 0;
        newBody[frameNum].updatePos(0);
    }
    if(frameNum == bvh->num_frame){
        frameNum = 0;
    }
}

void render() {

    for(int i = 1; i < 5; i++){
        newBody[(totalFrame/4) * i].controlPRender();
    }
    
    if(bvh->joints.size() > 0){
//        newBody[frameNum].render();
        newBody[frameNum].shapeRender();
        for(int i = 0; i < bvh->num_frame; i++){
            newBody[i].shapeRender();
        }
    }
    if( picked>=0 ) {
        drawSphere( targetPt, 1.5, glm::vec4(1,1,0,.1) );
    }

    drawQuad(glm::vec3(0,0,0), glm::vec3(0,1,0), glm::vec2(2000,2000));
}

bool getFile(const string file) {

    newBody.clear();
    oldBody.clear();
    frameNum = 0;
    frameTime = 0;
    bvh->Clear();
    bvh->Load(file.c_str());

    Body temp;
    for(int i = 0; i < bvh->num_frame; i++){
        temp.clear();
        bvh->UpdatePose(i,temp, 5);
        newBody.push_back(temp);
        oldBody.push_back(temp);
        
        oldBody[i].updatePos(0);
        newBody[i].updatePos(0);
        
        newBody[i].getDisplacement(newBody[i], newBody[i]);
        oldBody[i].getDisplacement(oldBody[i], oldBody[i]);
    }

    return true;
}

bool press( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
    oldPt3 = pt3;
    oldPt2 = pt2;
    oldD = d;
//    for( int i=0; i < 31 ; i++)
//    if( length( pt3 - newBody[frameNum].links[i].getPos() ) < 1.5 ) {
//        picked = i;
//        targetPt = pickPt = newBody[frameNum].links[i].getPos();
//        return true;
//    }
    for( int i=1; i < 5 ; i++)
    if( length( pt3 - newBody[(totalFrame/4) * i].links[i].getPos() ) < 1.5 ) {
        picked = i;
        targetPt = pickPt = newBody[(totalFrame/4) * i].links[i].getPos();
        return true;
    }
    return false;
}

//4번 LeftHeel 6번 LeftToe 10번 RightHeel 12번 RigthToe
bool drag( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
    if( picked>=0 ) {
        targetPt = pickPt + animView->unproject( pt2, oldD ) - oldPt3;
        newBody[frameNum].solveIK(picked, targetPt);
        int condition = 0;
        if(picked > 0 && picked < 7) condition = 1;
        else if(picked >= 7 && picked < 13) condition = 2;
        else if(picked >= 13 && picked < newBody[frameNum].links.size()) condition = 3;
        newBody[frameNum].updatePos(condition);
        
        newBody[frameNum].getDisplacement(oldBody[frameNum], newBody[frameNum]);

        newBody[frameNum].constraint = true;
//
//        for(int i = 0; i < totalFrame; i++){
//            cout << i << " frame " << endl;
//            cout << body[i].displacement << endl;
//        }
//
        return true;
    }
    return false;
}

void motionEdit(){
    vector<int> cons;
    float t = 0;
    int space = 2;
    int controlN;
    controlN = totalFrame/space + 1;
    vector<vec3> controlP;
    
    for(int i = 0; i < controlN; i++){
        controlP.push_back(vec3(0));
    }

    for(int i = 0; i < totalFrame; i++){
        if(newBody[i].constraint) {
            cons.push_back(i);
            newBody[i].constraint = false;
        }
    }

//    for(int i = 0; i < bvh->joints.size(); i++){
//        cout << i << " " << bvh->joints[i]->name << endl;
//    }
    for(int i = 0; i < bvh->joints.size()+1; i++){
        
        if(cons.size() == 0) break;
        
        Eigen::MatrixXf basis = Eigen::MatrixXf(cons.size(),4);
        Eigen::MatrixXf p = Eigen::MatrixXf(cons.size(),3);
        Eigen::MatrixXf b = Eigen::MatrixXf(4,3);
        
        for(int j = 0; j < cons.size(); j++){
            
            if(cons[j]/space < 1 || cons[j]/space > controlN - 3){
                cout << "예외" << endl;
            }
            else{
                t = (cons[j]%space)/float(space);
                basis(j,0) = (t*t*t)/6.f;
                basis(j,1) = (-3*t*t*t + 3*t*t + 3*t + 1)/6.f;
                basis(j,2) = (3*t*t*t - 6*t*t + 4)/6.f;
                basis(j,3) = ((1-t)*(1-t)*(1-t))/6.f;
                
                p(j,0) = newBody[cons[j]].displacement(i,0);
                p(j,1) = newBody[cons[j]].displacement(i,1);
                p(j,2) = newBody[cons[j]].displacement(i,2);
                
                auto solver = basis.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
                b = solver.solve(p);
                
//                cout << "jointNum " << i-1 << endl;
//                cout << b << endl;
                
                controlP[cons[j]/space - 1] = vec3(b(0,0), b(0,1), b(0,2));
                controlP[cons[j]/space    ] = vec3(b(1,0), b(1,1), b(1,2));
                controlP[cons[j]/space + 1] = vec3(b(2,0), b(2,1), b(2,2));
                controlP[cons[j]/space + 2] = vec3(b(3,0), b(3,1), b(3,2));
            }
        }
        
        cout << "jointNum " << i-1 << endl;
        for(int j = 0; j < controlP.size(); j++){

            cout << controlP[j].x << " " << controlP[j].y << " " << controlP[j].z << endl;
        }
        
        for(int j = 0; j < totalFrame; j++){

            if(i == 0) break;
            
            t = (j%space)/float(space);

            if(j/space < 1 || j/space > controlN - 3){
                
            }
            else{
                vec3 bspline;
                
                bspline =
                controlP[j/space - 1] * (t*t*t)/6.f +
                controlP[j/space    ] * (-3*t*t*t + 3*t*t + 3*t + 1)/6.f +
                controlP[j/space + 1] * (3*t*t*t - 6*t*t + 4)/6.f +
                controlP[j/space + 2] * ((1-t)*(1-t)*(1-t))/6.f;
                
                quat dq = quat(1,bspline.x,bspline.y,bspline.x);
                
//                cout << "frame " << j << endl;
//                cout << dq.x << " " << dq.y << " " << dq.z << endl;
//                cout << bspline.x << " " << bspline.x << " " << bspline.x << endl;
                
                newBody[j].links[i-1].q = oldBody[j].links[i-1].q * dq;
            }
        }
    }
    
    for(int j = 0; j < totalFrame; j++){
        newBody[j].updatePos(0);
    }
    
}

bool release( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
    picked = -1;
    return true;
}

int main(int argc, const char * argv[]) {
    JGL::Window* window = new JGL::Window(800, 600, "simulation");
    window->alignment(JGL::ALIGN_ALL);
    bvh = new BVH;
    
    animView = new AnimView(0, 0, 800, 600);
    
    animView->initFunction = init;
    animView->getFileName = getFile;
    animView->renderFunction = render;
    animView->frameFunction = frame;
    animView->editFunction = motionEdit;
    animView->pressFunc = press;
    animView->dragFunc = drag;
    animView->releaseFunc = release;
    
    
    init();
    window->show();
    JGL::_JGL::run();
    return 0;
}
