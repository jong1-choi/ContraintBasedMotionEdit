//
// BVH.h
// ConstraintBasedMotionEdit
//
// BVH motion capture file parser and pose applicator.
// Supports HIERARCHY + MOTION sections of the BVH format.
//

#pragma once

#include <string>
#include <vector>
#include <map>
#include <glm/gtx/quaternion.hpp>

#include "IK.h"

class BVH {
public:
    enum ChannelEnum {
        X_ROTATION, Y_ROTATION, Z_ROTATION,
        X_POSITION, Y_POSITION, Z_POSITION
    };

    struct Joint;

    struct Channel {
        Joint*      joint = nullptr;
        ChannelEnum type;
        int         index = 0;
    };

    struct Joint {
        std::string              name;
        int                      index   = 0;
        Joint*                   parent  = nullptr;
        std::vector<Joint*>      children;
        double                   offset[3] = {};
        bool                     has_site  = false;
        double                   site[3]   = {};
        std::vector<Channel*>    channels;
        glm::quat                quat     = glm::quat(1, 0, 0, 0);
        glm::vec3                position = glm::vec3(0);
    };

public:
    bool                           is_load_success = false;
    std::string                    file_name;
    std::string                    motion_name;
    int                            num_channel = 0;
    std::vector<Channel*>          channels;
    std::vector<Joint*>            joints;
    std::map<std::string, Joint*>  joint_index;
    int                            num_frame = 0;
    double                         interval  = 0.0;
    std::vector<double>            motion;   // [frame * num_channel + channel]

public:
    BVH();
    explicit BVH(const char* bvhFile);
    ~BVH();

    void Clear();
    void Load(const char* bvhFile);

    bool        IsLoadSuccess() const { return is_load_success; }
    int         GetNumFrame()   const { return num_frame; }
    double      GetInterval()   const { return interval; }
    double      GetMotion(int f, int c) const { return motion[(size_t)f * num_channel + c]; }
    void        SetMotion(int f, int c, double v) { motion[(size_t)f * num_channel + c] = v; }

    // Apply frame data to a Body skeleton.
    void UpdatePose(int frameNo, Body& body, float scale = 1.f);

    static void UpdatePose(Joint* root, const double* data, Body& body, float scale = 1.f);
    static void RenderPose(glm::vec3 pos, glm::vec3 parentPos, float size);
};
