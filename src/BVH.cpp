//
// BVH.cpp
// ConstraintBasedMotionEdit
//
// BVH file loader: parses HIERARCHY and MOTION sections,
// converts Euler angles to quaternions, and applies pose to Body.
//

#include "BVH.h"
#include "ShaderUtils.h"

#include <fstream>
#include <cstring>
#include <iostream>

static constexpr float k_pi = 3.14159265f;

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

BVH::BVH()  { Clear(); }

BVH::BVH(const char* bvhFile) {
    Clear();
    Load(bvhFile);
}

BVH::~BVH() { Clear(); }

// ---------------------------------------------------------------------------
// Clear
// ---------------------------------------------------------------------------

void BVH::Clear() {
    for (auto* c : channels) delete c;
    for (auto* j : joints)   delete j;

    channels.clear();
    joints.clear();
    joint_index.clear();
    motion.clear();

    is_load_success = false;
    file_name       = "";
    motion_name     = "";
    num_channel     = 0;
    num_frame       = 0;
    interval        = 0.0;
}

// ---------------------------------------------------------------------------
// Load
// ---------------------------------------------------------------------------

void BVH::Load(const char* bvhFile) {
    constexpr int k_bufLen = 1024 * 32;

    std::ifstream file(bvhFile, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "[BVH] Cannot open: " << bvhFile << "\n";
        return;
    }

    Clear();

    // Derive motion name from file path
    file_name = bvhFile;
    const char* nameStart = bvhFile;
    const char* nameEnd   = bvhFile + strlen(bvhFile);
    if (auto* p = strrchr(bvhFile, '\\')) nameStart = p + 1;
    else if (auto* p = strrchr(bvhFile, '/')) nameStart = p + 1;
    if (auto* p = strrchr(bvhFile, '.')) nameEnd = p;
    if (nameEnd < nameStart) nameEnd = bvhFile + strlen(bvhFile);
    motion_name.assign(nameStart, nameEnd);

    char   line[k_bufLen];
    char   sep[] = " :,\t\r";
    char*  tok       = nullptr;
    Joint* joint     = nullptr;
    Joint* new_joint = nullptr;

    std::vector<Joint*> stack;

    // Parse HIERARCHY
    while (!file.eof()) {
        file.getline(line, k_bufLen);
        tok = strtok(line, sep);
        if (!tok) continue;

        if (strcmp(tok, "{") == 0) {
            stack.push_back(joint);
            joint = new_joint;
            continue;
        }
        if (strcmp(tok, "}") == 0) {
            joint = stack.back();
            stack.pop_back();
            continue;
        }

        if (strcmp(tok, "ROOT") == 0 || strcmp(tok, "JOINT") == 0 || strcmp(tok, "End") == 0) {
            new_joint = new Joint();
            new_joint->index    = (int)joints.size();
            new_joint->parent   = joint;
            new_joint->has_site = (strcmp(tok, "End") == 0);
            joints.push_back(new_joint);
            if (joint) joint->children.push_back(new_joint);

            tok = strtok(nullptr, "");
            while (tok && *tok == ' ') tok++;
            new_joint->name = tok ? tok : "";
            joint_index[new_joint->name] = new_joint;
            continue;
        }

        if (strcmp(tok, "OFFSET") == 0 && joint) {
            auto nextDouble = [&]() -> double {
                tok = strtok(nullptr, sep);
                return tok ? atof(tok) : 0.0;
            };
            joint->offset[0] = nextDouble();
            joint->offset[1] = nextDouble();
            joint->offset[2] = nextDouble();
            continue;
        }

        if (strcmp(tok, "CHANNELS") == 0 && joint) {
            tok = strtok(nullptr, sep);
            int count = tok ? atoi(tok) : 0;
            joint->channels.resize(count);
            for (int i = 0; i < count; i++) {
                auto* ch  = new Channel();
                ch->joint = joint;
                ch->index = (int)channels.size();
                channels.push_back(ch);
                joint->channels[i] = ch;

                tok = strtok(nullptr, sep);
                if      (!tok)                            ch->type = X_ROTATION;
                else if (strcmp(tok, "Xrotation") == 0)  ch->type = X_ROTATION;
                else if (strcmp(tok, "Yrotation") == 0)  ch->type = Y_ROTATION;
                else if (strcmp(tok, "Zrotation") == 0)  ch->type = Z_ROTATION;
                else if (strcmp(tok, "Xposition") == 0)  ch->type = X_POSITION;
                else if (strcmp(tok, "Yposition") == 0)  ch->type = Y_POSITION;
                else                                      ch->type = Z_POSITION;
            }
            continue;
        }

        if (strcmp(tok, "MOTION") == 0) break;
    }

    // Parse MOTION header
    file.getline(line, k_bufLen);
    tok = strtok(line, sep);
    if (!tok || strcmp(tok, "Frames") != 0) return;
    tok = strtok(nullptr, sep);
    if (!tok) return;
    num_frame = atoi(tok);

    file.getline(line, k_bufLen);
    tok = strtok(line, ":");
    if (!tok || strcmp(tok, "Frame Time") != 0) return;
    tok = strtok(nullptr, sep);
    if (!tok) return;
    interval = atof(tok);

    num_channel = (int)channels.size();
    motion.assign((size_t)num_frame * num_channel, 0.0);

    for (int i = 0; i < num_frame; i++) {
        file.getline(line, k_bufLen);
        tok = strtok(line, sep);
        for (int j = 0; j < num_channel; j++) {
            if (!tok) break;
            motion[(size_t)i * num_channel + j] = atof(tok);
            tok = strtok(nullptr, sep);
        }
    }

    file.close();
    is_load_success = true;
    std::cout << "[BVH] Loaded: " << file_name
              << "  frames=" << num_frame << "\n";
}

// ---------------------------------------------------------------------------
// UpdatePose
// ---------------------------------------------------------------------------

void BVH::UpdatePose(int frameNo, Body& body, float scale) {
    UpdatePose(joints[0], motion.data() + (size_t)frameNo * num_channel, body, scale);
}

void BVH::UpdatePose(Joint* joint, const double* data, Body& body, float scale) {
    using namespace glm;

    joint->quat = quat(1, 0, 0, 0);

    // Apply rotation channels via exponential map (Euler → quaternion)
    for (auto* ch : joint->channels) {
        float angle = (float)data[ch->index] * k_pi / 180.f / 2.f;
        if      (ch->type == X_ROTATION) joint->quat *= glm::exp(angle * quat(0, 1, 0, 0));
        else if (ch->type == Y_ROTATION) joint->quat *= glm::exp(angle * quat(0, 0, 1, 0));
        else if (ch->type == Z_ROTATION) joint->quat *= glm::exp(angle * quat(0, 0, 0, 1));
        joint->quat = glm::normalize(joint->quat);
    }

    Joint* parent = joint->parent;
    if (!parent) {
        // Root joint — position comes from the first three channels
        joint->position = scale * vec3(data[0], data[1], data[2]);
        body.add(-1, joint->children[0]->index,
                 joint->position, joint->quat,
                 vec3(0), quat(1, 0, 0, 0), joint->has_site);
    }
    else if (joint->has_site) {
        // End effector — no rotation, just offset
        joint->quat = quat(1, 0, 0, 0);
        vec3 offset  = scale * vec3(joint->offset[0], joint->offset[1], joint->offset[2]);
        joint->position = parent->quat * offset + parent->position;
        body.add(parent->index, -1, offset, joint->quat,
                 parent->position, parent->quat, joint->has_site);
    }
    else {
        vec3 offset  = scale * vec3(joint->offset[0], joint->offset[1], joint->offset[2]);
        joint->position = parent->quat * offset + parent->position;
        body.add(parent->index, joint->children[0]->index,
                 offset, joint->quat,
                 parent->position, parent->quat, joint->has_site);
    }

    for (auto* child : joint->children)
        UpdatePose(child, data, body, scale);
}

void BVH::RenderPose(glm::vec3 pos, glm::vec3 parentPos, float size) {
    drawSphere(pos, size, glm::vec4(1, 1, 0, .1f));
    drawCylinder(pos, parentPos, 1.f);
}
