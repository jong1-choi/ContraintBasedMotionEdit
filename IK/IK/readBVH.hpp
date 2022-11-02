//
//  readBVH.hpp
//  IK
//
//  Created by 최종원 on 2022/05/17.
//

#ifndef readBVH_hpp
#define readBVH_hpp
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <glm/gtx/quaternion.hpp>
#include "IK.hpp"


using namespace  std;

class  BVH
{
  public:
    enum  ChannelEnum
    {
        X_ROTATION, Y_ROTATION, Z_ROTATION,
        X_POSITION, Y_POSITION, Z_POSITION
    };
    struct  Joint;
    
    struct  Channel
    {
        Joint *              joint;
        
        ChannelEnum          type;

        int                  index;
    };
    
    struct  Joint
    {
        string               name;

        int                  index;

        Joint *              parent;

        vector< Joint * >    children;

        double               offset[3];

        bool                 has_site;

        double               site[3];

        vector< Channel * >  channels;
        
        glm::quat            quat;
        
        glm::vec3            position;

    };


  public:
    
    bool                     is_load_success;
    string                   file_name;
    string                   motion_name;
    int                      num_channel;
    vector< Channel * >      channels;
    vector< Joint * >        joints;
    map< string, Joint * >   joint_index;
    int                      num_frame;
    double                   interval;
    double *                 motion;
    
  public:

    BVH();
    BVH( const char * bvh_file_name );
    ~BVH();

    void  Clear();

    void  Load( const char * bvh_file_name );

  public:
    bool  IsLoadSuccess() const { return is_load_success; }

    const string &  GetFileName() const { return file_name; }
    const string &  GetMotionName() const { return motion_name; }

    const int       GetNumJoint() const { return  joints.size(); }
    const Joint *   GetJoint( int no ) const { return  joints[no]; }
    const int       GetNumChannel() const { return  channels.size(); }
    const Channel * GetChannel( int no ) const { return  channels[no]; }

    const Joint *   GetJoint( const string & j ) const  {
        map< string, Joint * >::const_iterator  i = joint_index.find( j );
        return  ( i != joint_index.end() ) ? (*i).second : NULL; }
    const Joint *   GetJoint( const char * j ) const  {
        map< string, Joint * >::const_iterator  i = joint_index.find( j );
        return  ( i != joint_index.end() ) ? (*i).second : NULL; }

    int     GetNumFrame() const { return  num_frame; }
    double  GetInterval() const { return  interval; }
    double  GetMotion( int f, int c ) const { return  motion[ f*num_channel + c ]; }

    void  SetMotion( int f, int c, double v ) { motion[ f*num_channel + c ] = v; }

  public:
    void  UpdatePose( int frame_no, Body& body, float scale = 1.0f );

    static void  UpdatePose( Joint * root, const double * data, Body& body, float scale = 1.0f );

    static void  RenderPose(glm::vec3 pos, glm::vec3 parentPos, float size);
};

#endif // _BVH_H_
