//
//  readBVH.cpp
//  IK
//
//  Created by 최종원 on 2022/05/17.
//
//#include <math.h>
#include <fstream>
#include <string.h>
#include <glm/gtx/quaternion.hpp>
#include <glm/glm.hpp>
//#include <GLFW/glfw3.h>
#include <OpenGL/gl3.h>
#include "GLTools.hpp"
#include "readBVH.hpp"
#include "IK.hpp"
#define PI 3.141592f


using namespace std;
using namespace glm;

BVH::BVH()
{
    motion = NULL;
    Clear();
}

BVH::BVH( const char * bvh_file_name )
{
    motion = NULL;
    Clear();

    Load( bvh_file_name );
}

BVH::~BVH()
{
    Clear();
}

void  BVH::Clear()
{
    int  i,j;
    for ( i=0; i<channels.size(); i++ )
        delete  channels[ i ];
    for ( i=0; i<joints.size(); i++ )
        delete  joints[ i ];
    
    if ( motion != NULL )
        delete  motion;

    is_load_success = false;
    
    file_name = "";
    motion_name = "";
    
    num_channel = 0;
    channels.clear();
    joints.clear();
    joint_index.clear();
    
    num_frame = 0;
    interval = 0.0;
    motion = NULL;
}

void  BVH::Load( const char * bvh_file_name )
{
    #define  BUFFER_LENGTH  1024*32

    ifstream  file;
    char      line[ BUFFER_LENGTH ];
    char *    token;
    char      separater[] = " :,\t\r";
    vector< Joint * >   joint_stack;
    Joint *   joint = NULL;
    Joint *   new_joint = NULL;
    bool      is_site = false;
    double    x, y ,z;
    int       i, j;

    Clear();

    file_name = bvh_file_name;
    const char *  mn_first = bvh_file_name;
    const char *  mn_last = bvh_file_name + strlen( bvh_file_name );
    if ( strrchr( bvh_file_name, '\\' ) != NULL )
        mn_first = strrchr( bvh_file_name, '\\' ) + 1;
    else if ( strrchr( bvh_file_name, '/' ) != NULL )
        mn_first = strrchr( bvh_file_name, '/' ) + 1;
    if ( strrchr( bvh_file_name, '.' ) != NULL )
        mn_last = strrchr( bvh_file_name, '.' );
    if ( mn_last < mn_first )
        mn_last = bvh_file_name + strlen( bvh_file_name );
    motion_name.assign( mn_first, mn_last );

    file.open( bvh_file_name, ios::in );
    if ( file.is_open() == 0 )  return;

    while ( ! file.eof() )
    {
        if ( file.eof() )
            goto bvh_error;

        file.getline( line, BUFFER_LENGTH );
        token = strtok( line, separater );

        if ( token == NULL )  continue;

        if ( strcmp( token, "{" ) == 0 )
        {
            joint_stack.push_back( joint );
            joint = new_joint;
            continue;
        }
        if ( strcmp( token, "}" ) == 0 )
        {
            joint = joint_stack.back();
            joint_stack.pop_back();
            is_site = false;
            continue;
        }

//        if ( ( strcmp( token, "ROOT" ) == 0 ) ||
//             ( strcmp( token, "JOINT" ) == 0 ) ||
//             ( strcmp( token, "End" ) == 0 ) )
//        {
//            new_joint = new Joint();
//            new_joint->index = joints.size();
//            new_joint->parent = joint;
//            new_joint->has_site = false;
//            new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
//            new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;
//            joints.push_back( new_joint );
//            if ( joint )
//                joint->children.push_back( new_joint );
//
//            token = strtok( NULL, "" );
//            while ( *token == ' ' )  token ++;
//            new_joint->name = token;
//
//            joint_index[ new_joint->name ] = new_joint;
//            continue;
//        }
        if ( ( strcmp( token, "ROOT" ) == 0 ) ||
             ( strcmp( token, "JOINT" ) == 0 ))
        {
            new_joint = new Joint();
            new_joint->index = joints.size();
            new_joint->parent = joint;
            new_joint->has_site = false;
            new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
            new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;
            joints.push_back( new_joint );
            if ( joint )
                joint->children.push_back( new_joint );

            token = strtok( NULL, "" );
            while ( *token == ' ' )  token ++;
            new_joint->name = token;

            joint_index[ new_joint->name ] = new_joint;
            continue;
        }
        if ( ( strcmp( token, "End" ) == 0 ))
        {
            new_joint = new Joint();
            new_joint->index = joints.size();
            new_joint->parent = joint;
            new_joint->has_site = true;
            new_joint->offset[0] = 0.0;  new_joint->offset[1] = 0.0;  new_joint->offset[2] = 0.0;
            new_joint->site[0] = 0.0;  new_joint->site[1] = 0.0;  new_joint->site[2] = 0.0;
            joints.push_back( new_joint );
            if ( joint )
                joint->children.push_back( new_joint );

            token = strtok( NULL, "" );
            while ( *token == ' ' )  token ++;
            new_joint->name = token;

            joint_index[ new_joint->name ] = new_joint;
            continue;
        }
        if ( strcmp( token, "OFFSET" ) == 0 )
        {
            token = strtok( NULL, separater );
            x = token ? atof( token ) : 0.0;
            token = strtok( NULL, separater );
            y = token ? atof( token ) : 0.0;
            token = strtok( NULL, separater );
            z = token ? atof( token ) : 0.0;

//            if ( is_site )
//            {
//                joint->has_site = true;
//                joint->site[0] = x;
//                joint->site[1] = y;
//                joint->site[2] = z;
//            }
//            else
//            {
//                joint->offset[0] = x;
//                joint->offset[1] = y;
//                joint->offset[2] = z;
//            }
            joint->offset[0] = x;
            joint->offset[1] = y;
            joint->offset[2] = z;
            continue;
        }

        if ( strcmp( token, "CHANNELS" ) == 0 )
        {
            token = strtok( NULL, separater );
            joint->channels.resize( token ? atoi( token ) : 0 );

            for ( i=0; i<joint->channels.size(); i++ )
            {
                Channel *  channel = new Channel();
                channel->joint = joint;
                channel->index = channels.size();
                
                channels.push_back( channel );
                joint->channels[ i ] = channel;

                token = strtok( NULL, separater );
                if ( strcmp( token, "Xrotation" ) == 0 )
                    channel->type = X_ROTATION;
                else if ( strcmp( token, "Yrotation" ) == 0 )
                    channel->type = Y_ROTATION;
                else if ( strcmp( token, "Zrotation" ) == 0 )
                    channel->type = Z_ROTATION;
                else if ( strcmp( token, "Xposition" ) == 0 )
                    channel->type = X_POSITION;
                else if ( strcmp( token, "Yposition" ) == 0 )
                    channel->type = Y_POSITION;
                else if ( strcmp( token, "Zposition" ) == 0 )
                    channel->type = Z_POSITION;
            }
        }

        if ( strcmp( token, "MOTION" ) == 0 )
            break;
    }

    file.getline( line, BUFFER_LENGTH );
    token = strtok( line, separater );
    if ( strcmp( token, "Frames" ) != 0 )
        goto bvh_error;
    token = strtok( NULL, separater );
    if ( token == NULL )
        goto bvh_error;
    num_frame = atoi( token );

    file.getline( line, BUFFER_LENGTH );
    token = strtok( line, ":" );
    if ( strcmp( token, "Frame Time" ) != 0 )
        goto bvh_error;
    token = strtok( NULL, separater );
    if ( token == NULL )
        goto bvh_error;
    interval = atof( token );

    num_channel = channels.size();
    motion = new double[ num_frame * num_channel ];

    for ( i=0; i<num_frame; i++ )
    {
        file.getline( line, BUFFER_LENGTH );
        token = strtok( line, separater );
        for ( j=0; j<num_channel; j++ )
        {
            if ( token == NULL )
                goto bvh_error;
            motion[ i*num_channel + j ] = atof( token );
            token = strtok( NULL, separater );
        }
    }

    file.close();

    is_load_success = true;
    if(is_load_success) cout << "load complete" << endl;

    return;

bvh_error:
    file.close();
}

void  BVH::UpdatePose( int frame_num, Body &body, float scale )
{
    UpdatePose( joints[0], motion + frame_num * num_channel, body, scale );
//    for(int i = 0; i < frame_size; i++){
//        UpdatePose( joints[0], motion + i * num_channel, body, scale );
//
//    }
    
}


void  BVH::UpdatePose( Joint * joint, const double * data, Body &body, float scale)
{
    Joint *parent = joint->parent;
//    glm::vec3 pos = joint->position;

    joint->quat = glm::quat(1,0,0,0);
    
    for ( int i=0; i < joint->channels.size(); i++ )
    {
        Channel *  channel = joint->channels[ i ];
        
        if ( channel->type == X_ROTATION )
            joint->quat *= glm::exp(float(data[channel->index])*PI/180/2 * quat(0,1,0,0));
        else if ( channel->type == Y_ROTATION )
            joint->quat *= glm::exp(float(data[channel->index])*PI/180/2 * quat(0,0,1,0));
        else if ( channel->type == Z_ROTATION )
            joint->quat *= glm::exp(float(data[channel->index])*PI/180/2 * quat(0,0,0,1));
    }

    if ( parent == NULL )
    {
        joint->position = scale * glm::vec3(data[ 0 ], data[ 1 ], data[ 2 ] );
        body.add(-1, joint->children[0]->index , joint->position, joint->quat, vec3(0), quat(1,0,0,0), joint->has_site);
    }
    else if( joint->has_site ){
        joint->quat = quat(1,0,0,0);
        glm::vec3 offset = scale * glm::vec3(joint->offset[ 0 ], joint->offset[ 1 ], joint->offset[ 2 ]);
        joint->position = parent->quat * offset + parent->position;
        body.add(parent->index, -1, offset, joint->quat, parent->position, parent->quat, joint->has_site);
    }
    else
    {
//        joint->quat = parent->quat * joint->quat;
        glm::vec3 offset = scale * glm::vec3(joint->offset[ 0 ], joint->offset[ 1 ], joint->offset[ 2 ]);
        joint->position = parent->quat * offset + parent->position;
        
        body.add(parent->index, joint->children[0]->index, offset, joint->quat, parent->position, parent->quat, joint->has_site);
        
//        if(joint->children.size())
//            body.add(parent->index, joint->children[0]->index, offset, joint->quat, parent->position, parent->quat, joint->has_site);
//        else
//            body.add(parent->index, -1, joint->position, offset, parent->position, parent->quat, joint->has_site);
    }
//    
//    if(joint->parent){
//    cout
//    << joint->name
//    << " "
//    << joint->index
////    << " parent "
////    << joint->parent->name
//    << " child "
//    << joint->children.size()
//    << " link "
//    << joint->position.x
//    << " "
//    << joint->position.y
//    << " "
//    << joint->position.z
//    << " orientation "
//    << joint->quat.x
//    << " "
//    << joint->quat.y
//    << " "
//    << joint->quat.z
//    << endl;
//    }
//    
//    if(joint->parent != NULL)
//        RenderPose(pos, parent->position, 1);
    
    for ( int i=0; i<joint->children.size(); i++ )
    {
        UpdatePose( joint->children[ i ], data, body, scale );
    }
}

void  BVH::RenderPose(glm::vec3 pos, glm::vec3 parentPos, float size)
{
    drawSphere(pos, size, glm::vec4(1,1,0,.1));
    drawCylinder(pos,parentPos,1);
}

//void  BVH::RenderPose( int frame_no, float size){
//    for( int i = 0; i < joints_by_frame[frame_no].size(); i++){
//        drawSphere(joints_by_frame[frame_no][i]->position, size, glm::vec4(1,1,0,.1));
//        if(joints_by_frame[frame_no][i]->parent)
//        drawCylinder(joints_by_frame[frame_no][i]->position,joints_by_frame[frame_no][i]->parent->position,1);
//    }
//}

