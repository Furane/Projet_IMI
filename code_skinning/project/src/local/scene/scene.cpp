

#include <GL/glew.h>

#include "scene.hpp"
#include "../../lib/opengl/glutils.hpp"

#include "../../lib/perlin/perlin.hpp"
#include "../../lib/interface/camera_matrices.hpp"

#include "../interface/myWidgetGL.hpp"

#include <cmath>


#include <string>
#include <sstream>
#include "../../lib/mesh/mesh_io.hpp"

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
    {printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

using namespace cpe;


static cpe::mesh build_ground(float const L,float const h)
{
    mesh m;
    m.add_vertex(vec3(-L, h,-L));
    m.add_vertex(vec3(-L, h, L));
    m.add_vertex(vec3( L, h, L));
    m.add_vertex(vec3( L, h,-L));

    m.add_triangle_index({0,2,1});
    m.add_triangle_index({0,3,2});

    m.fill_color(vec3(0.8,0.9,0.8));

    return m;
}


static cpe::mesh_skinned build_cylindre(float radius, float length, int rad_samples, int axe_samples)
{
    mesh_skinned m;

    for(int i=0;i<rad_samples;++i)
    {
        float const u = float(i)/(rad_samples);

        for(int j=0;j<axe_samples;++j)
        {
            float const v = float(j)/(axe_samples-1);

            float const x = radius*cos(2.0f*M_PI*u);
            float const y = radius*sin(2.0f*M_PI*u);
            float const z = length*v;

            m.add_vertex({x,y,z});


            float const w0 = v;
            float const w1 = 1-w0;
            vertex_weight_parameter weight_parameter;

            weight_parameter[0].joint_id = 0;
            weight_parameter[0].weight = w1;
            weight_parameter[1].joint_id = 1;
            weight_parameter[1].weight = w0;

            /*if(v<0.5f)
            {
                weight_parameter[0].weight = 1;
                weight_parameter[1].weight = 0;
            }
            else
            {
                weight_parameter[0].weight = 0;
                weight_parameter[1].weight = 1;
            }*/


            m.add_vertex_weight(weight_parameter);

        }
    }

    for(int i=0;i<rad_samples;++i)
    {
        for(int j=0;j<axe_samples-1;++j)
        {
            int const k0 = axe_samples*i+j;
            int const k1 = axe_samples*((i+1)%rad_samples)+j;
            int const k2 = axe_samples*((i+1)%rad_samples)+(j+1);
            int const k3 = axe_samples*(i+0)+(j+1);

            m.add_triangle_index({k0,k1,k2});
            m.add_triangle_index({k0,k2,k3});
        }
    }

    //m.fill_color(vec3(0.8,0.9,0.8));

    return m;
}

void init_cylinder_skeleton(cpe::skeleton_parent_id& sk_cylinder_parent_id, cpe::skeleton_geometry& sk_cylinder_bind_pose)
{
    float length = 50.0f;

    sk_cylinder_parent_id.push_back(-1);
    sk_cylinder_parent_id.push_back(0);
    sk_cylinder_parent_id.push_back(1);

    sk_cylinder_bind_pose.push_back(skeleton_joint({0,0,0},{0,0,0,1}));
    sk_cylinder_bind_pose.push_back(skeleton_joint({0,0,length/2.0f},{0,0,0,1}));
    sk_cylinder_bind_pose.push_back(skeleton_joint({0,0,length/2.0f},{0,0,0,1}));
}

void cylinder_animation( cpe::skeleton_geometry& sk_cylinder_bind_pose, skeleton_animation& sk_cylinder_animation)
{
    cpe::skeleton_geometry position0, position1, position2, position3;
    quaternion q1, q2, q3;

    position0 = sk_cylinder_bind_pose;
    position1 = sk_cylinder_bind_pose;
    position2 = sk_cylinder_bind_pose;
    position3 = sk_cylinder_bind_pose;

    q1.set_axis_angle({1,0,0},M_PI/6.0f);
    q2.set_axis_angle({1,0,0},M_PI/3.0f);
    q3.set_axis_angle({1,0,0},M_PI/2.0f);

    position1[1].orientation = q1;
    position2[1].orientation = q2;
    position3[1].orientation = q3;

    sk_cylinder_animation.push_back(position0);
    sk_cylinder_animation.push_back(position1);
    sk_cylinder_animation.push_back(position2);
    sk_cylinder_animation.push_back(position3);
}

void scotty_animation( cpe::skeleton_geometry& sk_scotty_bind_pose, skeleton_animation& sk_scotty_animation)
{
    cpe::skeleton_geometry position0, position1, position2, position3;
    quaternion q1, q2, q3;

    position0 = sk_scotty_bind_pose;
    position1 = sk_scotty_bind_pose;
    position2 = sk_scotty_bind_pose;
    position3 = sk_scotty_bind_pose;

    q1.set_axis_angle({0,0,1},M_PI/6.0f);
    q2.set_axis_angle({0,0,1},M_PI/3.0f);
    q3.set_axis_angle({0,0,1},M_PI/2.0f);

    position1[13].orientation = q1;
    position2[13].orientation = q2;
    position3[13].orientation = q3;

    sk_scotty_animation.push_back(position0);
    sk_scotty_animation.push_back(position1);
    sk_scotty_animation.push_back(position2);
    sk_scotty_animation.push_back(position3);
}

/*void init_scotty_weight(cpe::mesh_skinned& mesh,cpe::skeleton_geometry sk_scotty_bind_pose)
{
    int size_mesh = mesh.size_vertex();

    vertex_weight_parameter weight_parameter;

    //parcourt des points du mesh
    for(int i=0;i<size_mesh;++i)
    {
        vec3 vertex_position = mesh.vertex_original(i);


    }
}*/

void updateUserState(const nite::UserData& user, unsigned long long ts)
{
    if (user.isNew())
        USER_MESSAGE("New")
    else if (user.isVisible() && !g_visibleUsers[user.getId()])
        USER_MESSAGE("Visible")
    else if (!user.isVisible() && g_visibleUsers[user.getId()])
        USER_MESSAGE("Out of Scene")
    else if (user.isLost())
        USER_MESSAGE("Lost")

    g_visibleUsers[user.getId()] = user.isVisible();


    if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
    {
        switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
        {
        case nite::SKELETON_NONE:
            USER_MESSAGE("Stopped tracking.")
            break;
        case nite::SKELETON_CALIBRATING:
            USER_MESSAGE("Calibrating...")
            break;
        case nite::SKELETON_TRACKED:
            USER_MESSAGE("Tracking!")
            break;
        case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
        case nite::SKELETON_CALIBRATION_ERROR_HANDS:
        case nite::SKELETON_CALIBRATION_ERROR_LEGS:
        case nite::SKELETON_CALIBRATION_ERROR_HEAD:
        case nite::SKELETON_CALIBRATION_ERROR_TORSO:
            USER_MESSAGE("Calibration Failed... :-|")
            break;
        }
    }
}

void scene::get_kinect_skeleton()
{
    float scale = 300.0f;//250.0f

    niteRc = userTracker.readFrame(&userTrackerFrame);
    if (niteRc != nite::STATUS_OK)
    {
        printf("Get next frame failed\n");
        return;
    }

    const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
    for (int i = 0; i < users.getSize(); ++i)
    {
        const nite::UserData& user = users[i];
        updateUserState(user,userTrackerFrame.getTimestamp());
        if (user.isNew())
        {
            userTracker.startSkeletonTracking(user.getId());
        }
        else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
        {
            skeleton_joint sk_center_hip, sk_left_hip, sk_right_hip, sk_head, sk_neck, sk_left_shoulder, sk_right_shoulder, sk_left_elbow,
                    sk_right_elbow, sk_left_hand, sk_right_hand, sk_torso, sk_left_knee, sk_right_knee, sk_left_foot, sk_right_foot;

            const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
            //if (head.getPositionConfidence() > .5)
                sk_head = skeleton_joint({head.getPosition().x/scale, head.getPosition().y/scale, -head.getPosition().z/scale},
                {head.getOrientation().x, head.getOrientation().y, head.getOrientation().z, head.getOrientation().w});//head.getPosition().z

            const nite::SkeletonJoint& neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
            //if (neck.getPositionConfidence() > .5)
                sk_neck = skeleton_joint({neck.getPosition().x/scale, neck.getPosition().y/scale, -neck.getPosition().z/scale},
                {neck.getOrientation().x, neck.getOrientation().y, neck.getOrientation().z, neck.getOrientation().w});//neck.getPosition().z
            const nite::SkeletonJoint& left_shoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
            //if (left_shoulder.getPositionConfidence() > .5)
                sk_left_shoulder = skeleton_joint({left_shoulder.getPosition().x/scale, left_shoulder.getPosition().y/scale, -left_shoulder.getPosition().z/scale},
                {left_shoulder.getOrientation().x, left_shoulder.getOrientation().y, left_shoulder.getOrientation().z, left_shoulder.getOrientation().w});//left_shoulder.getPosition().z

            const nite::SkeletonJoint& right_shoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
            //if (right_shoulder.getPositionConfidence() > .5)
                sk_right_shoulder = skeleton_joint({right_shoulder.getPosition().x/scale, right_shoulder.getPosition().y/scale, -right_shoulder.getPosition().z/scale},
                {right_shoulder.getOrientation().x, right_shoulder.getOrientation().y, right_shoulder.getOrientation().z, right_shoulder.getOrientation().w});//right_shoulder.getPosition().z

            const nite::SkeletonJoint& left_elbow = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
            //if (left_elbow.getPositionConfidence() > .5)
                sk_left_elbow = skeleton_joint({left_elbow.getPosition().x/scale, left_elbow.getPosition().y/scale, -left_elbow.getPosition().z/scale},
                {left_elbow.getOrientation().x, left_elbow.getOrientation().y, left_elbow.getOrientation().z, left_elbow.getOrientation().w});//left_elbow.getPosition().z

            const nite::SkeletonJoint& right_elbow = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
            //if (right_elbow.getPositionConfidence() > .5)
                sk_right_elbow = skeleton_joint({right_elbow.getPosition().x/scale, right_elbow.getPosition().y/scale, -right_elbow.getPosition().z/scale},
                {right_elbow.getOrientation().x, right_elbow.getOrientation().y, right_elbow.getOrientation().z, right_elbow.getOrientation().w});//right_elbow.getPosition().z

            const nite::SkeletonJoint& left_hand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
            //if (left_hand.getPositionConfidence() > .5)
                sk_left_hand = skeleton_joint({left_hand.getPosition().x/scale, left_hand.getPosition().y/scale, -left_hand.getPosition().z/scale},
                {left_hand.getOrientation().x, left_hand.getOrientation().y, left_hand.getOrientation().z, left_hand.getOrientation().w});//left_hand.getPosition().z

            const nite::SkeletonJoint& right_hand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
            //if (right_hand.getPositionConfidence() > .5)
                sk_right_hand = skeleton_joint({right_hand.getPosition().x/scale, right_hand.getPosition().y/scale, -right_hand.getPosition().z/scale},
                {right_hand.getOrientation().x, right_hand.getOrientation().y, right_hand.getOrientation().z, right_hand.getOrientation().w});//right_hand.getPosition().z

            const nite::SkeletonJoint& torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);
            //if (torso.getPositionConfidence() > .5)
                sk_torso = skeleton_joint({torso.getPosition().x/scale, torso.getPosition().y/scale, -torso.getPosition().z/scale},
                {torso.getOrientation().x, torso.getOrientation().y, torso.getOrientation().z, torso.getOrientation().w});//torso.getPosition().z

            const nite::SkeletonJoint& left_hip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
            //if (left_hip.getPositionConfidence() > .5)
                sk_left_hip = skeleton_joint({left_hip.getPosition().x/scale, left_hip.getPosition().y/scale, -left_hip.getPosition().z/scale},
                {left_hip.getOrientation().x, left_hip.getOrientation().y, left_hip.getOrientation().z, left_hip.getOrientation().w});//left_hip.getPosition().z

            const nite::SkeletonJoint& right_hip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
            //if (right_hip.getPositionConfidence() > .5)
                sk_right_hip = skeleton_joint({right_hip.getPosition().x/scale, right_hip.getPosition().y/scale, -right_hip.getPosition().z/scale},
                {right_hip.getOrientation().x, right_hip.getOrientation().y, right_hip.getOrientation().z, right_hip.getOrientation().w});//right_hip.getPosition().z

            const nite::SkeletonJoint& left_knee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
            //if (left_knee.getPositionConfidence() > .5)
                sk_left_knee = skeleton_joint({left_knee.getPosition().x/scale, left_knee.getPosition().y/scale, -left_knee.getPosition().z/scale},
                {left_knee.getOrientation().x, left_knee.getOrientation().y, left_knee.getOrientation().z, left_knee.getOrientation().w});//left_knee.getPosition().z

            const nite::SkeletonJoint& right_knee = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);
            //if (right_knee.getPositionConfidence() > .5)
                sk_right_knee = skeleton_joint({right_knee.getPosition().x/scale, right_knee.getPosition().y/scale, -right_knee.getPosition().z/scale},
                {right_knee.getOrientation().x, right_knee.getOrientation().y, right_knee.getOrientation().z, right_knee.getOrientation().w});//right_knee.getPosition().z

            const nite::SkeletonJoint& left_foot = user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT);
            //if (left_foot.getPositionConfidence() > .5)
                sk_left_foot = skeleton_joint({left_foot.getPosition().x/scale, left_foot.getPosition().y/scale, -left_foot.getPosition().z/scale},
                {left_foot.getOrientation().x, left_foot.getOrientation().y, left_foot.getOrientation().z, left_foot.getOrientation().w});//left_foot.getPosition().z

            const nite::SkeletonJoint& right_foot = user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT);
            //if (right_foot.getPositionConfidence() > .5)
                sk_right_foot = skeleton_joint({right_foot.getPosition().x/scale, right_foot.getPosition().y/scale, -right_foot.getPosition().z/scale},
                {right_foot.getOrientation().x, right_foot.getOrientation().y, right_foot.getOrientation().z, right_foot.getOrientation().w});//right_foot.getPosition().z

            sk_center_hip = skeleton_joint((sk_left_hip.position+sk_right_hip.position)/2.0f,(sk_left_hip.orientation+sk_right_hip.orientation)/2.0f);
            //sk_center_hip.position.z()=0;

            sk_kinect[0]= sk_center_hip;
            sk_kinect[1]= sk_torso;
            sk_kinect[2]= sk_left_hip;
            sk_kinect[3]= sk_right_hip;
            sk_kinect[4]= sk_neck;
            sk_kinect[5]= sk_left_knee;
            sk_kinect[6]= sk_right_knee;
            sk_kinect[7]= sk_left_shoulder;
            sk_kinect[8]= sk_head;
            sk_kinect[9]= sk_right_shoulder;
            sk_kinect[10]= sk_left_foot;
            sk_kinect[11]= sk_right_foot;
            sk_kinect[12]= sk_left_elbow;
            sk_kinect[13]= sk_right_elbow;
            sk_kinect[14]= sk_left_hand;
            sk_kinect[15]= sk_right_hand;
        }
    }

}

void scene::load_scene()
{

    //*****************************************//
    // Preload default structure               //
    //*****************************************//
    texture_default = load_texture_file("data/white.jpg");
    shader_mesh     = read_shader("shaders/shader_mesh.vert",
                                  "shaders/shader_mesh.frag");           PRINT_OPENGL_ERROR();
    shader_skeleton = read_shader("shaders/shader_skeleton.vert",
                                  "shaders/shader_skeleton.frag");       PRINT_OPENGL_ERROR();


    //*****************************************//
    // Build ground
    //*****************************************//
    mesh_ground = build_ground(100.0f , -25.0f);
    mesh_ground.fill_empty_field_by_default();
    mesh_ground_opengl.fill_vbo(mesh_ground);

    //*****************************************//
    // Build cylinder
    //*****************************************//
    /*mesh_cylinder = build_cylindre(4.0f, 50.0f, 100, 100);
    mesh_cylinder.fill_empty_field_by_default();
    mesh_cylinder_opengl.fill_vbo(mesh_cylinder);

    init_cylinder_skeleton(sk_cylinder_parent_id, sk_cylinder_bind_pose);
    cylinder_animation(sk_cylinder_bind_pose,sk_cylinder_animation);

    pose = 0;
    time.start();

    cat_pose=0;
    cat_time.start();*/

    //*****************************************//
    // Load cat
    //*****************************************//
    /*mesh_cat.load("data/cat.obj");
    texture_cat=load_texture_file("data/cat.png");
    mesh_cat.fill_empty_field_by_default();
    mesh_cat_opengl.fill_vbo(mesh_cat);*/

    /*sk_cat_bind_pose.load("data/cat_bind_pose.skeleton");
    sk_cat_parent_id.load("data/cat_bind_pose.skeleton");*/
    //sk_cat_animation.load("data/cat.animations",sk_cat_parent_id.size());

    //*****************************************//
    // Load Rigged Stick Figure
    //*****************************************//
    mesh_scotty.load("data/blender_mesh/Scotty_skinning_heavy.obj");//StickFigurea.obj  Scotty.obj Scotty_part.obj
    //texture_stick=load_texture_file("data/StickFigurea/cat.png");
    mesh_scotty.fill_empty_field_by_default();
    mesh_scotty_opengl.fill_vbo(mesh_scotty);

    sk_scotty_bind_pose.load("data/blender_mesh/scotty_bind_pose_bras_etendu.skeleton");//left_arm_bind_pose
    sk_scotty_parent_id.load("data/blender_mesh/scotty_bind_pose_bras_etendu.skeleton");//left_arm_bind_pose
    /*scotty_animation(sk_scotty_bind_pose,sk_scotty_animation);
    sk_scotty_animation.load("data/scotty.animations",sk_scotty_parent_id.size());*/

    //*****************************************//
    // Run Nite
    //*****************************************//
    nite::NiTE::initialize();

    niteRc = userTracker.create();
    if (niteRc != nite::STATUS_OK)
    {
        printf("Couldn't create user tracker\n");
        return;
    }
    printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

    sk_kinect = local_to_global(sk_scotty_bind_pose,sk_scotty_parent_id);
}

void scene::draw_scene()
{

    setup_shader_skeleton(shader_skeleton);

    //Here we can draw skeletons as 3D segments

    setup_shader_mesh(shader_mesh);

    mesh_ground_opengl.draw();
    //mesh_cylinder_opengl.draw();

    /*int periode = 300;

    if(time.elapsed()>periode)
    {
        pose++;
        pose = pose%sk_cylinder_animation.size();
        time.restart();
    }

    alpha = time.elapsed()/float(periode);

    int cat_periode = 300;

    if(cat_time.elapsed()>periode)
    {
        cat_pose++;
        cat_pose = cat_pose%sk_cat_animation.size();
        cat_time.restart();
    }

    cat_alpha = cat_time.elapsed()/float(cat_periode);*/

    /*skeleton_geometry const sk_cylinder_global = local_to_global(sk_cylinder_animation(pose,alpha),sk_cylinder_parent_id);
    std::vector<vec3> const sk_cylinder_bones = extract_bones(sk_cylinder_global,sk_cylinder_parent_id);
    draw_skeleton(sk_cylinder_bones);

    skeleton_geometry const sk_cylinder_inverse_bind_pose = inversed(sk_cylinder_bind_pose);
    skeleton_geometry const sk_cylinder_binded = multiply(sk_cylinder_global,sk_cylinder_inverse_bind_pose);
    mesh_cylinder.apply_skinning(sk_cylinder_binded);
    mesh_cylinder.fill_normal();
    mesh_cylinder_opengl.update_vbo_vertex(mesh_cylinder);
    mesh_cylinder_opengl.update_vbo_normal(mesh_cylinder);
    mesh_cylinder_opengl.draw();

    skeleton_geometry const sk_cat_global = local_to_global(sk_cat_animation(cat_pose,cat_alpha),sk_cat_parent_id);//sk_cat_animation(cat_pose,cat_alpha)
    std::vector<vec3> const sk_cat_bones = extract_bones(sk_cat_global,sk_cat_parent_id);//sk_cat_global
    draw_skeleton(sk_cat_bones);

    glBindTexture(GL_TEXTURE_2D,texture_cat);
    skeleton_geometry const sk_cat_inverse_bind_pose = inversed(sk_cat_bind_pose);
    skeleton_geometry const sk_cat_binded = multiply(sk_cat_global,sk_cat_inverse_bind_pose);
    mesh_cat.apply_skinning(sk_cat_binded);
    mesh_cat.fill_normal();
    mesh_cat_opengl.update_vbo_vertex(mesh_cat);
    mesh_cat_opengl.update_vbo_normal(mesh_cat);
    mesh_cat_opengl.draw();*/

    /*skeleton_geometry const sk_scotty_global = local_to_global(sk_scotty_animation(pose,alpha),sk_scotty_parent_id);
    std::vector<vec3> const sk_scotty_bones = extract_bones(sk_scotty_global,sk_scotty_parent_id);//sk_cat_global
    draw_skeleton(sk_scotty_bones);*/

    //glBindTexture(GL_TEXTURE_2D,texture_cat);
    /*skeleton_geometry const sk_scotty_inverse_bind_pose = inversed(local_to_global(sk_scotty_bind_pose,sk_scotty_parent_id));
    skeleton_geometry const sk_scotty_binded = multiply(sk_scotty_global,sk_scotty_inverse_bind_pose);
    mesh_scotty.apply_skinning(sk_scotty_binded);
    mesh_scotty.fill_normal();
    mesh_scotty_opengl.update_vbo_vertex(mesh_scotty);
    mesh_scotty_opengl.update_vbo_normal(mesh_scotty);
    mesh_scotty_opengl.draw();*/

    get_kinect_skeleton();
    std::vector<vec3> const sk_kinect_bones = extract_bones(sk_kinect,sk_scotty_parent_id);

    if(draw_sk)
        draw_skeleton(sk_kinect_bones);

    skeleton_geometry const sk_scotty_inverse_bind_pose = inversed(local_to_global(sk_scotty_bind_pose,sk_scotty_parent_id));//local_to_global(sk_scotty_bind_pose,sk_scotty_parent_id)
    skeleton_geometry const sk_scotty_binded = multiply(sk_kinect,sk_scotty_inverse_bind_pose);
    mesh_scotty.apply_skinning(sk_scotty_binded);
    mesh_scotty.fill_normal();
    mesh_scotty_opengl.update_vbo_vertex(mesh_scotty);
    mesh_scotty_opengl.update_vbo_normal(mesh_scotty);

    if(draw_mesh)
        mesh_scotty_opengl.draw();
}


void scene::setup_shader_mesh(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"normal_matrix"),1,false,cam.normal.pointer());           PRINT_OPENGL_ERROR();

    //load white texture
    glBindTexture(GL_TEXTURE_2D,texture_default);                                                      PRINT_OPENGL_ERROR();
    glLineWidth(1.0f);                                                                                 PRINT_OPENGL_ERROR();

}

void scene::setup_shader_skeleton(GLuint const shader_id)
{
    //Setup uniform parameters
    glUseProgram(shader_id);                                                                           PRINT_OPENGL_ERROR();

    //Get cameras parameters (modelview,projection,normal).
    camera_matrices const& cam=pwidget->camera();

    //Set Uniform data to GPU
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_modelview"),1,false,cam.modelview.pointer());     PRINT_OPENGL_ERROR();
    glUniformMatrix4fv(get_uni_loc(shader_id,"camera_projection"),1,false,cam.projection.pointer());   PRINT_OPENGL_ERROR();
    glUniform3f(get_uni_loc(shader_id,"color") , 0.0f,0.0f,0.0f);                                      PRINT_OPENGL_ERROR();

    //size of the lines
    glLineWidth(3.0f);                                                                                 PRINT_OPENGL_ERROR();
}

void scene::draw_skeleton(std::vector<vec3> const& positions) const
{
    // Create temporary a VBO to store data
    GLuint vbo_skeleton=0;
    glGenBuffers(1,&vbo_skeleton);                                                                     PRINT_OPENGL_ERROR();
    glBindBuffer(GL_ARRAY_BUFFER,vbo_skeleton);                                                        PRINT_OPENGL_ERROR();

    // Update data on the GPU
    glBufferData(GL_ARRAY_BUFFER , sizeof(vec3)*positions.size() , &positions[0] , GL_STATIC_DRAW);    PRINT_OPENGL_ERROR();

    // Draw data
    glEnableClientState(GL_VERTEX_ARRAY);                                                              PRINT_OPENGL_ERROR();
    glVertexPointer(3, GL_FLOAT, 0, 0);                                                                PRINT_OPENGL_ERROR();
    glDrawArrays(GL_LINES,0,positions.size());                                                         PRINT_OPENGL_ERROR();

    // Delete temporary VBO
    glDeleteBuffers(1,&vbo_skeleton);                                                                  PRINT_OPENGL_ERROR();
}

scene::scene()
    :shader_mesh(0)
{}


GLuint scene::load_texture_file(std::string const& filename)
{
    return pwidget->load_texture_file(filename);
}

void scene::set_widget(myWidgetGL* widget_param)
{
    pwidget=widget_param;
}

void scene::set_draw_skeleton(bool const is_skeleton)
{
    draw_sk=is_skeleton;
}

void scene::set_draw_mesh(bool const is_mesh)
{
    draw_mesh=is_mesh;
}


