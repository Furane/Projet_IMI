/*
**    TP CPE Lyon
**    Copyright (C) 2015 Damien Rohmer
**
**    This program is free software: you can redistribute it and/or modify
**    it under the terms of the GNU General Public License as published by
**    the Free Software Foundation, either version 3 of the License, or
**    (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**    but WITHOUT ANY WARRANTY; without even the implied warranty of
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**    GNU General Public License for more details.
**
**    You should have received a copy of the GNU General Public License
**    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "mesh_skinned.hpp"

#include "../lib/common/error_handling.hpp"
#include "../lib/mesh/mesh_io.hpp"
#include "skeleton_geometry.hpp"

#include <sstream>
#include <fstream>


namespace cpe
{

int mesh_skinned::size_vertex_weight() const
{
    return vertex_weight_data.size();
}

vertex_weight_parameter const& mesh_skinned::vertex_weight(int const index) const
{
    ASSERT_CPE(index>=0,"Index ("+std::to_string(index)+") must be positive");
    ASSERT_CPE(index<int(vertex_weight_data.size()) , "Index ("+std::to_string(index)+") must be less than the current size of the weight vector ("+std::to_string(vertex_weight_data.size())+")");

    return vertex_weight_data[index];
}
vertex_weight_parameter& mesh_skinned::vertex_weight(int const index)
{
    ASSERT_CPE(index>=0,"Index ("+std::to_string(index)+") must be positive");
    ASSERT_CPE(index<int(vertex_weight_data.size()),"Index ("+std::to_string(index)+") must be less than the current size of the weight vector ("+std::to_string(vertex_weight_data.size())+")");

    return vertex_weight_data[index];
}

void mesh_skinned::add_vertex_weight(vertex_weight_parameter const& w)
{
    vertex_weight_data.push_back(w);
}


void mesh_skinned::load(std::string const& filename)
{
    //Warning: Can only handle meshes with same connectivity for vertex and textures
    //(Format de fichier de David Odin)

    std::ifstream fid(filename.c_str());
    if(!fid.good())
        throw exception_cpe("Cannot open file "+filename,MACRO_EXCEPTION_PARAMETER);

    std::string buffer;

    std::vector<vertex_weight_parameter> skinning_info;

    vertex_weight_parameter init_skinning;
    vertex_weight_parameter temp_skinning;
    int const N_bone = temp_skinning.size();

    vec3 color_bone(0,0,0);

    for(int k_bone=0 ; k_bone<N_bone ; ++k_bone)
    {
        init_skinning[k_bone].joint_id = 0;
        init_skinning[k_bone].weight = 0;
    }

    //read the whole file
    while(fid.good()==true)
    {
        //read line
        std::getline(fid,buffer);

        if(buffer.size()>0)
        {
            std::stringstream tokens(buffer);
            std::string first_word;
            std::string skeleton_part;
            tokens >> first_word;

            //skip comments
            if(first_word.length()>0 && first_word[0]!='#')
            {
                if(first_word=="g")
                {
                    //nb_vertex_part = nb_vertex;
                    tokens >> skeleton_part;
                    if(skeleton_part == "ventre")
                    {
                        color_bone = vec3(1.0f,1.0f,0.0f);
                        temp_skinning[0].joint_id = 0;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "buste")
                    {
                        color_bone = vec3(1.0f,0.75f,0.0f);
                        temp_skinning[0].joint_id = 1;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "cuisse_r")
                    {
                        color_bone = vec3(0,0,1);
                        temp_skinning[0].joint_id = 2;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "cuisse_l")
                    {
                        color_bone = vec3(0,0,1);
                        temp_skinning[0].joint_id = 3;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "tete")
                    {
                        color_bone = vec3(1.0f,0.75f,0.75f);
                        temp_skinning[0].joint_id = 4;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "bas_jambe_r")
                    {
                        color_bone = vec3(0,1,1);
                        temp_skinning[0].joint_id = 5;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "bas_jambe_l")
                    {
                        color_bone = vec3(0,1,1);
                        temp_skinning[0].joint_id = 6;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "bras_r")
                    {
                        color_bone = vec3(0.5f,0.5f,0.5f);
                        temp_skinning[0].joint_id = 7;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "bras_l")
                    {
                        color_bone = vec3(0,1,0);
                        temp_skinning[0].joint_id = 9;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "main_r")
                    {
                        color_bone = vec3(0.75f,0.75f,0.75f);
                        temp_skinning[0].joint_id = 12;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                    if(skeleton_part == "main_l")
                    {
                        color_bone = vec3(0.85f,0.85f,0.85f);
                        temp_skinning[0].joint_id = 13;
                        temp_skinning[0].weight = 1;
                        //on met les autres bones a 0
                        for(int k_bone=1 ; k_bone<N_bone ; ++k_bone)
                        {
                            temp_skinning[k_bone].joint_id = 0;
                            temp_skinning[k_bone].weight = 0;
                        }
                    }
                }
                //vertices
                if(first_word=="v")
                {
                  vec3 vertex;
                  tokens >> vertex.x();
                  tokens >> vertex.y();
                  tokens >> vertex.z();

                  add_vertex(vertex);
                  add_color(color_bone);
                  skinning_info.push_back(init_skinning);//on ajoute le poids initialise à 0
                }

                //texture
                if(first_word=="vt")
                {
                  vec2 texture;
                  tokens >> texture.x();
                  tokens >> texture.y();

                  add_texture_coord(texture);
                }

                // normal
                if(first_word=="vn")
                {
                  vec3 normal;
                  tokens >> normal.x();
                  tokens >> normal.y();
                  tokens >> normal.z();

                  add_normal(normal);
                }

                //skinning
                /*if(first_word=="sk")
                {
                    vertex_weight_parameter temp_skinning;
                    int const N_bone = temp_skinning.size();
                    for(int k_bone=0 ; k_bone<N_bone ; ++k_bone)
                    {
                        tokens >> temp_skinning[k_bone].joint_id;
                        tokens >> temp_skinning[k_bone].weight;
                    }
                    skinning_info.push_back(temp_skinning);
                }*/


                //read connectivity
                if(first_word=="f")
                {
                  std::string u0_str,u1_str,u2_str;
                  tokens >> u0_str >> u1_str >> u2_str;

                  int const u0 = std::stoi(u0_str)-1;
                  int const u1 = std::stoi(u1_str)-1;
                  int const u2 = std::stoi(u2_str)-1;

                  add_triangle_index({u0,u1,u2});

                  skinning_info[u0] = temp_skinning;
                  skinning_info[u1] = temp_skinning;
                  skinning_info[u2] = temp_skinning;

                  color(u0)= color_bone;
                  color(u1)= color_bone;
                  color(u2)= color_bone;
                }
            }

        }

    }

    fid.close();

    //add the skinning weights in the mesh structure
    for(auto const& s : skinning_info)
        add_vertex_weight(s);//normalized(s)

    ASSERT_CPE(size_vertex_weight()==size_vertex(),"Mesh skinned seems to have the wrong number of skinning weights");

}


vec3 const& mesh_skinned::vertex_original(int index) const
{
    ASSERT_CPE(index>=0,"Index ("+std::to_string(index)+") must be positive");
    ASSERT_CPE(index<int(vertices_original_data.size()) , "Index ("+std::to_string(index)+") must be less than the current size of the vertex original vector ("+std::to_string(vertices_original_data.size())+")");

    return vertices_original_data[index];
}

void mesh_skinned::add_vertex(vec3 const& p)
{
    mesh::add_vertex(p);
    vertices_original_data.push_back(p);
}

void mesh_skinned::apply_skinning(skeleton_geometry const& skeleton)
{
    int const N_vertex = size_vertex();
    ASSERT_CPE(N_vertex==int(vertices_original_data.size()),"Incorrect size");

    for(int k_vertex=0 ; k_vertex<N_vertex ; ++k_vertex)
    {
        //TO DO: Calculer deformation par skinning
        //  vertex(k_vertex) = ...
        vertex_weight_parameter const& skinning_info = vertex_weight(k_vertex);
        int const N_joint = skinning_info.size();

        vec3 p0 = vertex_original(k_vertex);
        vec3 p_final(0.0f,0.0f,0.0f);
        for(int k_joint=0 ; k_joint<N_joint ; ++k_joint)
        {
            int const joint_id = skinning_info[k_joint].joint_id;
            float const w = skinning_info[k_joint].weight;

            skeleton_joint const& joint = skeleton[joint_id];


            vec3 const p = joint.orientation*p0 + joint.position;

            p_final += w*p;
        }
        vertex(k_vertex) = p_final;
    }

}

}
