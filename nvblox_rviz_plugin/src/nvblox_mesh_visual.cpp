/**
 * Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */
 
#include <iostream>
#include <limits>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "nvblox_rviz_plugin/nvblox_mesh_visual.h"

namespace nvblox_rviz_plugin {

unsigned int NvbloxMeshVisual::instance_counter_ = 0;

NvbloxMeshVisual::NvbloxMeshVisual(Ogre::SceneManager* scene_manager,
                                   Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  // Maybe not necessary anymore?
  frame_node_ = parent_node->createChildSceneNode();
  instance_number_ = instance_counter_++;
}

NvbloxMeshVisual::~NvbloxMeshVisual() {
  // Destroy all the objects
  for (const auto ogre_object : object_map_) {
    scene_manager_->destroyManualObject(ogre_object.second);
  }
}

void NvbloxMeshVisual::setCeilingCutoff(bool cut_ceiling,
                                        float ceiling_height) {
  cut_ceiling_ = cut_ceiling;
  ceiling_height_ = ceiling_height;

  // Iterate over all the ogre objects, setting blocks visible and not again.
  for (const auto kv : object_map_) {
    if (cut_ceiling_) {
      if (kv.first.z * block_size_ <= ceiling_height_) {
        kv.second->setVisible(true);
      } else {
        kv.second->setVisible(false);
      }
    } else {
      kv.second->setVisible(true);
    }
  }
}

void NvbloxMeshVisual::setMessage(
    const nvblox_msgs::msg::Mesh::ConstSharedPtr& msg) {
  block_size_ = msg->block_size;

  // First, check if we need to clear the existing map.
  if (msg->clear) {
    for (const auto ogre_object : object_map_) {
      scene_manager_->destroyManualObject(ogre_object.second);
    }
    object_map_.clear();
  }

  // Iterate over all the blocks in the message and make sure to add them.
  for (size_t i = 0; i < msg->block_indices.size(); i++) {
    const nvblox_msgs::msg::Index3D& block_index = msg->block_indices[i];
    const nvblox_msgs::msg::MeshBlock& mesh_block = msg->blocks[i];

    // create ogre object
    Ogre::ManualObject* ogre_object;
    bool new_object = true;
    const auto it = object_map_.find(block_index);
    if (it != object_map_.end()) {
      // delete empty mesh blocks
      if (mesh_block.vertices.empty()) {
        scene_manager_->destroyManualObject(it->second);
        object_map_.erase(it);
        continue;
      }

      ogre_object = it->second;
      new_object = false;
    } else {
      if (mesh_block.vertices.empty()) {
        continue;
      }
      std::string object_name =
          std::to_string(block_index.x) + std::string(" ") +
          std::to_string(block_index.y) + std::string(" ") +
          std::to_string(block_index.z) + std::string(" ") +
          std::to_string(instance_number_);
      ogre_object = scene_manager_->createManualObject(object_name);
      object_map_.insert(std::make_pair(block_index, ogre_object));

      frame_node_->attachObject(ogre_object);
    }

    ogre_object->estimateVertexCount(mesh_block.vertices.size());
    ogre_object->estimateIndexCount(mesh_block.triangles.size());
    if (new_object) {
      ogre_object->begin("BaseWhiteNoLighting",
                         Ogre::RenderOperation::OT_TRIANGLE_LIST);
    } else {
      ogre_object->beginUpdate(0);
    }

    for (size_t i = 0; i < mesh_block.vertices.size(); ++i) {
      // note calling position changes what vertex the color and normal calls
      // point to
      ogre_object->position(mesh_block.vertices[i].x, mesh_block.vertices[i].y,
                            mesh_block.vertices[i].z);

      ogre_object->normal(mesh_block.normals[i].x, mesh_block.normals[i].y,
                          mesh_block.normals[i].z);

      if (mesh_block.colors.empty()) {
        ogre_object->colour(mesh_block.normals[i].x * 0.5f + 0.5f,
                            mesh_block.normals[i].y * 0.5f + 0.5f,
                            mesh_block.normals[i].z * 0.5f + 0.5f);
      } else {
        ogre_object->colour(mesh_block.colors[i].r, mesh_block.colors[i].g,
                            mesh_block.colors[i].b);
      }
    }

    // needed for anything other than flat rendering
    for (int32_t index : mesh_block.triangles) {
      ogre_object->index(index);
    }

    ogre_object->end();

    // Cut the ceiling immediatly if we're doing that.
    if (cut_ceiling_ && block_index.z * block_size_ > ceiling_height_) {
      ogre_object->setVisible(false);
    }
  }
}

void NvbloxMeshVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void NvbloxMeshVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

}  // namespace nvblox_rviz_plugin
