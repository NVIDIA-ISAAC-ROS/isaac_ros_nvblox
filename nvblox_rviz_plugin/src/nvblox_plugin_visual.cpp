// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <limits>


#include <OgreSceneManager.h>
#include <OgreRoot.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreMaterialManager.h>
#include <OgreResourceGroupManager.h>
#include <OgreRenderSystem.h>
#include <OgreRenderSystemCapabilities.h>

#include "nvblox_rviz_plugin/nvblox_plugin_visual.h"
#include <nvblox_msgs/msg/voxel_block_layer.hpp>

#include "rviz_rendering/render_system.hpp"
#include "rviz_rendering/resource_config.hpp"

namespace nvblox_rviz_plugin
{


NvbloxVisualParams::NvbloxVisualParams(rviz_common::_RosTopicDisplay * parent)
{
  cut_ceiling_property_ = new rviz_common::properties::BoolProperty(
    "Cut Ceiling", false, "If set to true, will not visualize anything above a certain z value.",
    nullptr, SLOT(updateCeilingOptions()), this);


  ceiling_height_property_ = new rviz_common::properties::FloatProperty(
    "Ceiling Height", 1.5, "Height above which the visualization will be cut off.", nullptr,
    SLOT(updateCeilingOptions()), this);

  parent->addChild(cut_ceiling_property_);
  parent->addChild(ceiling_height_property_);
}

void NvbloxVisualParams::updateCeilingOptions()
{
  std::cout << "Ceiling options changed: height: " << getCeilingHeight() << " cutCeiling: " <<
    getCutCeiling() << std::endl;
  params_changed_ = true;
}

bool NvbloxVisualParams::paramsChanged()
{
  if (params_changed_) {
    params_changed_ = false;
    return true;
  }
  return false;
}

float NvbloxVisualParams::getCeilingHeight()
{
  return ceiling_height_property_->getFloat();
}

bool NvbloxVisualParams::getCutCeiling()
{
  return cut_ceiling_property_->getBool();
}


template<typename MessageType>
unsigned int
NvbloxBaseVisual<MessageType>::instance_counter_ = 0;

// Custom material group for the glsl150 materials

template<typename MessageType>
void NvbloxBaseVisual<MessageType>::initializeBoxMaterial()
{
  // For some reason reason, only rviz's glsl120 kernels are enabled per default, so we need to
  // enable the glsl150 ones explicilty as we need them to properly render the pointcloud box
  constexpr const char * kBoxMaterialName = "nvblox/box";
  constexpr const char * kBoxMaterialGroup = "rviz_rendering_150";
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_rendering::get_resource_directory() + "/ogre_media/materials/glsl150", "FileSystem",
    kBoxMaterialGroup);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
    rviz_rendering::get_resource_directory() + "/ogre_media/materials/scripts150", "FileSystem",
    kBoxMaterialGroup);

  Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(kBoxMaterialGroup);
  if (!Ogre::ResourceGroupManager::getSingleton().isResourceGroupInitialised(kBoxMaterialGroup)) {
    std::cerr << "ERROR: could not initialize resource group" << std::endl;
    std::abort();
  }

  Ogre::ResourceGroupManager::getSingleton().loadResourceGroup(kBoxMaterialGroup);
  if (!Ogre::ResourceGroupManager::getSingleton().isResourceGroupLoaded(kBoxMaterialGroup)) {
    std::cerr << "ERROR: could not load resource group" << std::endl;
    std::abort();
  }

  // Clone the existing PointCloudBox to avoid affecting other users of the same material
  voxel_material_ = Ogre::MaterialManager::getSingleton().getByName(kBoxMaterialName);
  if (voxel_material_ == nullptr) {
    auto material = Ogre::MaterialManager::getSingleton().getByName("rviz/PointCloudBox");
    material->load();
    voxel_material_ = material->clone(kBoxMaterialName);
  }

  // The default geometry shader uses directional lighting which creates large contrasts between the
  // sides of the rendered cubes. We therefore select another shader that doesn't use lighting.
  voxel_material_->getBestTechnique()->getPass(0)->setGeometryProgram("rviz/glsl150/box.geom");
}
template<typename MessageType>
void NvbloxBaseVisual<MessageType>::initializeFallbackMaterial()
{
  voxel_material_ = Ogre::MaterialManager::getSingleton().getByName("BaseWhiteNoLighting");
  using_fallback_voxel_material_ = true;
}

template<typename MessageType>
NvbloxBaseVisual<MessageType>::NvbloxBaseVisual(
  rviz_common::_RosTopicDisplay * parent,
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_node)
{
  scene_manager_ = scene_manager;
  // Maybe not necessary anymore?
  frame_node_ = parent_node->createChildSceneNode();
  instance_number_ = instance_counter_++;

  params_ = std::make_unique<NvbloxVisualParams>(parent);

  if (rviz_rendering::RenderSystem::get()->getGlslVersion() < 150) {
    std::cerr << "ERROR: Nvblox rviz plugin require support for glsl 150 or higher" << std::endl;
    std::abort();
  }

  // Setup material used for renering voxels (mesh always uses the same material). If the GPU
  // supports geoemtry shader, we render boxes, otherwise we fall back to point rendering.
  const bool can_process_geometry_shader =
    Ogre::Root::getSingleton().getRenderSystem()->getCapabilities()->hasCapability(
    Ogre::RSC_GEOMETRY_PROGRAM);
  const bool use_fallback_matrial = !can_process_geometry_shader || getenv(
    "ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL") != nullptr;

  if (use_fallback_matrial) {
    std::cout <<
      "Nvblox plugin did not detect a geometry shader capable GPU. Using fallback rendering."
              << std::endl;
    initializeFallbackMaterial();
  } else {
    initializeBoxMaterial();
  }
}

template<typename MessageType>
NvbloxBaseVisual<MessageType>::~NvbloxBaseVisual()
{
  // Destroy all the objects
  for (const auto ogre_object : object_map_) {
    scene_manager_->destroyManualObject(ogre_object.second);
  }
}
template<typename MessageType>
void NvbloxBaseVisual<MessageType>::maybeCutCeiling(
  const bool cut_ceiling,
  const float ceiling_height)
{
  // Iterate over all the ogre objects, setting blocks visible and not again.
  for (const auto kv : object_map_) {
    if (cut_ceiling) {
      if (kv.first.z * block_size_m_ <= ceiling_height) {
        kv.second->setVisible(true);
      } else {
        kv.second->setVisible(false);
      }
    } else {
      kv.second->setVisible(true);
    }
  }
}

// Specialization for mesh message
template<> void
NvbloxBaseVisual<nvblox_msgs::msg::Mesh>::setMessage(
  const nvblox_msgs::msg::Mesh::ConstSharedPtr & msg)
{
  block_size_m_ = msg->block_size_m;

  if (params_->paramsChanged()) {
    maybeCutCeiling(params_->getCutCeiling(), params_->getCeilingHeight());
  }

  // First, check if we need to clear the existing map.
  if (msg->clear) {
    for (const auto ogre_object : object_map_) {
      scene_manager_->destroyManualObject(ogre_object.second);
    }
    object_map_.clear();
  }

  // Iterate over all the blocks in the message and make sure to add them.
  for (size_t i = 0; i < msg->block_indices.size(); i++) {
    const nvblox_msgs::msg::Index3D & block_index = msg->block_indices[i];
    const nvblox_msgs::msg::MeshBlock & mesh_block = msg->blocks[i];

    // create ogre object
    Ogre::ManualObject * ogre_object;
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
      std::string object_name = std::to_string(block_index.x) + std::string(" ") +
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
      ogre_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    } else {
      ogre_object->beginUpdate(0);
    }

    for (size_t i = 0; i < mesh_block.vertices.size(); ++i) {
      // note calling position changes what vertex the color and normal calls
      // point to
      ogre_object->position(
        mesh_block.vertices[i].x, mesh_block.vertices[i].y,
        mesh_block.vertices[i].z);

      std_msgs::msg::ColorRGBA color;
      if (!mesh_block.colors.empty()) {
        color = mesh_block.colors[i];
      }
      ogre_object->colour(color.r, color.g, color.b);
    }

    // needed for anything other than flat rendering
    for (int32_t index : mesh_block.triangles) {
      ogre_object->index(index);
    }

    // Cut the ceiling immediatly if we're doing that.
    if (params_->getCutCeiling() && block_index.z * block_size_m_ > params_->getCeilingHeight()) {
      ogre_object->setVisible(false);
    }

    ogre_object->end();
  }
}

void addVertex(
  const float x, const float y, const float z,
  const std_msgs::msg::ColorRGBA & color, Ogre::ManualObject * obj)
{
  obj->position(x, y, z);
  obj->colour(color.r, color.g, color.b);
}

// Specialization for voxelblocks
template<> void
NvbloxBaseVisual<nvblox_msgs::msg::VoxelBlockLayer>::setMessage(
  const nvblox_msgs::msg::VoxelBlockLayer::ConstSharedPtr & msg)
{
  block_size_m_ = msg->block_size_m;

  if (params_->paramsChanged()) {
    maybeCutCeiling(params_->getCutCeiling(), params_->getCeilingHeight());
  }

  // Set size. How this is done depends on which material we are rendering
  if (using_fallback_voxel_material_) {
    // Set size for a point
    voxel_material_->getBestTechnique()->getPass(0)->setPointSize(5.0f);
  } else {
    // Set the size of a box by passing param to the geometry shader
    Ogre::GpuProgramParametersSharedPtr params =
      voxel_material_->getBestTechnique()->getPass(0)->getGeometryProgramParameters();
    params->setNamedConstant(
      "size",
      Ogre::Vector4(msg->voxel_size_m, msg->voxel_size_m, msg->voxel_size_m, 1));
  }

  // First, check if we need to clear the existing map.
  if (msg->clear) {
    for (const auto ogre_object : object_map_) {
      scene_manager_->destroyManualObject(ogre_object.second);
    }
    object_map_.clear();
  }

  // Iterate over all the blocks in the message and make sure to add them.
  for (size_t i = 0; i < msg->block_indices.size(); i++) {
    const nvblox_msgs::msg::Index3D & block_index = msg->block_indices[i];
    const nvblox_msgs::msg::VoxelBlock & voxel_block = msg->blocks[i];

    // create ogre object
    Ogre::ManualObject * ogre_object;
    bool new_object = true;
    const auto it = object_map_.find(block_index);

    if (it != object_map_.end()) {
      // The block already exists

      // delete empty blocks
      if (voxel_block.centers.empty()) {
        scene_manager_->destroyManualObject(it->second);
        object_map_.erase(it);
        continue;
      }

      ogre_object = it->second;
      new_object = false;
    } else {
      //  The block doesn't exist
      if (voxel_block.centers.empty()) {
        continue;
      }
      std::string object_name = std::to_string(msg->layer_type) + ":" +
        std::to_string(block_index.x) + std::string(" ") +
        std::to_string(block_index.y) + std::string(" ") +
        std::to_string(block_index.z) + std::string(" ") +
        std::to_string(instance_number_);
      ogre_object = scene_manager_->createManualObject(object_name);
      object_map_.insert(std::make_pair(block_index, ogre_object));

      frame_node_->attachObject(ogre_object);
    }
    ogre_object->setDynamic(true);
    ogre_object->estimateVertexCount(voxel_block.centers.size());

    // Create points for all voxels
    if (new_object) {
      ogre_object->begin(
        voxel_material_->getName(), Ogre::RenderOperation::OT_POINT_LIST,
        voxel_material_->getGroup());
    } else {
      ogre_object->beginUpdate(0);
    }

    for (size_t i = 0; i < voxel_block.centers.size(); ++i) {
      std_msgs::msg::ColorRGBA color;
      if (!voxel_block.colors.empty()) {
        color = voxel_block.colors[i];
      }

      addVertex(
        voxel_block.centers[i].x, voxel_block.centers[i].y, voxel_block.centers[i].z, color,
        ogre_object);
    }
    ogre_object->end();

    // Cut the ceiling immediatly if we're doing that.
    if (params_->getCutCeiling() && block_index.z * block_size_m_ > params_->getCeilingHeight()) {
      ogre_object->setVisible(false);
    }
  }  // end block_indices loop
}


template<typename MessageType>
void
NvbloxBaseVisual<MessageType>::setFramePosition(const Ogre::Vector3 & position)
{
  frame_node_->setPosition(position);
}

template<typename MessageType>
void
NvbloxBaseVisual<MessageType>::setFrameOrientation(const Ogre::Quaternion & orientation)
{
  frame_node_->setOrientation(orientation);
}

template class NvbloxBaseVisual<nvblox_msgs::msg::Mesh>;
template class NvbloxBaseVisual<nvblox_msgs::msg::VoxelBlockLayer>;

}  // namespace nvblox_rviz_plugin
