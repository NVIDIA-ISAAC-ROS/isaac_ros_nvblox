// A message converter to convert nvblox ROS messages to foxglove.

import {
  Color,
  CubePrimitive,
  Pose,
  SceneEntity,
  SceneEntityDeletion,
  SceneEntityDeletionType,
  SceneUpdate,
} from "@foxglove/schemas";
import { Duration } from "@foxglove/schemas/schemas/typescript/Duration";
import { Time } from "@foxglove/schemas/schemas/typescript/Time";
import { ExtensionContext } from "@foxglove/studio";

// Redefine ROS message types.
type Header = { stamp: Time; frame_id: string };
type Index3D = { x: number; y: number; z: number };
type Point3D = { x: number; y: number; z: number };

type MeshBlock = {
  vertices: Point3D[];
  normals: Point3D[];
  colors: Color[];
  triangles: number[];
};

type Mesh = {
  header: Header;
  block_size_m: number;
  block_indices: Index3D[];
  blocks: MeshBlock[];
  clear: boolean;
};

type VoxelBlock = {
  centers: Point3D[];
  colors: Color[];
};

type VoxelBlockLayer = {
  header: Header;
  block_size_m: number;
  block_indices: Index3D[];
  blocks: VoxelBlock[];
  clear: boolean;
  layer_type: number;
};

const BLOCK_LIFETIME: Duration = { sec: 10, nsec: 0 };

export function activate(extensionContext: ExtensionContext): void {
  console.log("Activating nvbloxFoxglove extension.");

  extensionContext.registerMessageConverter({
    fromSchemaName: "nvblox_msgs/msg/Mesh",
    toSchemaName: "foxglove.SceneUpdate",
    converter: foxgloveFromRosMesh,
  });

  extensionContext.registerMessageConverter({
    fromSchemaName: "nvblox_msgs/msg/VoxelBlockLayer",
    toSchemaName: "foxglove.SceneUpdate",
    converter: foxgloveFromRosVoxelBlockLayer,
  });
}

function foxgloveFromRosVoxelBlockLayer(layer: VoxelBlockLayer): SceneUpdate {
  const entities: SceneEntity[] = [];
  const deletions: SceneEntityDeletion[] = [];

  const voxel_size = layer.block_size_m / 8;

  for (let i_block = 0; i_block < layer.blocks.length; ++i_block) {
    const block = layer.blocks[i_block];
    const idx = layer.block_indices[i_block];
    if (!idx || !block) {
      console.log("Received invalid block or block index.");
      continue;
    }
    const id_string = `${layer.layer_type}_${idx.x}_${idx.y}_${idx.z}`;
    if (block.centers.length == 0) {
      const deletion: SceneEntityDeletion = {
        timestamp: layer.header.stamp,
        type: SceneEntityDeletionType.MATCHING_ID,
        id: id_string,
      };
      deletions.push(deletion);
    } else {
      const voxel_cubes: CubePrimitive[] = [];
      for (let i_voxel = 0; i_voxel < block.centers.length; ++i_voxel) {
        const center = block.centers[i_voxel];
        const color = block.colors[i_voxel];

        if (center && color) {
          voxel_cubes.push({
            pose: {
              position: { x: center.x, y: center.y, z: center.z },
              orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
            size: { x: voxel_size, y: voxel_size, z: voxel_size },
            color: { r: color.r, g: color.g, b: color.b, a: 1 },
          });
        }
      }
      const entity: SceneEntity = {
        timestamp: layer.header.stamp,
        frame_id: layer.header.frame_id,
        id: id_string,
        lifetime: BLOCK_LIFETIME,
        frame_locked: true,
        metadata: [],
        arrows: [],
        triangles: [],
        cylinders: [],
        lines: [],
        cubes: voxel_cubes,
        texts: [],
        models: [],
        spheres: [],
      };
      entities.push(entity);
    }
  }

  return {
    deletions,
    entities,
  };
}

function foxgloveFromRosMesh(mesh: Mesh): SceneUpdate {
  const default_pose: Pose = {
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 },
  };
  const default_color: Color = { r: 1, g: 0, b: 0, a: 1 };

  const entities: SceneEntity[] = [];
  const deletions: SceneEntityDeletion[] = [];

  for (let i = 0; i < mesh.blocks.length; ++i) {
    const block = mesh.blocks[i];
    const idx = mesh.block_indices[i];
    if (!idx || !block) {
      console.log("Received invalid block or block index.");
      continue;
    }
    const id_string = `${idx.x}_${idx.y}_${idx.z}`;
    if (block.vertices.length == 0) {
      const deletion: SceneEntityDeletion = {
        timestamp: mesh.header.stamp,
        type: SceneEntityDeletionType.MATCHING_ID,
        id: id_string,
      };
      deletions.push(deletion);
    } else {
      const entity: SceneEntity = {
        timestamp: mesh.header.stamp,
        frame_id: mesh.header.frame_id,
        id: id_string,
        lifetime: BLOCK_LIFETIME,
        frame_locked: true,
        metadata: [],
        arrows: [],
        cubes: [],
        cylinders: [],
        lines: [],
        triangles: [
          {
            pose: default_pose,
            points: block.vertices,
            colors: block.colors,
            color: default_color,
            indices: block.triangles,
          },
        ],
        texts: [],
        models: [],
        spheres: [],
      };
      entities.push(entity);
    }
  }

  return {
    deletions,
    entities,
  };
}
