// A message converter to convert nvblox ROS messages to foxglove.

import { SceneUpdate, SceneEntity, SceneEntityDeletion, SceneEntityDeletionType, Pose, Color } from "@foxglove/schemas";
import { Duration } from "@foxglove/schemas/schemas/typescript/Duration";
import { Time } from "@foxglove/schemas/schemas/typescript/Time";
import { ExtensionContext } from "@foxglove/studio";

// Redefine ROS message types.
type Header =  {stamp: Time, frame_id: string}
type Index3D = {x: number, y: number, z: number}
type Point3D = {x: number, y: number, z: number}

type MeshBlock = {
  vertices: Point3D[],
  normals: Point3D[],
  colors: Color[],
  triangles: number[]
}

type Mesh = {
  header: Header,
  block_size: number,
  block_indices: Index3D[],
  blocks: MeshBlock[],
  clear: boolean
}

const MESH_BLOCK_LIFETIME: Duration = {sec: 10, nsec: 0};

export function activate(extensionContext: ExtensionContext): void {
  console.log("Activating nvbloxFoxglove extension.");

  extensionContext.registerMessageConverter({
    fromSchemaName: "nvblox_msgs/msg/Mesh",
    toSchemaName: "foxglove.SceneUpdate",
    converter: foxgloveFromRos,
  });
}

function foxgloveFromRos(mesh: Mesh): SceneUpdate {
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
    if( block.vertices.length == 0){
      const deletion: SceneEntityDeletion = {
        timestamp: mesh.header.stamp,
        type: SceneEntityDeletionType.MATCHING_ID,
        id: id_string
      };
      deletions.push(deletion);
    }else{
      const entity: SceneEntity = {
        timestamp: mesh.header.stamp,
        frame_id: mesh.header.frame_id,
        id: id_string,
        lifetime: MESH_BLOCK_LIFETIME,
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

