# This code takes the following inputs:
#
# 1. A .skn (MuJoCo skin) file
# 2. An .obj file containing the old mesh,
#    and converted from an .stl file using Blender
# 3. An .obj file containing a new mesh, also using Blender
#
# In addition, it uses .mtl files associated with the .obj's (see readme).
#
# And outputs a new .skn file containing weights for the new mesh.

import numpy as np
import pywavefront
from scipy.spatial import cKDTree

from dm_control.utils import io as resources
from dm_control.mjcf import skin


def _read_skin(fp):
    """Reads MuJoCo's .skn file from a filepath fp."""
    class FakeBody:
        def __init__(self, full_identifier):
            self.full_identifier = full_identifier
    raw_skn = resources.GetResource(fp, mode='r+b')
    src_skn = skin.parse(raw_skn, body_getter=FakeBody)
    return src_skn
 

def _read_blender_obj(fp):
    """Reads .obj file from Blender and reverses Blender's coordinate transform.
    Returns: read obj and transformed vertices/faces as a numpy array."""
    obj = pywavefront.Wavefront(fp, collect_faces=True)
    transform = np.array(obj.vertices, dtype=np.float32)
    # reverse Blender's transformations
    transform[:, [1, 2]] = transform[:, [2, 1]]
    transform[:, 1] *= -1
    faces = np.array(obj.meshes['SKINbody'].faces, dtype=np.int32)
    return transform, faces
 

def _list_texcoords_and_faces(fp):
    """List texcoords and texturized faces from an .obj stored in path fp."""
    texcoords = []
    texfaces = []
    with open(fp, 'r') as f:
        for line in f:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'vt':
                texcoords.append(list(map(float, parts[1:])))
            if parts[0] == 'f':
                texfaces.append(list(map(str, parts[1:])))
    return texcoords, texfaces
 

def _calculate_target_weights(src_skn, src_vertices, k):
    """Calculates target weights for the vertices using kNN interpolation."""
    src_weight_matrix = np.zeros((len(src_skn.vertices), len(src_skn.bones)), 
                                 dtype=np.float32)
    for j, bone in enumerate(src_skn.bones):
        for vert_idx, weight in zip(bone.vertex_ids, bone.vertex_weights):
            src_weight_matrix[vert_idx, j] = weight
 
    tree = cKDTree(src_skn.vertices)
    distances, nearest_idxs = tree.query(src_vertices, k=1)
    if (k > 1):
        weights = 1.0 / distances
        weights /= np.sum(weights, axis=1, keepdims=True)

    target_weight_matrix = np.zeros((len(src_vertices), len(src_skn.bones)), 
                                    dtype=np.float32)
    for i in range(len(src_vertices)):
        if (k > 1):
            raise NotImplementedError
        else:
            target_weight_matrix[i, :] = src_weight_matrix[nearest_idxs[i], :]
    return target_weight_matrix
 

def _calculate_texcoords(s_texcoords, s_vertices, t_vertices):
    """Performs 1NN interpolation to calculate texcoordinates for t_vertices
    from s_texcoords via s_vertices."""
    tree = cKDTree(s_vertices)
    _, nearest_idxs = tree.query(t_vertices, k=1)
    t_texcoords = []
    for i, _ in enumerate(t_vertices):
        t_texcoords.append(s_texcoords[nearest_idxs[i]])
    return np.array(t_texcoords)
  
def reskin(s_skn_fp: str, t_obj_fp: str, t_skn_fp: str, k: int):

    s_skn = _read_skin(s_skn_fp)
    t_vertices, t_faces = _read_blender_obj(t_obj_fp)

    weights = _calculate_target_weights(s_skn, t_vertices, k=1)

    # convert source bones to target bones with new weights
    # (note weight info is held by bones in the .skn)
    t_bones = []
    for j, bone in enumerate(s_skn.bones):
        body = bone.body
        bindpos = bone.bindpos
        bindquat = bone.bindquat
        vertex_ids = np.arange(len(t_vertices))
        # use new weights for vertex
        vertex_weights = weights[:, j]
        target_bone = skin.Bone(body, bindpos, bindquat, vertex_ids, vertex_weights)
        t_bones.append(target_bone)
    
    s_texcoords = s_skn.texcoords
    s_vertices = s_skn.vertices
    t_texcoords = _calculate_texcoords(s_texcoords, s_vertices, t_vertices)

    t_skn = skin.Skin(t_vertices, t_texcoords, t_faces, t_bones)
    t_skn = skin.serialize(t_skn)
    with open(t_skn_fp, 'wb') as f:
        f.write(t_skn)
    

# --------------------------------------
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=
        "Converts an existing MuJoCo skin and a Blender .obj "
        "into a new MuJoCo skin. Requires an .mtl file"
        "associated with the .obj.")
    parser.add_argument('src_fp', type=str, help='Path to source MuJoCo .skn file')
    parser.add_argument('obj_fp', type=str, help='Path to Blender .obj file')
    parser.add_argument('target_fp', type=str, help='Location to save new .skn file')
    args = parser.parse_args()
    reskin(args.src_fp, args.obj_fp, args.target_fp, k=1)
