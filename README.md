# mujoco_misc
Various small utilities I've created while working with MuJoCo.

### reskin.py

Converts an existing MuJoCo skin `A.skn` and a Blender `B.obj` into a new MuJoCo skin using `B`'s vertices, interpolated on `A`'s vertices using nearest-neighbor interpolation.

#### Usage

One thing that you might want to do with skins is to create them from a Blender mesh, saved as an `.obj`. But that is tough, since there's no GUI for weighting vertices and it's obviously a cumbersome task to do on your own. One solution, if you already have a MuJoCo skin, is to interpolate to it. That's what my script is doing. 

If you have an `A.skn` and a `B.obj`, you should first make sure that the vertices are roughly in the same position, so that `B` mesh can be meaningfully projected onto `A` by nearest-vertex interpolation. If `A.skn` was generated from an `.stl`/`.msh`/`.obj` file, you can do this by just loading that file into Blender and aligning your `B` mesh with it, and saving the result.

Download `reskin.py`, make sure that `B.mtl` (the `.mtl` associated with `B.obj`) is in the same directory as `B.obj`, and run script in Python like this:
```
>>> python reskin.py path1/to/A.skn path2/to/B.obj path3/to/B.skn 
```
It will save the reskinned mesh `B.skn` into `path3/to`.


#### Explanation

Starting from 2.0, MuJoCo has [a new type of rendering mesh](https://mujoco.readthedocs.io/en/latest/overview.html#skin) called *skin* and associated with the `.skn` extension. It is used for example in dm_control's Dog model. Calling them meshes can be slightly misleading; they don't participate in the physics, and are exclusively used for rendering. They follow a custom binary format described [here](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-skin). 

In mathy terms, a skin consists of a mesh (collection of vertices $w = (x, y, z) \in {\bf R}^3$ and triangle faces $t = (w, w', w'')$ ), texcoords $w \mapsto (u_w, v_w) \in [0, 1]^2$ and bones $b$, which provide the connection to MuJoCo's physics; each bone is associated with a MuJoCo body, and holds a list of vertices $W(b)$ whose position it affects, and associated weights $a_{b, w}$ for $w \in W(b)$. So if each bone $b$ moves by $\Delta b$, then a vertex $w$ may be expected to move by

$$
\sum_{w \in W(b)} a_{b, w} \cdot \Delta b. 
$$

The texcoords are of course for wrapping a bitmap on a mesh. What my script does is taking a new set of vertices, $z$, and first calculating the nearest neighbor $w(z)$, and then setting $a_{b, z} = a_{b, w(z)}$ for all $z$, and $(u_z, v_z) = (u_{w(z)}, u_{v(z)})$. 
