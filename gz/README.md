# gz: the Gazebo model and world of MARV
`make_model.py` reads the vehicle card (`marv-card/marv.toml`: mass, centre of mass, inertia, rotor positions) and the
CAD export (`model/marv_quad/{model.sdf,meshes/}` in the CAD repo: meshes, link offsets, collision shapes). It writes
`gz/marv/{model.sdf,model.config,model.json}` and `gz/world.sdf`. Meshes are referenced by relative path, not copied.

Regenerate (byte-identical output for the same inputs):

    python3 gz/make_model.py --card marv-card/marv.toml --cad /home/luis/Projects/CAD/drone

View:

    GZ_SIM_RESOURCE_PATH=$PWD/gz gz sim -r --force-version 8.15.0 gz/world.sdf

The world has ZERO gravity on purpose: the sim's plant applies the weight at the centre of mass. The vehicle starts
at z = h_rest (in `model.json`) with its stance on the ground plane. Friction 1 and restitution 0 are placeholders.
