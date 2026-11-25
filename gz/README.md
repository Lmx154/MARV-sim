# gz

Part of [MARV-sim](../README.md)

The Gazebo model and world of the MARV quadcopter, generated from the vehicle card and the CAD export.

| Path | Contents |
|---|---|
| `make_model.py` | The generator: reads `marv-card/marv.toml` and the CAD export, writes the files below |
| `marv/model.sdf`, `model.config`, `model.json` | The model; `model.json` has the CAD commit, card sha256 and `h_rest` |
| `world.sdf` | The world, with zero gravity (the sim applies the weight itself) |

```
python3 gz/make_model.py --card marv-card/marv.toml --cad <path to the CAD drone folder>
GZ_SIM_RESOURCE_PATH=$PWD/gz gz sim -r --force-version 8.15.0 gz/world.sdf
```

Details: [gazebo](../docs/gazebo.md).
