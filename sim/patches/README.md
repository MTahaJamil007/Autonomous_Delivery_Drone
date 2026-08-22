# Historical PX4 submodule patches

These record the state the simulation was in *before* remediation plan P0.2
vendored the assets into `sim/`. They are kept for reproducibility and
forensics, not for use.

## `0001-px4-gz-submodule-sensor-edits.patch`

The uncommitted working-tree diff of `PX4-Autopilot/Tools/simulation/gz` as
found on 2026-08-23:

- `models/x500_base/model.sdf` — the downward camera and `rplidar_a1` added
  directly into the stock airframe's `base_link`, both with **absolute** topic
  names `/camera/image` and `/lidar/scan`.
- `worlds/default.sdf` — world origin moved to Chichawatni, plus `great_wall`
  and a `landing_pad` include of the undecodable `model://arucotag`.

Not captured by the patch (they were untracked, so `git diff` does not see
them): `models/arucotag_0/`, `models/arucotag_1/`, `models/arucotag_2/`, and
`worlds/default_backup.txt`. The three `arucotag_N` textures are the useful
part and have been copied into `sim/models/pad_0|1|2/`.

## Do not re-apply these

The vendored equivalents in `sim/` supersede them and fix three defects the
originals carry:

| Original defect | Vendored fix |
| --- | --- |
| Sensors live inside a third-party git submodule, so a `submodule update --force` deletes them (F2) | `sim/models/x500_delivery/` under this project's version control |
| Absolute `/camera/image` and `/lidar/scan`, so an N-drone fleet shares two topics (F7) | No `<topic>` element; Gazebo derives a model-scoped name per instance |
| `landing_pad` uses `model://arucotag`, which has no quiet zone and decodes in zero of 27 ArUco dictionaries (F1) | Pads spawned at mission time from the decodable `pad_0/1/2` models |

## Reverting the PX4 submodule to stock

Once you have confirmed the vendored assets fly, the submodule can and should
go back to stock — that is the whole point of vendoring:

```bash
cd ~/PX4-Autopilot/Tools/simulation/gz
git stash            # or: git checkout -- models/x500_base/model.sdf worlds/default.sdf
```

`scripts/preflight.py` will still pass, because it checks the topics our own
model advertises. If it starts failing after this, the vendoring is incomplete
and that is exactly the signal you want.
