# Attribution and component licenses

This repository is a portfolio publication of an ongoing Texas A&M team capstone. It does not claim sole authorship of inherited rover code or third-party assets.

## Source provenance

- Integrated source: [agfoti04/AMD-Capstone](https://github.com/agfoti04/AMD-Capstone), base commit `14d616ea1f27f6442223cab8d2b20446e41da3ec`, plus the `navigation/mapping-and-exploration` worktree through October 25, 2026.
- Destination history: the original Ian Benge webcam starter is retained under `examples/webcam`; its initial Git commit is preserved.
- Existing file copyright headers, package maintainers and authorship declarations are retained. `evidence/publication-source.json` records imported file hashes and provenance.

## Licensing declarations

`rover_navigation`, `rover_navigation_interfaces`, `rover_perception`, `rover_training` and `mars_training_environment` declare **Apache-2.0** in their ROS package metadata. Those declarations remain in place. The inherited `four_wheel_sim` package still contains its original unresolved `TODO: License declaration`; this publication does not assign it a new license or assert a blanket repository license. Existing third-party file notices continue to apply.

## Assets and dependencies

- Bundled Falling Rock meshes/textures: Open Robotics Gazebo Fuel, **CC BY 4.0**; authors and details are preserved in [the asset attribution](src/rover_training/models/ATTRIBUTION.md).
- External scanned rocks and ground textures: Poly Haven, **CC0**, referenced in pinned asset manifests and fetched separately. The asset cache is not distributed here.
- NASA imagery is referenced for visual design context; no claim of NASA-created training meshes or NASA endorsement is made.
- ROS 2, Gazebo, RTAB-Map, Nav2, PyTorch/ROCm, OpenCV, AMD runtime components and Ultralytics retain their respective licenses. Their inclusion as dependencies does not relicense them. Model weights, AMD SDK binaries and compiled model caches are not distributed.

Consult the relevant upstream licenses before redistributing dependencies or model artifacts.
