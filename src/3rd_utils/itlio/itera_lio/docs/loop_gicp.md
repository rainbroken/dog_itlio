# Robust loop GICP

LoopVerifier uses the in-tree manual_gicp implementation for Cauchy robust
GICP in all five initialization hypotheses (one for local refinement).
This registration runs in loop verification, separately from the ESIKF
frontend and pose-graph optimizer. No external registration engine is built
or linked by IteraLIO.

The C++ and ROS defaults select manual_gicp. The registration_engine field
is retained for ROS configuration and archived loop-pair compatibility;
manual_gicp is its only accepted value. loop_pair_replay can replay older
archives by explicitly selecting manual_gicp on its command line.

Parameters under loop_closure.verifier:

- max_correspondence_distance: optimization search radius (m), 1.5.
- inlier_distance: forward/reverse evaluation radius (m), 0.45.
- gicp_voxel_size: covariance preprocessing resolution (m), 0.30.
- robust_kernel_scale: Cauchy scale in the covariance-weighted GICP objective, 1.0.
- num_threads: preprocessing/reduction OpenMP threads, 2.
- max_iterations: optimizer limit, 40.
- max_fitness: mean squared evaluation residual (m^2), not the GICP objective
  or RMSE; the C++ default is 0.05 and loop.yaml selects 0.03.

Preprocessing is shared among hypotheses within each coordinate frame. Existing
source/target point caps, bidirectional overlap, geometry, height, gravity and
multi-keyframe confirmation gates remain active. Restricted world x/y/yaw
hypotheses use manual_gicp's xy_yaw_only mode with exact world x/y/yaw updates.
No-inlier hypotheses must not outrank supported alignments merely because
their correction is small.
The failure reason is now gicp_not_converged. Legacy CSV column names containing
icp are retained for downstream reader compatibility.

The existing backend and optimized-map export are reused. cloud_color.pcd is
not retroactively corrected by this change; successful loop constraints and
optimization are prerequisites for cloud_color_optimized.pcd.

Loop weights now retain the full geometry-derived 6x6 information in both
online verification and local refinement. Correlated submap support is averaged,
not summed as independent measurements. The information is transported from
left-rotation/additive-translation measurement coordinates to the residual of
the inverse graph measurement; no extra fixed yaw multiplier is applied.
Local refinement rejects a distant competing solution when its overlap is no
smaller and its clipped loss over all source points is no larger. It never uses
that probe to accept a global relocation as a local correction.

Validation: build with
`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`,
then run the package C++ regression tests. Replay the same bag and compare accepted/committed loops,
wrong-loop rejection, worker queue/timeouts, optimized trajectory and revisit
ground geometry before claiming that map ghosting is fixed.
