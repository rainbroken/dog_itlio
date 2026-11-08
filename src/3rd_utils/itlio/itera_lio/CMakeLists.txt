cmake_minimum_required(VERSION 3.8)
project(itera_lio)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
option(ITERA_LIO_NATIVE_LOOP_SEARCH "Use own bounded exact KD-tree for loop correspondences" ON)
option(ITERA_LIO_PARALLEL_LOOP_SEARCH "Parallelize independent loop correspondence queries" ON)
option(ITERA_LIO_CACHED_LOOP_BOUNDS "Reuse exact query bounds and contiguous leaf storage" ON)
option(ITERA_LIO_NATIVE_LOOP_KNN "Use own exact KNN for loop plane covariances" ON)
option(ITERA_LIO_DIRECT_LOOP_INVERSE "Use fixed 3x3 inverse with SPD checks and LDLT fallback" ON)
option(ITERA_LIO_DIRECT_LOOP_EIGEN "Use analytic symmetric 3x3 eigensolver for loop normals" ON)
option(ITERA_LIO_FLOAT_LOOP_BOUNDS "Use conservative float AABB bounds" ON)
option(ITERA_LIO_SIMD_LOOP_LEAVES "Use SIMD structure-of-arrays leaf distances" ON)
option(ITERA_LIO_SIMD_LOOP_KNN "Use SIMD distances in exact covariance KNN" ON)
option(ITERA_LIO_AVX_LOOP_LEAVES "Use runtime-dispatched AVX2 with SSE/scalar fallback" ON)
option(ITERA_LIO_CACHE_LOOP_SOURCE "Cache bit-identical source preprocessing for loop registration" ON)
option(ITERA_LIO_GATED_INTERNAL_PROFILE "Enable detailed internal profiling only when explicitly requested" ON)
add_compile_definitions(ITERA_LIO_GATED_DIAGNOSTICS=$<BOOL:${ITERA_LIO_GATED_INTERNAL_PROFILE}>)
set(ITERA_LIO_LOOP_LEAF_SIZE 128 CACHE STRING "Maximum points per exact KD-tree leaf")
if(NOT ITERA_LIO_LOOP_LEAF_SIZE MATCHES "^[1-9][0-9]*$")
  message(FATAL_ERROR "ITERA_LIO_LOOP_LEAF_SIZE must be a positive integer")
endif()
set(ITERA_LOOP_CORE_DEFINITIONS
  ITERA_MANUAL_NATIVE_SEARCH=$<BOOL:${ITERA_LIO_NATIVE_LOOP_SEARCH}>
  ITERA_MANUAL_PARALLEL_SEARCH=$<BOOL:${ITERA_LIO_PARALLEL_LOOP_SEARCH}>
  ITERA_MANUAL_CACHED_BOUNDS=$<BOOL:${ITERA_LIO_CACHED_LOOP_BOUNDS}>
  ITERA_MANUAL_NATIVE_KNN=$<AND:$<BOOL:${ITERA_LIO_NATIVE_LOOP_SEARCH}>,$<BOOL:${ITERA_LIO_NATIVE_LOOP_KNN}>>
  ITERA_MANUAL_DIRECT_INVERSE=$<BOOL:${ITERA_LIO_DIRECT_LOOP_INVERSE}>
  ITERA_MANUAL_DIRECT_EIGEN=$<BOOL:${ITERA_LIO_DIRECT_LOOP_EIGEN}>
  ITERA_MANUAL_FLOAT_BOUNDS=$<BOOL:${ITERA_LIO_FLOAT_LOOP_BOUNDS}>
  ITERA_MANUAL_SIMD_LEAVES=$<AND:$<BOOL:${ITERA_LIO_CACHED_LOOP_BOUNDS}>,$<BOOL:${ITERA_LIO_SIMD_LOOP_LEAVES}>>
  ITERA_MANUAL_SIMD_KNN=$<BOOL:${ITERA_LIO_SIMD_LOOP_KNN}>
  ITERA_MANUAL_AVX_LEAVES=$<BOOL:${ITERA_LIO_AVX_LOOP_LEAVES}>
  ITERA_MANUAL_LEAF_SIZE=${ITERA_LIO_LOOP_LEAF_SIZE})

option(ITERA_LIO_FIXED_BACKEND_BLOCKS "Use fixed-size 6-DoF pose-edge matrices" ON)
option(ITERA_LIO_CACHED_BACKEND_SPARSE "Reuse checked sparse coefficient offsets inside a solve" ON)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(sensor_buffer
  src/sensor/sensor_buffer.cpp
)
add_library(color_mapping
  src/mapping/color/colorizer.cpp
  src/mapping/color/colored_global_map.cpp
)
target_compile_features(color_mapping PUBLIC cxx_std_17)
target_include_directories(color_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(color_mapping sensor_msgs cv_bridge)
target_link_libraries(color_mapping Eigen3::Eigen)
target_compile_features(sensor_buffer PUBLIC cxx_std_17)
target_include_directories(sensor_buffer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sensor_buffer Eigen3::Eigen)

add_library(measurement_synchronizer
  src/sensor/measurement_synchronizer.cpp
)
target_compile_features(measurement_synchronizer PUBLIC cxx_std_17)
target_include_directories(measurement_synchronizer PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(measurement_synchronizer sensor_buffer Eigen3::Eigen)

add_library(estimator_state
  src/estimator/state.cpp
)
target_compile_features(estimator_state PUBLIC cxx_std_17)
target_include_directories(estimator_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(estimator_state Eigen3::Eigen)

add_library(imu_initializer
  src/estimator/imu_initializer.cpp
)
target_compile_features(imu_initializer PUBLIC cxx_std_17)
target_include_directories(imu_initializer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_initializer estimator_state Eigen3::Eigen)

add_library(imu_propagator
  src/estimator/imu_propagator.cpp
)
target_compile_features(imu_propagator PUBLIC cxx_std_17)
target_include_directories(imu_propagator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_propagator estimator_state Eigen3::Eigen)

add_library(lidar_parser
  src/sensor/lidar_parser.cpp
)
target_compile_features(lidar_parser PUBLIC cxx_std_17)
target_include_directories(lidar_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lidar_parser sensor_msgs)
target_link_libraries(lidar_parser Eigen3::Eigen)

add_library(deskewer
  src/frontend/deskewer.cpp
)
target_compile_features(deskewer PUBLIC cxx_std_17)
target_include_directories(deskewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(deskewer estimator_state Eigen3::Eigen OpenMP::OpenMP_CXX)

add_library(lidar_preprocessor
  src/frontend/lidar_preprocessor.cpp
)
target_compile_features(lidar_preprocessor PUBLIC cxx_std_17)
target_include_directories(lidar_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lidar_preprocessor Eigen3::Eigen)

add_library(imu_trajectory
  src/system/imu_trajectory.cpp
)
target_compile_features(imu_trajectory PUBLIC cxx_std_17)
target_include_directories(imu_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_trajectory Eigen3::Eigen)

add_library(ikd_tree_vendor
  third_party/ikd_tree/src/ikd_Tree.cpp
)
target_compile_features(ikd_tree_vendor PUBLIC cxx_std_17)
target_include_directories(ikd_tree_vendor PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ikd_tree/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ikd_tree_vendor ${PCL_LIBRARIES} pthread)

add_library(ikd_map
  src/frontend/ikd_map.cpp
)
target_compile_features(ikd_map PUBLIC cxx_std_17)
target_include_directories(ikd_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ikd_map PRIVATE ikd_tree_vendor ${PCL_LIBRARIES})
target_link_libraries(ikd_map PUBLIC Eigen3::Eigen)

add_library(global_map
  src/frontend/global_map.cpp
)
target_compile_features(global_map PUBLIC cxx_std_17)
target_include_directories(global_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(global_map Eigen3::Eigen)

add_library(scan_to_map
  src/frontend/plane_fitter.cpp
  src/frontend/scan_to_map.cpp
)
target_compile_features(scan_to_map PUBLIC cxx_std_17)
target_include_directories(scan_to_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(scan_to_map estimator_state Eigen3::Eigen OpenMP::OpenMP_CXX)

add_library(lidar_update
  src/estimator/lidar_update.cpp
)
target_compile_features(lidar_update PUBLIC cxx_std_17)
target_include_directories(lidar_update PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lidar_update scan_to_map estimator_state Eigen3::Eigen)

add_library(degeneracy_detector
  src/estimator/degeneracy_detector.cpp
)
target_compile_features(degeneracy_detector PUBLIC cxx_std_17)
target_include_directories(degeneracy_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(degeneracy_detector lidar_update estimator_state Eigen3::Eigen)

add_library(tracking_state
  src/estimator/tracking_state.cpp
)
target_compile_features(tracking_state PUBLIC cxx_std_17)
target_include_directories(tracking_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tracking_state Eigen3::Eigen)

add_library(esikf
  src/estimator/esikf.cpp
)
target_compile_features(esikf PUBLIC cxx_std_17)
target_include_directories(esikf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(esikf lidar_update scan_to_map estimator_state Eigen3::Eigen)

add_library(local_map_manager
  src/frontend/local_map_manager.cpp
)
target_compile_features(local_map_manager PUBLIC cxx_std_17)
target_include_directories(local_map_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(local_map_manager Eigen3::Eigen)

add_library(pose_graph
  src/backend/pose_graph.cpp
)
target_compile_features(pose_graph PUBLIC cxx_std_17)
target_include_directories(pose_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose_graph Eigen3::Eigen)

add_library(keyframe_manager
  src/backend/keyframe_manager.cpp
)
target_compile_features(keyframe_manager PUBLIC cxx_std_17)
target_include_directories(keyframe_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(keyframe_manager pose_graph Eigen3::Eigen)

add_library(pose_graph_optimizer
  src/backend/pose_graph_optimizer.cpp
)
target_compile_features(pose_graph_optimizer PUBLIC cxx_std_17)
target_compile_definitions(pose_graph_optimizer PRIVATE
  ITERA_BACKEND_FIXED_BLOCKS=$<BOOL:${ITERA_LIO_FIXED_BACKEND_BLOCKS}>
  ITERA_BACKEND_CACHED_SPARSE=$<BOOL:${ITERA_LIO_CACHED_BACKEND_SPARSE}>)
target_include_directories(pose_graph_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose_graph_optimizer pose_graph Eigen3::Eigen)

add_library(backend_manager
  src/backend/backend_manager.cpp
)
target_compile_features(backend_manager PUBLIC cxx_std_17)
target_include_directories(backend_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(backend_manager keyframe_manager pose_graph_optimizer Eigen3::Eigen)

add_library(manual_gicp src/loop/manual_gicp.cpp)
target_compile_features(manual_gicp PUBLIC cxx_std_17)
target_compile_definitions(manual_gicp PRIVATE ${ITERA_LOOP_CORE_DEFINITIONS})
target_include_directories(manual_gicp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(manual_gicp PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(manual_gicp PUBLIC Eigen3::Eigen OpenMP::OpenMP_CXX PRIVATE ${PCL_LIBRARIES})

add_library(loop_closure
  src/loop/loop_types.cpp
  src/loop/keyframe_cloud_store.cpp
  src/loop/submap_builder.cpp
  src/loop/loop_detector.cpp
  src/loop/loop_verifier.cpp
  src/loop/overlap_audit.cpp
  src/loop/loop_alignment.cpp
  src/loop/loop_pair_archive.cpp
  src/loop/loop_closure_manager.cpp
)
option(ITERA_LIO_EXACT_LOOP_SCORING "Use bounded exact SIMD 1NN scores and reciprocal coverage" ON)
target_compile_features(loop_closure PUBLIC cxx_std_17)
target_compile_definitions(loop_closure PRIVATE ${ITERA_LOOP_CORE_DEFINITIONS}
  ITERA_LIO_CACHE_LOOP_SOURCE=$<BOOL:${ITERA_LIO_CACHE_LOOP_SOURCE}>
  ITERA_LIO_EXACT_LOOP_SCORING=$<BOOL:${ITERA_LIO_EXACT_LOOP_SCORING}>)
target_include_directories(loop_closure PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(loop_closure PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(loop_closure backend_manager manual_gicp Eigen3::Eigen ${PCL_LIBRARIES})

add_executable(loop_pair_replay tools/loop_pair_replay.cpp)
target_compile_features(loop_pair_replay PRIVATE cxx_std_17)
target_include_directories(loop_pair_replay PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(loop_pair_replay loop_closure ${PCL_LIBRARIES})

add_library(lio_pipeline
  src/system/lio_pipeline.cpp
)
target_compile_features(lio_pipeline PUBLIC cxx_std_17)
target_include_directories(lio_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lio_pipeline
  esikf degeneracy_detector tracking_state scan_to_map ikd_map global_map
  local_map_manager backend_manager loop_closure Eigen3::Eigen)

add_library(lio_processor
  src/system/lio_processor.cpp
)
target_compile_features(lio_processor PUBLIC cxx_std_17)
target_include_directories(lio_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lio_processor
  sensor_buffer imu_initializer imu_propagator deskewer lidar_preprocessor imu_trajectory
  lio_pipeline Eigen3::Eigen)

add_library(lio_output_support
  src/output/image_recorder.cpp
  src/output/evaluation_logger.cpp
  src/output/evaluation_csv.cpp
  src/output/evaluation_summary.cpp
  src/output/scan_archive.cpp
  src/output/static_map_filter.cpp
  src/output/static_map_export.cpp
  src/output/pcd_writer.cpp
  src/output/graph_export.cpp
  src/output/archive_pose_correction.cpp
  src/output/full_cloud_writer.cpp
  src/output/colored_cloud_writer.cpp
  src/output/optimized_colored_export.cpp
  src/output/preview_cloud.cpp
)
target_compile_features(lio_output_support PUBLIC cxx_std_17)
target_include_directories(lio_output_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lio_output_support cv_bridge sensor_msgs)
target_link_libraries(lio_output_support
  color_mapping global_map lio_processor Eigen3::Eigen)

add_library(lio_ros_support
  src/ros/diagnostics_reporter.cpp
  src/ros/message_conversions.cpp
  src/ros/input_adapter.cpp
  src/ros/parameter_reader.cpp
  src/ros/services/map_preview_server.cpp
  src/ros/services/reconstruction_services.cpp
  src/ros/config/node_config.cpp
  src/ros/config/sensor_config.cpp
  src/ros/config/frontend_config.cpp
  src/ros/config/backend_config.cpp
  src/ros/config/loop_config.cpp
  src/ros/config/output_config.cpp
  src/ros/config/camera_config.cpp
  src/ros/config/evaluation_config.cpp
  src/ros/publishers/output_publisher.cpp
  src/ros/publishers/state_publisher.cpp
  src/ros/publishers/cloud_publisher.cpp
  src/ros/publishers/map_preview_publisher.cpp
  src/ros/publishers/colored_cloud_publisher.cpp
  src/ros/publishers/optimized_map_publisher.cpp
  src/ros/map_export_controller.cpp
  src/ros/evaluation_reporter.cpp
)
target_compile_features(lio_ros_support PUBLIC cxx_std_17)
target_include_directories(lio_ros_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(lio_ros_support ament_index_cpp cv_bridge geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs tf2_ros)
target_link_libraries(lio_ros_support
  lio_output_support lio_pipeline measurement_synchronizer lidar_parser
  Eigen3::Eigen ${PCL_LIBRARIES})

# Layer facade targets keep targets inside this package independent from the
# current per-module implementation split. Existing module targets remain
# available for focused tests and incremental builds.
add_library(itera_lio_common INTERFACE)
target_include_directories(itera_lio_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(itera_lio_sensor INTERFACE)
target_link_libraries(itera_lio_sensor INTERFACE
  sensor_buffer measurement_synchronizer lidar_parser Eigen3::Eigen)
target_include_directories(itera_lio_sensor INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(itera_lio_frontend INTERFACE)
target_link_libraries(itera_lio_frontend INTERFACE
  deskewer lidar_preprocessor ikd_map global_map scan_to_map local_map_manager Eigen3::Eigen)
target_include_directories(itera_lio_frontend INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(itera_lio_estimator INTERFACE)
target_link_libraries(itera_lio_estimator INTERFACE
  estimator_state imu_initializer imu_propagator lidar_update degeneracy_detector
  tracking_state esikf Eigen3::Eigen)
target_include_directories(itera_lio_estimator INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(itera_lio_backend INTERFACE)
target_link_libraries(itera_lio_backend INTERFACE
  pose_graph keyframe_manager pose_graph_optimizer backend_manager Eigen3::Eigen)
target_include_directories(itera_lio_backend INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(itera_lio_system INTERFACE)
target_link_libraries(itera_lio_system INTERFACE
  imu_trajectory lio_pipeline lio_processor itera_lio_sensor itera_lio_frontend itera_lio_estimator itera_lio_backend
  Eigen3::Eigen)
target_include_directories(itera_lio_system INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(itera_lio_mapping INTERFACE)
target_link_libraries(itera_lio_mapping INTERFACE
  color_mapping Eigen3::Eigen)
target_include_directories(itera_lio_mapping INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(itera_lio_output INTERFACE)
target_link_libraries(itera_lio_output INTERFACE
  lio_output_support Eigen3::Eigen)
target_include_directories(itera_lio_output INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(itera_lio_node src/itera_lio_node.cpp)
target_compile_features(itera_lio_node PUBLIC cxx_std_17)
target_link_libraries(itera_lio_node
  itera_lio_system itera_lio_mapping itera_lio_output lio_ros_support
  ${PCL_LIBRARIES})
target_include_directories(itera_lio_node PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(itera_lio_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

add_executable(filter_static_map tools/filter_static_map.cpp)
target_compile_features(filter_static_map PUBLIC cxx_std_17)
target_link_libraries(filter_static_map lio_output_support)

install(TARGETS filter_static_map
  color_mapping lio_output_support
  estimator_state imu_initializer imu_propagator lidar_parser deskewer lidar_preprocessor
  imu_trajectory ikd_tree_vendor ikd_map global_map
  scan_to_map lidar_update degeneracy_detector tracking_state esikf lio_pipeline lio_processor
  lio_ros_support
  local_map_manager sensor_buffer measurement_synchronizer pose_graph keyframe_manager pose_graph_optimizer backend_manager itera_lio_node
  loop_closure manual_gicp loop_pair_replay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Vendored LIO distributions may omit the original workspace's offline tools.
# They are not needed for online LIO or geometric PCD export.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/refine_saved_run.py")
  install(PROGRAMS ../../tools/refine_saved_run.py
    DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY ../../tools/
    DESTINATION share/${PROJECT_NAME}/refinement
    FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
else()
  message(STATUS "Optional offline refinement tools absent; skipping their installation")
endif()
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_static_map_filter test/test_static_map_filter.cpp)
  target_link_libraries(test_static_map_filter lio_output_support)
  find_package(tf2_msgs REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Preserve vendored upstream formatting and licensing headers.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE third_party/ikd_tree)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_output_components test/test_output_components.cpp)
  if(TARGET test_output_components)
    target_link_libraries(test_output_components lio_ros_support)
    ament_target_dependencies(test_output_components tf2_msgs)
    set_tests_properties(test_output_components PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=161;ROS_LOCALHOST_ONLY=1")
  endif()

  ament_add_gtest(test_node_config test/test_node_config.cpp)
  if(TARGET test_node_config)
    target_link_libraries(test_node_config lio_ros_support)
    target_compile_definitions(test_node_config PRIVATE
      ITERA_LIO_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endif()

  ament_add_gtest(test_sensor_buffer test/test_sensor_buffer.cpp)
  if(TARGET test_sensor_buffer)
    target_link_libraries(test_sensor_buffer sensor_buffer)
  endif()

  ament_add_gtest(test_state test/test_state.cpp)
  if(TARGET test_state)
    target_link_libraries(test_state estimator_state Eigen3::Eigen)
  endif()

  ament_add_gtest(test_imu_initializer test/test_imu_initializer.cpp)
  if(TARGET test_imu_initializer)
    target_link_libraries(test_imu_initializer imu_initializer Eigen3::Eigen)
  endif()

  ament_add_gtest(test_imu_propagator test/test_imu_propagator.cpp)
  if(TARGET test_imu_propagator)
    target_link_libraries(test_imu_propagator imu_propagator Eigen3::Eigen)
  endif()

  ament_add_gtest(test_so3 test/test_so3.cpp)
  if(TARGET test_so3)
    target_include_directories(test_so3 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_so3 Eigen3::Eigen)
  endif()

  ament_add_gtest(test_lidar_parser test/test_lidar_parser.cpp)
  if(TARGET test_lidar_parser)
    target_link_libraries(test_lidar_parser lidar_parser Eigen3::Eigen)
  endif()

  ament_add_gtest(test_deskewer test/test_deskewer.cpp)
  if(TARGET test_deskewer)
    target_link_libraries(test_deskewer deskewer Eigen3::Eigen)
  endif()

  ament_add_gtest(test_lidar_preprocessor test/test_lidar_preprocessor.cpp)
  if(TARGET test_lidar_preprocessor)
    target_link_libraries(test_lidar_preprocessor lidar_preprocessor Eigen3::Eigen)
  endif()

  ament_add_gtest(test_imu_trajectory test/test_imu_trajectory.cpp)
  if(TARGET test_imu_trajectory)
    target_link_libraries(test_imu_trajectory imu_trajectory Eigen3::Eigen)
  endif()

  ament_add_gtest(test_ikd_map test/test_ikd_map.cpp)
  ament_add_gtest(test_readonly_tree test/test_readonly_tree.cpp)
  if(TARGET test_readonly_tree)
    target_link_libraries(test_readonly_tree ikd_tree_vendor)
  endif()
  if(TARGET test_ikd_map)
    target_link_libraries(test_ikd_map ikd_map Eigen3::Eigen)
  endif()

  ament_add_gtest(test_global_map test/test_global_map.cpp)
  if(TARGET test_global_map)
    target_link_libraries(test_global_map global_map Eigen3::Eigen)
  endif()
  ament_add_gtest(test_colored_global_map test/test_colored_global_map.cpp)
  if(TARGET test_colored_global_map)
    target_link_libraries(test_colored_global_map lio_output_support Eigen3::Eigen)
  endif()

  ament_add_gtest(test_colorizer test/test_colorizer.cpp)
  if(TARGET test_colorizer)
    target_link_libraries(test_colorizer color_mapping estimator_state Eigen3::Eigen)
  endif()

  ament_add_gtest(test_preview_cloud test/test_preview_cloud.cpp)
  if(TARGET test_preview_cloud)
    target_link_libraries(test_preview_cloud lio_output_support Eigen3::Eigen)
  endif()

  ament_add_gtest(test_plane_fitter test/test_plane_fitter.cpp)
  if(TARGET test_plane_fitter)
    target_link_libraries(test_plane_fitter scan_to_map Eigen3::Eigen)
  endif()

  ament_add_gtest(test_scan_to_map test/test_scan_to_map.cpp)
  if(TARGET test_scan_to_map)
    target_link_libraries(test_scan_to_map scan_to_map ikd_map Eigen3::Eigen)
  endif()

  ament_add_gtest(test_lidar_update test/test_lidar_update.cpp)
  if(TARGET test_lidar_update)
    target_link_libraries(test_lidar_update lidar_update Eigen3::Eigen)
  endif()

  ament_add_gtest(test_esikf test/test_esikf.cpp)
  if(TARGET test_esikf)
    target_link_libraries(test_esikf esikf Eigen3::Eigen)
  endif()

  ament_add_gtest(test_lio_pipeline test/test_lio_pipeline.cpp)
  if(TARGET test_lio_pipeline)
    target_link_libraries(test_lio_pipeline lio_pipeline Eigen3::Eigen)
  endif()

  ament_add_gtest(test_lio_processor test/test_lio_processor.cpp)
  if(TARGET test_lio_processor)
    target_link_libraries(test_lio_processor lio_processor Eigen3::Eigen)
  endif()

  ament_add_gtest(test_degeneracy_detector test/test_degeneracy_detector.cpp)
  if(TARGET test_degeneracy_detector)
    target_link_libraries(test_degeneracy_detector degeneracy_detector Eigen3::Eigen)
  endif()

  ament_add_gtest(test_tracking_state test/test_tracking_state.cpp)
  if(TARGET test_tracking_state)
    target_link_libraries(test_tracking_state tracking_state Eigen3::Eigen)
  endif()

  ament_add_gtest(test_local_map_manager test/test_local_map_manager.cpp)
  if(TARGET test_local_map_manager)
    target_link_libraries(test_local_map_manager local_map_manager ikd_map Eigen3::Eigen)
  endif()

  ament_add_gtest(test_backend_manager test/test_backend_manager.cpp)
  if(TARGET test_backend_manager)
    target_link_libraries(test_backend_manager backend_manager pose_graph_optimizer keyframe_manager pose_graph Eigen3::Eigen)
  endif()

  ament_add_gtest(test_exact_kdtree test/test_exact_kdtree.cpp test/test_exact_knn.cpp)
  target_include_directories(test_exact_kdtree PRIVATE include)
  target_compile_definitions(test_exact_kdtree PRIVATE ${ITERA_LOOP_CORE_DEFINITIONS})
  ament_add_gtest(test_manual_gicp test/test_manual_gicp.cpp)
  if(TARGET test_manual_gicp)
    target_link_libraries(test_manual_gicp manual_gicp loop_closure)
    set_tests_properties(test_manual_gicp PROPERTIES ENVIRONMENT "ITERA_LIO_TEST_ARTIFACTS=${CMAKE_CURRENT_BINARY_DIR}/test_artifacts")
  endif()
  ament_add_gtest(test_loop_closure test/test_loop_closure.cpp)
  if(TARGET test_loop_closure)
    target_link_libraries(test_loop_closure loop_closure backend_manager pose_graph_optimizer keyframe_manager pose_graph Eigen3::Eigen)
  endif()

  ament_add_gtest(test_evaluation_logger test/test_evaluation_logger.cpp)
  if(TARGET test_evaluation_logger)
    target_link_libraries(test_evaluation_logger lio_output_support Eigen3::Eigen)
  endif()
endif()

if(BUILD_TESTING)
  add_executable(b010_backend_graph_replay tools/b010_backend_graph_replay.cpp)
  add_executable(backend_prefix_diagnostic tools/backend_prefix_diagnostic.cpp)
  target_compile_features(backend_prefix_diagnostic PRIVATE cxx_std_17)
  target_link_libraries(backend_prefix_diagnostic backend_manager pose_graph_optimizer keyframe_manager pose_graph Eigen3::Eigen)
  target_compile_features(b010_backend_graph_replay PRIVATE cxx_std_17)
  target_link_libraries(b010_backend_graph_replay pose_graph_optimizer pose_graph Eigen3::Eigen)
  add_custom_target(b010_screen DEPENDS loop_pair_replay test_exact_kdtree
    test_manual_gicp test_loop_closure test_backend_manager b010_backend_graph_replay)
endif()
ament_export_include_directories(include)
ament_export_libraries(
  color_mapping lio_output_support
  estimator_state imu_initializer imu_propagator lidar_parser deskewer lidar_preprocessor
  imu_trajectory ikd_map global_map
  scan_to_map lidar_update degeneracy_detector esikf lio_pipeline lio_processor lio_ros_support
  local_map_manager sensor_buffer pose_graph keyframe_manager pose_graph_optimizer backend_manager
  loop_closure)
ament_export_dependencies(
  builtin_interfaces Eigen3 geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs tf2_ros)
ament_package()
