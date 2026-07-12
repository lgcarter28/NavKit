# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

find_package(doctest REQUIRED CONFIG)

add_executable(navkit_tests
    test_main.cpp
    test_config_policy.cpp
    test_injection_reset_policy.cpp
    test_log_product_policy.cpp
    test_measurement_model_policy.cpp
    test_measurement_statistics.cpp
    test_noise_policy.cpp
    test_profile_csv_writer.cpp
    test_profiling_policy.cpp
    test_ring_buffer.cpp
    test_ring_buffer_policies.cpp
    test_runtime_config_validation.cpp
    test_sensor_behavior.cpp
    test_segment.cpp
    test_state_def_policy.cpp
    test_units_frames.cpp
    test_csv_writer.cpp
    test_environment_policies.cpp
    test_gnss_pos_update.cpp
    test_imu_simulator.cpp
    test_navigator_compile.cpp
    test_navigator_policy.cpp
    test_trajectory_generator.cpp
)

target_link_libraries(navkit_tests PRIVATE navkit::core navkit::sim navkit::io doctest::doctest)
target_include_directories(navkit_tests
    PRIVATE
        ${PROJECT_SOURCE_DIR}/config/compiletime
        ${NAVKIT_GENERATED_INCLUDE_DIR}
)

add_test(NAME navkit_tests COMMAND navkit_tests)
navkit_configure_target(navkit_tests)
