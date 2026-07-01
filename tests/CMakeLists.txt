# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

find_package(doctest REQUIRED CONFIG)

add_executable(navkit_tests
    test_main.cpp
    test_injection_reset_policy.cpp
    test_measurement_policy.cpp
    test_noise_policy.cpp
    test_ring_buffer.cpp
    test_segment.cpp
    test_state_def_policy.cpp
    test_units_frames.cpp
    test_environment_policies.cpp
    test_gnss_pos_update.cpp
    test_navigator_compile.cpp
)

target_link_libraries(navkit_tests PRIVATE navkit::core doctest::doctest)

add_test(NAME navkit_tests COMMAND navkit_tests)
