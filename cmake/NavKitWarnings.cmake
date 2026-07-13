# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

option(NAVKIT_ENABLE_WARNINGS "Enable NavKit warning flags on NavKit-owned targets" ON)
option(NAVKIT_WARNINGS_AS_ERRORS "Treat NavKit-owned target warnings as errors" OFF)
option(NAVKIT_ENABLE_COVERAGE "Enable coverage instrumentation for NavKit-owned Debug targets" OFF)

function(navkit_apply_warnings target_name)
    if(NOT NAVKIT_ENABLE_WARNINGS)
        return()
    endif()

    if(MSVC)
        target_compile_options(${target_name}
            PRIVATE
                /W4
                /bigobj
                /permissive-
                /Zc:__cplusplus
                $<$<BOOL:${NAVKIT_WARNINGS_AS_ERRORS}>:/WX>
        )
    else()
        target_compile_options(${target_name}
            PRIVATE
                -Wall
                -Wextra
                -Wpedantic
                -Wconversion
                -Wsign-conversion
                -Wshadow
                $<$<BOOL:${NAVKIT_WARNINGS_AS_ERRORS}>:-Werror>
        )
    endif()
endfunction()

function(navkit_apply_release_optimizations target_name)
    if(MSVC)
        target_compile_options(${target_name}
            PRIVATE
                $<$<CONFIG:Release>:/O2>
                $<$<CONFIG:Release>:/Ob2>
                $<$<CONFIG:Release>:/Gy>
                $<$<CONFIG:Release>:/Gw>
        )
        target_link_options(${target_name}
            PRIVATE
                $<$<CONFIG:Release>:/OPT:REF>
                $<$<CONFIG:Release>:/OPT:ICF>
        )
    else()
        target_compile_options(${target_name}
            PRIVATE
                $<$<CONFIG:Release>:-O2>
                $<$<CONFIG:Release>:-ffunction-sections>
                $<$<CONFIG:Release>:-fdata-sections>
        )
        if(APPLE)
            target_link_options(${target_name}
                PRIVATE
                    $<$<CONFIG:Release>:-Wl,-dead_strip>
            )
        else()
            target_link_options(${target_name}
                PRIVATE
                    $<$<CONFIG:Release>:-Wl,--gc-sections>
            )
        endif()
    endif()
endfunction()

function(navkit_apply_debug_diagnostics target_name)
    if(MSVC)
        target_compile_options(${target_name}
            PRIVATE
                $<$<CONFIG:Debug>:/sdl>
        )
    else()
        target_compile_options(${target_name}
            PRIVATE
                $<$<CONFIG:Debug>:-fno-omit-frame-pointer>
        )
    endif()
endfunction()

function(navkit_apply_coverage target_name)
    if(NOT NAVKIT_ENABLE_COVERAGE)
        return()
    endif()

    if(MSVC)
        message(FATAL_ERROR "NAVKIT_ENABLE_COVERAGE is supported only with GCC/Clang-style coverage tooling.")
    endif()

    target_compile_options(${target_name}
        PRIVATE
            $<$<CONFIG:Debug>:--coverage>
            $<$<CONFIG:Debug>:-O0>
            $<$<CONFIG:Debug>:-g>
    )
    target_link_options(${target_name}
        PRIVATE
            $<$<CONFIG:Debug>:--coverage>
    )
endfunction()

function(navkit_configure_target target_name)
    navkit_apply_warnings(${target_name})
    navkit_apply_debug_diagnostics(${target_name})
    navkit_apply_release_optimizations(${target_name})
    navkit_apply_coverage(${target_name})
endfunction()
