# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMakeDeps, cmake_layout

class NavKitRecipe(ConanFile):
    name = "navkit"
    version = "0.1.0"
    package_type = "application"
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.requires("eigen/3.4.0")
        self.requires("nlohmann_json/3.11.3")
        self.requires("doctest/2.4.11")

    def layout(self):
        cmake_layout(self)

    def generate(self):
        deps = CMakeDeps(self)
        deps.generate()
        tc = CMakeToolchain(self)
        tc.generate()
