from conan import ConanFile
from conan.tools.cmake import cmake_layout


class ExampleRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeDeps", "CMakeToolchain", "cmake_find_package", "cmake"

    def requirements(self):
        self.requires("osqp/0.6.3")

    def layout(self):
        cmake_layout(self)
