from conans import ConanFile, CMake

class LsdSlamConan(ConanFile):
  name = "lsd_slam_conan"
  version = "master"
  settings = "os", "compiler", "build_type", "arch"
  generators = "cmake"
  options = {"opencv_dir": "ANY",  "build_parallel": [True, False]}
  default_options = "opencv_dir=''", "build_parallel=True"
  exports = ['lib/*', 'include/**', 'test/', 'CMakeLists.txt', 'Rakefile', 'conanfile.py', '.rb/']
  requires = "lsd_slam/master@amarburg/testing", \
             "libvideoio/master@amarburg/testing"

  def config(self):
    #self.options['lsd_slam'].build_parallel = self.options.build_parallel

    ## Which of these are strictly necessary?
    self.options['lsd_slam'].opencv_dir = self.options.opencv_dir
    self.options['liblogger'].opencv_dir = self.options.opencv_dir
    self.options['libvideoio'].opencv_dir = self.options.opencv_dir
    self.options['g2o_conan'].build_type = 'Release'

    if self.scope.dev and self.scope.build_tests:
      self.requires( "gtest/1.8.0@lasote/stable" )
      self.options["gtest"].shared = False

  def imports(self):
    self.copy("*.dll", dst="bin", src="bin") # From bin to bin
    self.copy("*.dylib*", dst="bin", src="lib") # From lib to bin

  def build(self):
    cmake = CMake(self.settings)
    build_opts=""
    if self.options.opencv_dir:
      cmake_opts = "-DOpenCV_DIR=%s" % (self.options.opencv_dir)

    build_opts = "-j" if self.options.build_parallel else ""

    flag_build_tests = "-DBUILD_UNIT_TESTS=1" if self.scope.dev and self.scope.build_tests else ""

    self.run('cmake "%s" %s %s %s' % (self.conanfile_directory, cmake.command_line, cmake_opts, flag_build_tests))
    self.run('cmake --build . %s -- %s' % (cmake.build_config, build_opts) )
    if self.scope.dev and self.scope.build_tests:
      self.run('make unit_test')

  def package(self):
    self.copy("*.h", dst="")
    #if self.options.shared:
    if self.settings.os == "Macos":
        self.copy(pattern="*.dylib", dst="lib", keep_path=False)
    else:
        self.copy(pattern="*.so*", dst="lib", src="lib", keep_path=False)
    #else:
    #    self.copy(pattern="*.a", dst="lib", src="lib", keep_path=False)

  #def package_info(self):
      #self.cpp_info.libs = ["lsdslam"]
