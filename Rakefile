
require 'pathname'

$:.unshift File.dirname(__FILE__) + "/.rb"
require 'docker'

def root_dir
  Pathname.new(__FILE__).parent
end

@cmake_opts = ['-D BUILD_UNIT_TESTS:BOOL=True']
load 'config.rb' if FileTest::exists? 'config.rb'

build_root = ENV['LSDSLAM_BUILD_DIR'] || "build"

task :default => "debug:test"

builds = %w( release debug )
builds.each do |build|

  deps_touchfile = '.DEPS_MADE'

  namespace build do

    cmake_args = %W(-DCMAKE_BUILD_TYPE:string=#{build}
          #{ENV['CMAKE_FLAGS']}
          -DEXTERNAL_PROJECT_PARALLELISM:string=0 #{root_dir})

    build_dir = [build_root, build].join('_')
    build_dir = root_dir.join(build_dir)

    directory build_dir.to_s do |t|
      FileUtils::mkdir t.name
    end

    # On my system I need to specify the location of OpenCV by hand:
    #
    # n.b. CMAKE_FLAGS='-DOpenCV_DIR=/opt/opencv-2.4/share/opencv' rake debug:make
    #
    desc "Make lsd_slam for #{build}"
    task :build => build_dir do
      ##-Bbuild_ci -H.
      chdir build_dir do
        sh "cmake", *cmake_args
        sh "make deps && touch #{deps_touchfile}" unless File.readable? deps_touchfile
        sh "make"
      end
    end

    ## Force make deps
    desc "Force make deps for #{build}"
    task :deps => build_dir do
      chdir build_dir do
        sh "cmake", *cmake_args
        FileUtils.rm deps_touchfile
        sh "make deps && touch #{deps_touchfile}"
      end
    end

    task :clean => build_dir do
      chdir build_dir do
        sh "make clean"
      end
    end

    desc "Run all tests"
    task :test => [ :build, "test:unit" ]

    namespace :test do
      desc "Unit tests for #{build}"
      task :unit do
        chdir build_dir do
          sh "make unit_test"
        end
      end
    end

  end

end

DockerTasks.new( builds: builds )

#
# Platform-specific tasks for installing dependencies
#
namespace :dependencies do

  desc "Install dependencies for Ubuntu trusty"
  task :trusty do
    sh "sudo apt-get update &&
        sudo apt-get install -y cmake \
      		libopencv-dev libboost-all-dev libeigen3-dev \
      		libtclap-dev libgomp1 libsuitesparse-dev git \
      		libglew-dev libglm-dev autoconf libtool freeglut3-dev"
  end


  task :osx do
    sh "brew update"
    sh "brew tap homebrew/science"
    sh "brew install homebrew/science/opencv"
  end

  ## Travis-specific depenendcy rules
  namespace :travis do

    task :trusty => "dependencies:trusty"

    task :osx do
      sh "pip uninstall numpy"
      sh "brew update"
      sh "brew tap homebrew/science"
      sh "brew install homebrew/science/opencv"
    end

  end



end
