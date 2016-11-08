
$:.unshift File.dirname(__FILE__) + "/.rb"
require 'pathname'
require 'docker'

## Set defaults
@cmake_opts = ['-DBUILD_UNIT_TESTS:BOOL=True']
load 'config.rb' if FileTest::exists? 'config.rb'

build_root = ENV['LSDSLAM_BUILD_DIR'] || "build"

task :default => "debug:test"

builds = %w( release debug )
builds.each do |build|

  deps_touchfile = '.DEPS_MADE'

  namespace build do

    cmake_args = %W(-DCMAKE_BUILD_TYPE:string=#{build}
          #{ENV['CMAKE_FLAGS']}
          -DEXTERNAL_PROJECT_PARALLELISM:string=0 )

    build_dir = [build_root, build].join('_')

    desc "Make lsd_slam for #{build}"
    task :build => build_dir do
      mkdir build_dir unless FileTest.directory? build_dir
      chdir build_dir do
        sh "cmake % s .." % cmake_args
        sh "make deps && touch #{deps_touchfile}" unless File.readable? deps_touchfile
        sh "make"
      end
    end

    ## Force make deps
    desc "Force rebuild of the dependencies for #{build}"
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
    sh "brew install homebrew/science/opencv tclap"
  end

  ## Travis-specific depenendcy rules
  namespace :travis do

    task :linux => "dependencies:trusty"

    task :osx => [:pip_uninstall_numpy, "dependencies:osx"]

    task :pip_uninstall_numpy do
      sh "pip uninstall -y numpy"
    end

  end



end
