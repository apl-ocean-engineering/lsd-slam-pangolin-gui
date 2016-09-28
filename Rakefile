
require 'pathname'

def root_dir
  Pathname.new(__FILE__).parent
end

build_root = ENV['LSDSLAM_BUILD_DIR'] || "build"

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
    task :make => build_dir do
      ##-Bbuild_ci -H.
      chdir build_dir do
        sh "cmake", *cmake_args
        sh "make deps && touch #{deps_touchfile}" unless File.readable? deps_touchfile
        sh "make"
      end
    end

    ## Force make deps
    desc "Force make deps for #{build}"
    task :make_deps => build_dir do
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
    task :tests => [ "test:unit" ]

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

end


#
# Tasks for creating a Docker instance for testing
#

def docker_image
  "lsdslam:test"
end


def docker_run_opts
    %W( --rm
        -v #{root_dir}:/opt/lsd_slam
        #{docker_image})
end

def in_docker
  chdir ".docker" do
      yield
  end
end

def docker_run *args
  in_docker do
    sh *(['docker', 'run'] + docker_run_opts + args)
  end
end

namespace :docker do

  desc "Build test docker image."
  task :build_image do
    in_docker {
      sh "docker build -t #{docker_image} ."
    }
  end

  builds.each do |build|

    namespace build do

      desc "Make deps for #{build} in Docker"
      task :make_deps do
        docker_run "#{build}:make_deps"
      end

      desc "Make #{build} in Docker"
      task :make do
        docker_run "#{build}:make"
      end

      desc "Run #{build} tests in Docker"
      task :tests do
        docker_run "#{build}:tests"
      end

      task :clean do
        docker_run "#{build}:clean"
      end

    end
  end

  desc "Open console in Docker"
  task :console do
    in_docker {
      sh "docker run -ti --entrypoint \"/bin/bash\" #{docker_run_opts} "
    }
  end
end
