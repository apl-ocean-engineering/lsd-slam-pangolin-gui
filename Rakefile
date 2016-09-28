
require 'pathname'

root_dir = Pathname.new(__FILE__).parent
build_root = ENV['LSDSLAM_BUILD_DIR'] || "build"

builds = %w( release debug )

builds.each do |build|

  namespace build do

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
        args = %W(-DCMAKE_BUILD_TYPE:string=#{build}
              #{ENV['CMAKE_FLAGS']}
              -DEXTERNAL_PROJECT_PARALLELISM:string=0 #{root_dir})
        sh "cmake", *args
        sh "make deps && make"
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
namespace :docker do
  docker_image = "lsdslam:test"

  desc "Build test docker image."
  task :build_image do
    sh "cd .docker && docker build -t #{docker_image} ."
  end

  docker_run_opts = %W( --rm
                        -v #{root_dir}:/opt/lsd_slam
                        #{docker_image}).join(' ')

  builds.each do |build|

    namespace build do

      desc "Make #{build} in Docker"
      task :make do
        sh "cd .docker && docker run #{docker_run_opts} #{build}:make"
      end

      desc "Run #{build} tests in Docker"
      task :tests do
        sh "cd .docker && docker run #{docker_run_opts} #{build}:tests"
      end

    end
  end

  desc "Open console in Docker"
  task :console do
    sh "cd .docker && docker run -ti --entrypoint \"/bin/bash\" #{docker_run_opts} "
  end
end
