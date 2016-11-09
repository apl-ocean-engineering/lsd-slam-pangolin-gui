
task :default => "debug:test"

@build_opts = {}
load 'config.rb' if FileTest::exists? 'config.rb'

['Debug','Release'].each { |build_type|
  namespace build_type.downcase.to_sym do
    build_dir = ENV['BUILD_DIR'] || "build-#{build_type}"

    task :build do
      FileUtils::mkdir build_dir unless FileTest::directory? build_dir
      opencv_opt = "-o opencv_dir=%s" % @build_opts[:opencv_dir] if @build_opts[:opencv_dir]
      sh "cd %s && conan install --scope build_tests=True -s build_type=%s %s .. --build=missing" % [build_dir, build_type, opencv_opt]
      sh "cd %s && conan build .." % [build_dir]
    end

    task :test => :build do
      sh "cd %s && make unit_test" % build_dir
    end

  end
}

namespace :conan do
  task :export do
    sh "rm -rf build-*"
    sh "conan export amarburg/testing"
  end
end

namespace :dependencies do

  task :trusty do
    sh "sudo apt-get install -y cmake libopencv-dev libtclap-dev libboost-all-dev"
    sh "pip install conan"
  end

  task :osx do
    sh "brew update"
    sh "brew tap homebrew/science"
    sh "brew install homebrew/science/opencv tclap conan"
  end

  namespace :travis do

    task :linux => "dependencies:trusty"

    task :osx => [:pip_uninstall_numpy, "dependencies:osx"]

    task :pip_uninstall_numpy do
      sh "pip uninstall -y numpy"
    end

  end
end
