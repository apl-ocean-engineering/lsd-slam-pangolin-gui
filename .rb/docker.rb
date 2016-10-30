require 'rake'

#
# Tasks for creating a Docker instance for testing
#

class DockerTasks

  include Rake::DSL

  def initialize( opts = {})

    @builds = opts[:builds] || %w( debug release )

    @docker_path = Pathname.new(__FILE__).parent.parent.join('.docker')

    define_tasks
  end


  def docker_image
    "lsdslam:test"
  end

  def docker_run_opts
    %W( --rm
        -v #{root_dir}:/opt/lsd_slam
        #{docker_image})
  end

  def in_docker
    chdir @docker_path do
      yield
    end
  end

  def docker_run *args
    in_docker do
      sh *(['docker', 'run'] + docker_run_opts + args)
    end
  end


  def define_tasks


    namespace :docker do

      desc "Build test docker image."
      task :build_image do
        in_docker {
          sh "docker build -t #{docker_image} ."
        }
      end

      @builds.each do |build|

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
          args = %w(docker run -ti --entrypoint /bin/bash) + docker_run_opts
          sh *args
        }
      end
    end

  end


end
