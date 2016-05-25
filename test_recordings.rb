#!/usr/bin/env ruby

stats_file = "recorder_stats.csv"

[ "hd1080", "hd720", "vga" ].each { |resolution|
	[false, true].each { |do_display|

		10.times.each { |i|
			output = "/mnt/disk/ubuntu/#{resolution}#{do_display ? "_display" : ""}_#{i}.svo"

			system( {"DISPLAY" => "tegra-ubuntu:1"}, "./ZedRecorder --svo-output #{output} --statistics-output #{stats_file} --resolution #{resolution} #{do_display ? "--display" : ""} --duration 10" )
		}

	}
}
