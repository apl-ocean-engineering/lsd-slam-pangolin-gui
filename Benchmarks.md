
SVO Recording Data Rates
----

All resolutions recorded over a 30 sec interval on a Core i7 w/ GTX970, writing to a Samsung 850 Pro SSD on SATA.

<table>
	<tr><th>Resolution</th><th>Duration (s)</th><th>Nom. FPS</th><th>Meas. FPS</th><th>File Size (MB)</th><th>Rate (MB/s)</th><tr>
 	<tr><td>HD2K</td><td>10</td><td>15</td><td>15.0802</td><td>1569</td><td>156.695</td></tr>
	<tr><td>HD1080</td><td>10</td><td>30</td><td>28.6462</td><td>2127</td><td>212.269</td></tr>
	<tr><td>"</td><td>30</td><td>30</td><td>26.9453</td><td>2627</td><td>87.5333</td></tr>
	<tr><td>HD720</td><td>10</td><td>60</td><td>59.6885</td><td>2091</td><td>209.06</td></tr>
	<tr><td>VGA</td><td>10</td><td>100</td><td>99.2432</td><td>1162</td><td>116.134</td></tr>
</table>



RawLog Recording Rates
----

Recorded file includes left and right images (as BGRA_8C), and depth map (as DEPTH_32F) w/ zlib compress (level 6) on all three channels.

<table>
	<tr><th>Resolution</th><th>Duration (s)</th><th>Compression</th><th>Nom. FPS</th><th>Meas. FPS</th><th>File Size (MB)</th><th>Rate (MB/s)</th><tr>
	<tr><td>HD1080</td><td>30</td><td>Snappy</td><td>30</td><td>14.8935</td><td>3196</td><td>106.487</td></tr>
</table>



SVO to Logger transcoding
-----

<table>
	<tr><th>Resolution</th><th>Duration (s)</th><th>Compression</th><th>Channels</th><th>Meas. FPS</th><th>File Size (MB)</th><th>Rate (MB/s)</th><tr>
	<tr><td>HD1080</td><td>30</td><td>Snappy</td><td>LRD</td><td>15.8025</td><td>572</td><td>33.6023</td></tr>
</table>
