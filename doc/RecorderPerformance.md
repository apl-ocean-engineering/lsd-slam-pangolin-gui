
SVO Recording Data Rates
----

All resolutions recorded over a 10 sec interval, Zed SDK 0.9.3 unless otherwise noted.

On a Core i7 w/ GTX970, writing to a Samsung 850 Pro SSD on SATA.

<table>
	<tr><th>Resolution</th><th>Nom. FPS</th><th>Meas. FPS</th><th>File Size (MB)</th><th>Rate (MB/s)</th><tr>
 	<tr><td>HD2K</td><td>15</td><td>15.0802</td><td>1569</td><td>156.695</td></tr>
	<tr><td>HD1080</td><td>30</td><td>28.6462</td><td>2230</td><td>222.582</td></tr>
	<tr><td>HD720</td><td>60</td><td>59.6885</td><td>2091</td><td>209.06</td></tr>
	<tr><td>VGA</td><td>100</td><td>99.2432</td><td>1162</td><td>116.134</td></tr>
</table>

On the Jetson TX1 writing to an Intel 710 SSD on SATA (achieved ~200MB/sec with a simple dd test).  Zed SDK 0.9.4 beta, averaged over 10 runs.

<table>
	<tr><th>Resolution</th><th>Display?</th><th>Nom. FPS</th><th>Meas. FPS</th><th>File Size (MB)</th><th>Rate (MB/s)</th><th>Rate (MB/frame)</th><tr>
 	<tr><td>HD2K</td><td>n/a</td></tr>
	<tr><td> HD1080 </td><td> -- </td><td> 15.0 </td><td> 12.7063563011 </td><td> 735.6467 </td><td> 71.7925428718 </td><td> 5.65012826421 </td></tr>
	<tr><td> HD1080 </td><td> VNC </td><td> 15.0 </td><td> 11.1310442493 </td><td> 775.1976 </td><td> 75.294579298 </td><td> 6.76437696335 </td></tr>
	<tr><td> HD720 </td><td> -- </td><td> 30.0 </td><td> 26.0903853965 </td><td> 724.575 </td><td> 71.6077310556 </td><td> 2.74460227273 </td></tr>
	<tr><td> HD720 </td><td> VNC </td><td> 30.0 </td><td> 22.7924848302 </td><td> 745.3174 </td><td> 72.8145543644 </td><td> 3.19467381054 </td></tr>
	<tr><td> VGA </td><td> -- </td><td> 60.0 </td><td> 52.2519096223 </td><td> 560.81165 </td><td> 55.3993376518 </td><td> 1.06023565554 </td></tr>
	<tr><td> VGA </td><td> VNC </td><td> 60.0 </td><td> 53.5902394152 </td><td> 583.1363 </td><td> 57.6788739917 </td><td> 1.07629438907 </td></tr>
</table>


ZedRecorder Log Format Data Rates
----
