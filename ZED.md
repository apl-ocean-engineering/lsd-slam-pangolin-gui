
SVO Recording Data Rates
----

All resolutions recorded over a 10 sec interval on a Core i7 w/ GTX970, writing to a Samsung 850 Pro SSD on SATA.

<table>
	<tr><th>Resolution</th><th>Nom. FPS</th><th>Meas. FPS</th><th>File Size (MB)</th><th>Rate (MB/s)</th><tr>
 	<tr><td>HD2K</td><td>15</td><td>15.0802</td><td>1569</td><td>156.695</td></tr>
	<tr><td>HD1080</td><td>30</td><td>28.6462</td><td>2230</td><td>222.582</td></tr>
	<tr><td>HD720</td><td>60</td><td>59.6885</td><td>2091</td><td>209.06</td></tr>
	<tr><td>VGA</td><td>100</td><td>99.2432</td><td>1162</td><td>116.134</td></tr>
</table>

On the Jetson TX1 writing to an Intel 710 SSD on SATA (achieved ~200MB/sec with a simple dd test)

<table>
	<tr><th>Resolution</th><th>Nom. FPS</th><th>Meas. FPS</th><th>File Size (MB)</th><th>Rate (MB/s)</th><tr>
 	<tr><td>HD2K</td><td></td><td></td><td></td><td></td></tr>
	<tr><td>HD1080</td><td>15</td><td></td><td></td><td></td></tr>
	<tr><td>HD720</td><td>30</td><td></td><td></td><td></td></tr>
	<tr><td>VGA</td><td>60</td><td></td><td></td><td></td></tr>
	<tr><td></td><td>15</td><td>14.7336</td><td></td><td></td></tr>
</table>
