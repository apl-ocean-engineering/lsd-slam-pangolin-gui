
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

On the Jetson TX1 writing to an Intel 710 SSD on SATA (achieved ~200MB/sec with a simple dd test)

<table>
	<tr><th>Resolution</th><th>Nom. FPS</th><th>Display?</th><th>Meas. FPS</th><th>File Size (MB)</th><th>Rate (MB/s)</th><th>Rate (MB/frame)</th><tr>
 	<tr><td>HD2K</td><td>n/a</td></tr>
	<tr><td>HD1080</td><td>15</td><td>(0.9.4 SDK)</td><td>12.1364</td><td>316</td><td>31.1796</td><td>2.56911</td></tr>
	<tr><td>"</td><td>15</td><td>to local VNC server (0.9.4 SDK)</td><td>5.47207</td><td>427</td><td>42.4832</td><td>7.76364</td></tr>
	<tr><td>HD720</td><td>30</td><td>(0.9.4 SDK)</td><td>26.19</td><td>636</td><td>63.3339</td><td>2.41825</td></tr>
	<tr><td>"</td><td>30</td><td>to local VNC server</td><td>11.1845</td><td>386</td><td>37.3738</td><td>3.26724</td></tr>
	<tr><td>VGA</td><td>60</td><td>--</td><td>52.0121</td><td>93</td><td>9.21357</td></tr>
	<tr><td>"</td><td>60</td><td>X11 forwarding over SSH</td><td>23.626</td><td>295</td><td>27.6574</td></tr>
	<tr><td>"</td><td>60</td><td>to local VNC server</td><td>24.6756</td><td>291</td><td>28.8377</td></tr>
	<tr><td></td><td>15</td><td>--</td><td>14.7336</td><td></td><td></td></tr>
</table>


ZedRecorder Log Format Data Rates
----
