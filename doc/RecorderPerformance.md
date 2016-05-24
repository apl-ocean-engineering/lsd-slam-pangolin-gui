
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
	<tr><td> HD1080 </td><td> -- </td><td> 15.0 </td><td> 12.9135419944 </td><td> 355.9581 </td><td> 35.4408625489 </td><td> 2.74447262914 </td></tr>
<tr><td> HD1080 </td><td> VNC </td><td> 15.0 </td><td> 5.29516645092 </td><td> 404.2102 </td><td> 39.4900422539 </td><td> 7.45775276753 </td></tr>
<tr><td> HD720 </td><td> -- </td><td> 30.0 </td><td> 26.1614965853 </td><td> 343.8303 </td><td> 34.319401829 </td><td> 1.31182869134 </td></tr>
<tr><td> HD720 </td><td> VNC </td><td> 30.0 </td><td> 11.5067925911 </td><td> 341.7209 </td><td> 34.0148055392 </td><td> 2.95606314879 </td></tr>
<tr><td> VGA </td><td> -- </td><td> 60.0 </td><td> 36.5160894882 </td><td> 384.03105 </td><td> 37.6616414343 </td><td> 1.03137115617 </td></tr>
<tr><td> VGA </td><td> VNC </td><td> 60.0 </td><td> 23.9628071047 </td><td> 290.7481 </td><td> 27.9917261404 </td><td> 1.1681321816 </td></tr>
</table>


ZedRecorder Log Format Data Rates
----
