## NGIMU upperlimb application 

Evaluation of shoulder and elbow ISB angles using 5 NGIMUs: one placed on the torso, one the right upper arm, one in the right forearm, one in the left upper arm and one in the left forearm.
The ISB angles for the shoulder are the 'plane of elevation', 'upper arm elevation' and 'humeral rotation', while for the elbow are 'flexion-extension' and 'pronosupination'.  The YZY convention (https://pubmed.ncbi.nlm.nih.gov/15844264/) is used for the definition of rotation sequences in the shoulder. 

ISB data and rotation matrices are saved in .csv format. 


*  "ngimu_acquisitions.py" performs an initial calibration of the IMUs. The reference frame in which the calibration is performed is
 the one with z-axis upward, y-axis perpendicular to the torso and x-axis pointing from torso to right arm. 
The calibration consists in 2 phases: 15 seconds in which the subject has to stand with arm along sides and another 15 seconds in which the subject has to lift arms of 90° placing them horizontally. 

# How to perform calibration procedure: 

NPOSE: arms straight along sides with palms facing forward (check if the arm is vertical)
TPOSE: arms elevated in the frontal plane 

When calibration window is open click "start Npose/Tpose acquisition" when the subject is in the correct position. 
For each pose the acquisition lasts 15s (first 5s are not actually acquired).

The code provides the possibility to re-acquire Npose and Tpose data if the subject moves during the acquisition.
NB: Npose reacquisition is possible only if the Tpose has not been done yet. 

**NB: Before running the code, make sure to be connected to "NGIMU Network" (password: "xiotechnologies").**

# How to connect IMU to computer: 
* open NGIMU Synchronized software manager (the software is available for Windows only and can be downloaded from https://x-io.co.uk/ngimu/)  
* connect an IMU with the USB cable to the computer
* go to Tool-> Configure Wireless Settings Via USB-> access with username "admin" and password "admin"

# How to set IMUs IP addresses (router TpLink TL_WR902AC):
* go to http://tplinkwifi.net 
* access with username "admin" and password "admin"
* in LAN-> NETWORK "LAN Type" has to be set to "static IP" 
* in DHCP-> Address Reservation-> Add New and associate each MAC address of the IMUs to a different IP address between "192.168.0.100","192.168.0.101","192.168.0.102","192.168.0.103","192.168.0.104"

Current MAC-IP settings: 
04-CD-15-12-43-4C (00407CC0) IP: 192.168.0.100
04-CD-15-12-44-04 (00408402) IP:192.168.0.101
04-CD-15-12-40-94  (004088BD) IP:192.168.0.102
04-CD-15-12-41-60 (00407094) IP:192.168.0.103
04-CD-15-12-3F-B8 (004071A2) IP:192.168.0.104

# How to get the MAC address of each IMU:
* connect the router to the computer
* connect to NGIMU Network with password "xiotechnologies"
* open NGIMU GUI (the software is available for Windows only and can be downloaded from https://x-io.co.uk/ngimu/)
* connect the IMU 
* go to Settings-> Wi-Fi-> MAC Address 

# How to make the IMUs send messages: 
* open NGIMU GUI  software 
* connect an IMU
* go to Settings-> Send Rates-> set the desired frequency for Sensor, Rotation Matrix and Linear Acceleration 
* go to Send Command-> Write to device



If IPAddr is not found, probably the device name do not correspond with "wlan0". To check which is the device name run this command: 

```
ip route get 8.8.8.8
```
that will output something like: 


```
8.8.8.8 via 192.168.0.1 dev wlp2s0 src 192.168.0.104 uid 1000
cache 
```
in this case, "wlan0" need to be replaced with "wlp2s0" 



