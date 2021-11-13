Ethan Park
CS 482
Fall 2019
Homework Assignment 4

This directory contains the following files:
+--	Kalman-Filter
|	+--	srv
|	+--	kf.py
|	+--	kf.pyc
|	+--	kf_ros.py
|	+--	Makefile
|	+--	package.xml
|	+--	README.md
|	+--	sim.py

This directory contains a the files necessary to complete programming assignment 4 in both c and python, although only the python version is implemented. In short it should contain all the files included in provided sample directory with only kf.py being modified.

kf_update implements the equations needed for the kalman filter with sigma_x being set to np.linalg.matrix_power(G * G.T, 4) and sigma_z being set to 5*5.

In door_update the door_dist of the current door is set to P(D|S) = (P(S|D) * P(D)) / (P(S|D) * P(S|~D)). D is the probability that Hopper is at a door and S is the probability that the sensor is correct. To determine the probability of D, two arrays are made that keep track of the number of times each door index is true or false. Each door index is initially set so that their true value is 1 and their false value is 2 so that the probability of D is 50% initially. If the door sensor at time t-1 is true then increment true and if not increment false. The probability of D for each door index will incrementally increase or decrease depending on the door sensor at time t-1.

To run this program, python, numpy, and scipy must be installed:
sudo apt-get install python-numpy python-scipy python-opencv
chmod +x sim.py

Once this is done enter:
./sim.py
to run the program

No additional libraries were used that weren't already included in the sample directory.

