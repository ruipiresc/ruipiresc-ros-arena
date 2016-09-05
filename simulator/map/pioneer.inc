define hokuyo ranger(
	block(
		points 4
			point[0] [ -0.02 -0.02 ]
			point[1] [ 0.02 -0.02 ]
			point[2] [ 0.02 0.02 ]
			point[3] [ -0.02 0.02 ]
		z [ 0.0 0.02]
		color "gray40"
	)

	block(
		points 4
			point[0] [ -0.025 -0.025 ]
			point[1] [ 0.025 -0.025 ]
			point[2] [ 0.025 0.025 ]
			point[3] [ -0.025 0.025 ]
		z [ -0.05 0.0]
		color "gray40"
	)

	sensor( 			
		range [ 0.02  4.0 ]
		fov 240.0
		samples 682
	)

	# generic model properties
	size [ 0.05 0.05 0.02 ]
)

define pioneer position(
	size [0.26 0.26 0.215]

	# this set of blocks approximates the shape of a real Pioneer

	#  The geometry is from the Webots v5.3.0 manual. Thanks to Webots
	#  and Olivier Michel. If Stage or Gazebo do not do what you want,
	#  take a look at Webots. It's a very nice commercial simulator.
 
	block(			# lid
		points 22
		point[21] [  0.174 0 ]
		point[20] [  0.166 -0.056 ]
		point[19] [  0.145 -0.107 ]
		point[18] [  0.112 -0.155 ]
		point[17] [  0.064 -0.190 ]
		point[16] [  -0.074 -0.190 ]
		point[15] [  -0.096 -0.160 ]
		point[14] [  -0.151 -0.160 ]
		point[13] [  -0.2   -0.155 ]
		point[12] [  -0.236 -0.107 ]
		point[11] [  -0.256 -0.056 ]
		point[10] [  -0.264  0     ]
		point[9] [  -0.256  0.056 ]
		point[8] [ -0.236  0.107 ]
		point[7] [ -0.2    0.155 ]
		point[6] [ -0.151  0.160 ]
		point[5] [ -0.096  0.160 ]
		point[4] [ -0.074  0.190 ]
		point[3] [  0.064  0.190 ]
		point[2] [  0.112  0.155 ]
		point[1] [  0.145  0.107 ]
		point[0] [  0.166  0.056 ]
		z [ 0.234 0.24 ]

		# a dark top looks more realistic, but isn't very useful 
		# for a top-down view
		# color "gray50"
	)

	block(			# main body
		points 8
		point[0] [ -0.185 -0.135 ] 
		point[1] [  0.095 -0.135 ] 
		point[2] [  0.11  -0.08  ] 
		point[3] [  0.11   0.08  ] 
		point[4] [  0.095  0.135 ] 
		point[5] [ -0.185  0.135 ]
		point[6] [ -0.215  0.1   ]
		point[7] [ -0.215 -0.1   ] 
		z [ 0.059 0.234 ]
		color "red"
	)

	block(			# sonar case
		points 9
		point[0]  [ -0.135  0.136 ]
		point[1]  [ -0.185  0.136 ]
		point[2]  [ -0.223  0.101 ]
		point[3]  [ -0.248  0.054 ]
		point[4]  [ -0.258  0     ]
		point[5]  [ -0.248 -0.054 ]
		point[6]  [ -0.223 -0.101 ]
		point[7]  [ -0.185 -0.136 ]
		point[8]  [ -0.135 -0.136 ]
		z [ 0.184 0.234 ]
	)

	block(			# sonar case
		points 9
		point[0]  [ 0.046 -0.136 ]
		point[1]  [ 0.096 -0.136 ]
		point[2]  [ 0.134 -0.101 ]
		point[3]  [ 0.159 -0.054 ]
		point[4]  [ 0.168  0     ]
		point[5]  [ 0.159  0.054 ]
		point[6]  [ 0.134  0.101 ]
		point[7]  [ 0.096  0.136 ]
		point[8]  [ 0.046  0.136 ]
		z [ 0.184 0.234 ]
	)
	
	block( 			# left wheel
		points 4
		point[0] [  0.083  0.177 ]
		point[1] [ -0.083  0.177 ]
		point[2] [ -0.083  0.140 ]
		point[3] [  0.083  0.140 ]
		z [0 0.165 ]
		color "gray15"
	)

	block(			# right wheel
		points 4
		point[0] [  0.083  -0.14 ]
		point[1] [ -0.083  -0.14 ]
		point[2] [ -0.083  -0.177 ]
		point[3] [  0.083  -0.177 ]
		z [ 0 0.165 ]
		color "gray15"
	)

	block( 			 # castor
		points 4
		point[3] [ -0.2475  0.012 ]
		point[2] [ -0.1825  0.012 ]
		point[1] [ -0.1825 -0.012 ]
		point[0] [ -0.2475 -0.012 ]
		z [ 0 0.065 ]
		color "gray15"
	)

	gui_nose 1
	drive "diff" #[0.26 0.26 0.215]
	hokuyo( pose [0.15 0.0 -0.12 0] )
	color "gray50"
)