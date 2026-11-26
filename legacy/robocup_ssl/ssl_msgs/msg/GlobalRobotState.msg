Header header
float32 confidence 		#confidence of the identification of the robot
uint8 id 			#robot id
string team 			#robot's team 0->Blue 1->Yellow (this might be changed later on)
uint8 state 			#0->out of game/no detection, 1->in game, 2->in game but no move
geometry_msgs/Pose2D pose 	#robot's pose x,y,theta
geometry_msgs/Point32 pix_coord 	#robot's 2D position in the image plane
float32 height 			#robot's actual height
