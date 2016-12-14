int8 MAX_EFFECT_INDEX = 29

int8 NO_CHANGE=0
int8 REACHED=1
int8 MOVED_RIGHT=2
int8 MOVED_LEFT=3
int8 MOVED_FORWARD=4
int8 MOVED_BACKWARD=5
int8 DISAPPEARED=6
int8 VANISHED=7
int8 ROTATED_CW=8
int8 ROTATED_CCW=9
int8 TOPPLED_RGT=10
int8 TOPPLED_LFT=11
int8 TOPPLED_FWD=12
int8 TOPPLED_BWD=13
int8 ROLLED_RIGHT=14
int8 ROLLED_LEFT=15
int8 ROLLED_FORWARD=16
int8 ROLLED_BACKWARD=17
int8 GRASPED=18
int8 ACQUIRED=19
int8 RELEASED=20
int8 TAKEN=21
int8 GIVEN=22
int8 GOT_ATTENTION=23
int8 TOPPLED=24
int8 GOT_SEATED=25
int8 GOT_CLOSER=26
int8 GOT_FURTHER=27
int8 STOOD_UP=28
int8 WENT_AWAY=29

#later arg can be made vector if multiple effect
#instances to be captured in one action application
int8 effect
int8 arg
float32 prob #probability of this effect to occur, this is set by "learning"
