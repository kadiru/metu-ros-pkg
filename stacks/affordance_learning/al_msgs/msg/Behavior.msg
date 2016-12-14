int8 MAX_BEHAVIOR_INDEX = 36

int8 TUCK_ARMS = 0
int8 REACH = 1
int8 COVER = 2
int8 CALL = 3
int8 TAKE = 4
int8 GIVE = 5
int8 GRASP_TOP = 6
int8 GRASP_SIDE = 7
int8 PUSH_RGT = 8
int8 PUSH_LFT = 9
int8 PUSH_FWD = 10
int8 PUSH_BWD = 11
int8 PUSH_TOP_RGT = 12
int8 PUSH_TOP_LFT = 13
int8 PUSH_TOP_FWD = 14
int8 PUSH_TOP_BWD = 15
int8 POINT=16
int8 HIDE=17
int8 CANCEL=18
int8 STOP=19
int8 SHOW=20
int8 HOME=21
int8 LOOK_AT_REGION=22
int8 LOOK_AT_POINT=23
int8 LOOK_AT_FACE=24
int8 LIFT=25
int8 RELEASE=26
int8 CONTINUE=27
int8 SAY_PASS_ME=28
int8 HELLO=29
int8 COME=30
int8 SIT_DOWN=31
int8 STAND_UP=32
int8 BYE=33
int8 PUSH_YOUR_RIGHT=34
int8 PUSH_YOUR_LEFT=35
int8 TAKE_GET=36

Header header
int8 behavior
int8 arg
string entity
bool is_preemptive
bool is_interactive
float32 prob #probability of this behavior to result in a specific effect, this is set by "learning"
