# Effective RoboTurk motion gate and blue attack direction.
std_msgs/Header header
string command
int8 attack_sign
# Official broadcast metadata. Empty ssl_command identifies practice or missing feed.
string ssl_command
string stage
uint32 command_counter
string source
string restart_team
uint32 blue_score
uint32 yellow_score
# Designated placement position in field coordinates (metres).
string placement_team
bool has_placement_target
float64 placement_x
float64 placement_y
