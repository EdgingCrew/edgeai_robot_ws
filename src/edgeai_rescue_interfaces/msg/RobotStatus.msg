string IDLE=IDLE
string BUSY=BUSY
string OFFLINE=OFFLINE

string robot                   # robot namespace, e.g. robot1
string state                   # IDLE, BUSY or OFFLINE
string target_id               # current target, empty when IDLE
geometry_msgs/Pose2D pose      # current pose in the map frame
