Header header
uint64 id
uint16 type
string social_state
geometry_msgs/Pose pose
geometry_msgs/Twist twist
pedsim_msgs/AgentForce forces

# Use sensors package to control observability

# Social State string constants
string      TYPE_STANDING = "standing"
string      TYPE_INDIVIDUAL_MOVING = "individual_moving"
string      TYPE_WAITING_IN_QUEUE = "waiting_in_queue"
string      TYPE_GROUP_MOVING = "group_moving"


# Agent types
# 0, 1 -> ordinary agents
# 2 -> Robot
# 3 -> standing/elderly agents
