string      type        # see constants below
float32     confidence  # detection confidence

uint64[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple


# Constants for social activity type (just examples at the moment)
string      TYPE_SHOPPING = shopping
string      TYPE_STANDING = standing
string      TYPE_INDIVIDUAL_MOVING = individual_moving
string      TYPE_WAITING_IN_QUEUE = waiting_in_queue
string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen
string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk
string      TYPE_GROUP_ASSEMBLING = group_assembling
string      TYPE_GROUP_MOVING = group_moving
string      TYPE_FLOW_WITH_ROBOT = flow
string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow
string      TYPE_WAITING_FOR_OTHERS = waiting_for_others

#string      TYPE_COMMUNICATING = communicating
#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph
#string      TYPE_TALKING_ON_PHONE = talking_on_phone
