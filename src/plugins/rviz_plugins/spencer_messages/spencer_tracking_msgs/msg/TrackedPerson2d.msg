# Message defining a 2d image bbox of a tracked person
#

uint64      track_id        # unique identifier of the target, consistent over time
float32     person_height   # 3d height of person in m
int32       x               # top left corner x of 2d image bbox
int32       y               # top left corner y of 2d image bbox
uint32      w               # width of 2d image bbox
uint32      h               # height of 2d image bbox
float32     depth           # distance from the camera in m
