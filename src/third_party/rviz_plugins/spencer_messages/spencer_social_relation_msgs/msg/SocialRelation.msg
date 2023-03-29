string      type        # e.g. mother-son relationship, romantic relationship, etc.
float32     strength    # relationship strength between 0.0 and 1.0

uint32      track1_id
uint32      track2_id


# Constants for type (just examples at the moment)
string      TYPE_SPATIAL  = "spatial"
string      TYPE_ROMANTIC = "romantic"
string      TYPE_PARENT_CHILD = "parent_child"
string      TYPE_FRIENDSHIP = "friendship"