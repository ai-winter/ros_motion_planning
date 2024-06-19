# Message defining the trajectory of a tracked person.
#
# The distinction between track and trajectory is that, depending on the
# implementation of the tracker, a single track (i.e. tracked person) might
# change the trajectory if at some point a new trajectory "fits" that track (person)
# better.
#

uint64                   track_id   # Unique identifier of the tracked person.
PersonTrajectoryEntry[]  trajectory # All states of the last T frames of the most likely trajectory of that tracked person.
