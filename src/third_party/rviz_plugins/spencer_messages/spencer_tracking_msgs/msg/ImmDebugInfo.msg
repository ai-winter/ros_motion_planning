# Message for passing debug information of filter performance
#

uint64      track_id        # unique identifier of the target, consistent over time
float64      innovation      # innovation of prediction and associated observation
float64      CpXX            # variance of prediction acc. to x
float64      CpYY            # variance of prediction acc. to y
float64      CXX             # variance of state acc. to x
float64      CYY             # variance of state acc. to y
float64[]    modeProbabilities# array containing mode probabilities