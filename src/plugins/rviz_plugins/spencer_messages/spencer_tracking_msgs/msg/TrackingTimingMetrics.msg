# Message with timing values for analyzing the efficency 
#

Header              header                  # Header containing timestamp etc. of this message
uint64              track_count             # number of person currentl tracked
uint64              cycle_no                # incremented number of cycles        
float32             average_cycle_time      # average time for one cycle of tracker
float32             cycle_time              # cycle time of current cycle
float32             average_processing_rate # average frequency of processing data
float32             cpu_load                # current cpu load
float32             average_cpu_load        # average cpu load
float32             elapsed_time            # elapsed seconds since start
float32             elapsed_cpu_time        # elapsed cpu time since start