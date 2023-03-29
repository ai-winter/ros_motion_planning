# Message describing an array of rectangular regions of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
#

std_msgs/Header     header

string              rgb_topic       # not necessarily the raw camera output; might be preprocessed for rectification etc.
string              depth_topic     # might not be set if depth is not available, otherwise it is the registered depth image

PersonROI[]         elements
