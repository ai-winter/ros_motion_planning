uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name
string                  type                    # type of the attribute, see constants below

float32[]               values                  # values, each value also should have a confidence; highest-confidence value comes first!
float32[]               confidences             # corresponding confidences should sum up to 1.0


###########################################
### Constants for scalar attribute types. #
###########################################

string      PERSON_HEIGHT        = person_height
