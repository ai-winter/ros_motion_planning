uint64                  subject_id              # either an observation ID or a track ID (if information has been integrated over time); should be encoded in topic name
string                  type                    # type of the attribute, see constants below

string[]                values                  # values, each value also should have a confidence, highest-confidence attribute should come first
float32[]               confidences             # corresponding confidences should sum up to 1.0, highest confidence comes first


##################################################
### Constants for categorical attribute types. ###
##################################################

string      GENDER        = gender
string      AGE_GROUP     = age_group

###################################################
### Constants for categorical attribute values. ###
###################################################

string      GENDER_MALE             = male
string      GENDER_FEMALE           = female

# Age groups are based upon the categories from the "Images Of Groups" RGB database
string      AGE_GROUP_0_TO_2        = 0-2
string      AGE_GROUP_3_TO_7        = 3-7
string      AGE_GROUP_8_TO_12       = 8-12
string      AGE_GROUP_13_TO_19      = 13-19
string      AGE_GROUP_20_TO_36      = 20-36
string      AGE_GROUP_37_TO_65      = 37-65
string      AGE_GROUP_66_TO_99      = 66-99



