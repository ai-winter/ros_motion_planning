/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include "rviz/selection/selection_manager.h"

#ifndef Q_MOC_RUN
#include "social_activities_display.h"

#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/foreach.hpp>
#endif
#define foreach BOOST_FOREACH

// required to fix orientation of any Cylinder shapes
const Ogre::Quaternion shapeQuaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) );


namespace sr = spencer_social_relation_msgs;
namespace spencer_tracking_rviz_plugin
{

void SocialActivitiesDisplay::onInitialize()
{
    m_trackedPersonsCache.initialize(this, context_, update_nh_);
    PersonDisplayCommon::onInitialize();

    QObject::connect(m_commonProperties->style, SIGNAL(changed()), this, SLOT(personVisualTypeChanged()) );

    m_excluded_activity_types_property = new rviz::StringProperty( "Excluded activity types", "", "Comma-separated list of activity types whose visualization should be hidden", this, SLOT(stylesChanged()) );
    m_included_activity_types_property = new rviz::StringProperty( "Included activity types", "", "Comma-separated list of activity types whose visualization should be visible, overrides excluded", this, SLOT(stylesChanged()) );

    m_min_confidence_property = new rviz::FloatProperty( "Min. confidence", 0.0, "Minimum confidence for a social activity to be shown", this, SLOT(stylesChanged()) );
    m_min_confidence_property->setMin( 0.0 );

    m_hide_with_no_activity_property = new rviz::BoolProperty( "Hide tracks with no activity", false, "Hide all tracks which do not have at least one social activity assigned", this, SLOT(stylesChanged()));

    m_render_intraactivity_connections_property = new rviz::BoolProperty( "Connect tracks sharing the same activity", true, "Connect all tracks that share the same activity", this, SLOT(stylesChanged()));
    m_line_width_property = new rviz::FloatProperty( "Line width", 0.05, "Line width of connecting lines", m_render_intraactivity_connections_property, SLOT(stylesChanged()), this );
    m_line_width_property->setMin( 0.0 );

    m_render_activity_types_property = new rviz::BoolProperty( "Render activity type texts", true, "Render activity types as text", this, SLOT(stylesChanged()));
    m_activity_type_per_track_property = new rviz::BoolProperty( "Activity type per track", false, "Show activity type for each individual track", this, SLOT(stylesChanged()));
    m_render_confidences_property = new rviz::BoolProperty( "Render confidences", true, "Render confidence values next to activity type", this, SLOT(stylesChanged()));

    m_render_circles_property = new rviz::BoolProperty( "Render circles below person", true, "Render circles below person", this, SLOT(stylesChanged()));
    m_circle_radius_property = new rviz::FloatProperty( "Radius", 0.45, "Radius of circles below person in meters", m_render_circles_property, SLOT(stylesChanged()), this );
    m_circle_radius_property->setMin( 0.0 );

    m_circle_alpha_property = new rviz::FloatProperty( "Alpha", 1.0, "Alpha value (opacity) of circles below person", m_render_circles_property, SLOT(stylesChanged()), this );
    m_circle_alpha_property->setMin( 0.0 );
    m_circle_alpha_property->setMax( 1.0 );

    m_occlusion_alpha_property = new rviz::FloatProperty( "Occlusion alpha", 0.5, "Alpha multiplier for history of occluded tracks", this, SLOT(stylesChanged()) );
    m_occlusion_alpha_property->setMin( 0.0 );

    m_activity_type_offset = new rviz::FloatProperty( "Activity type Z offset", 2.0, "Offset in z position (height) of the activity type text", this, SLOT(stylesChanged()) );

    m_activity_colors = new rviz::Property( "Activity colors", "", "Colors of different social activity types", this );

    // Add colors for new activity types here, also adjust header file!
    m_activity_color_unknown = new rviz::ColorProperty( "(Unknown activity)", QColor(255,255,255), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_none = new rviz::ColorProperty( "(No activity)", QColor(200,200,200), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_shopping = new rviz::ColorProperty( sr::SocialActivity::TYPE_SHOPPING.c_str(), QColor(0,0,255), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_standing = new rviz::ColorProperty( sr::SocialActivity::TYPE_STANDING.c_str(), QColor(0,0,0), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_individual_moving = new rviz::ColorProperty( sr::SocialActivity::TYPE_INDIVIDUAL_MOVING.c_str(), QColor(128,128,128), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_waiting_in_queue = new rviz::ColorProperty( sr::SocialActivity::TYPE_WAITING_IN_QUEUE.c_str(), QColor(255,0,255), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_looking_at_information_screen = new rviz::ColorProperty( sr::SocialActivity::TYPE_LOOKING_AT_INFORMATION_SCREEN.c_str(), QColor(255,255,0), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_looking_at_kiosk = new rviz::ColorProperty( sr::SocialActivity::TYPE_LOOKING_AT_KIOSK.c_str(), QColor(255,128,0), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_group_assembling = new rviz::ColorProperty( sr::SocialActivity::TYPE_GROUP_ASSEMBLING.c_str(), QColor(0,128,255), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_group_moving = new rviz::ColorProperty( sr::SocialActivity::TYPE_GROUP_MOVING.c_str(), QColor(0,255,255), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_flow = new rviz::ColorProperty( sr::SocialActivity::TYPE_FLOW_WITH_ROBOT.c_str(), QColor(0,255,0), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_antiflow = new rviz::ColorProperty( sr::SocialActivity::TYPE_ANTIFLOW_AGAINST_ROBOT.c_str(), QColor(255,0,0), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_waiting_for_others = new rviz::ColorProperty( sr::SocialActivity::TYPE_WAITING_FOR_OTHERS.c_str(), QColor(255,170,255), "", m_activity_colors, SLOT(stylesChanged()), this);
    m_activity_color_looking_for_help = new rviz::ColorProperty( sr::SocialActivity::TYPE_LOOKING_FOR_HELP.c_str(), QColor(155,170,255), "", m_activity_colors, SLOT(stylesChanged()), this);

    m_commonProperties->color_transform->setHidden(true);
    m_commonProperties->color_map_offset->setHidden(true);

    // Create a scene node for visualizing group affiliation history
    m_socialActivitiesSceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
}

SocialActivitiesDisplay::~SocialActivitiesDisplay()
{
}

// Clear the visuals by deleting their objects.
void SocialActivitiesDisplay::reset()
{
    PersonDisplayCommon::reset();
    m_trackedPersonsCache.reset();
    m_socialActivityVisuals.clear();
    m_personVisualMap.clear();
    m_highestConfidenceActivityPerTrack.clear();
}

void SocialActivitiesDisplay::update(float wall_dt, float ros_dt)
{
    // Update animations
    foreach(PersonVisualContainer& personVisualContainer, m_personVisualMap | boost::adaptors::map_values) {
        if(personVisualContainer.personVisual) {
            personVisualContainer.personVisual->update(ros_dt);
        }
    }
}

void SocialActivitiesDisplay::stylesChanged()
{
    m_commonProperties->color_map_offset->setHidden(true);

    // Get list of group IDs belonging to tracks that shall be hidden or visible
    m_excludedActivityTypes.clear();
    {
        string inputString = m_excluded_activity_types_property->getStdString();
        char_separator<char> separator(",");
        tokenizer< char_separator<char> > tokens(inputString, separator);
        foreach(const string& token, tokens) {
            string tmp = token;
            boost::algorithm::to_lower(tmp);
            m_excludedActivityTypes.insert(tmp);
        }
    }
    m_includedActivityTypes.clear();
    {
        string inputString = m_included_activity_types_property->getStdString();
        char_separator<char> separator(",");
        tokenizer< char_separator<char> > tokens(inputString, separator);
        foreach(const string& token, tokens) {
            string tmp = token;
            boost::algorithm::to_lower(tmp);
            m_includedActivityTypes.insert(tmp);
        }
    }

    foreach(boost::shared_ptr<SocialActivityVisual> socialActivityVisual, m_socialActivityVisuals) {
        updateSocialActivityVisualStyles(socialActivityVisual);
    }

    foreach(PersonVisualContainer& personVisualContainer, m_personVisualMap | boost::adaptors::map_values) {
        if(personVisualContainer.personVisual) {
            // Update common styles to person visual, such as line width
            applyCommonStyles(personVisualContainer.personVisual);

            // Update color according to highest-ranking social activity for this person
            Ogre::ColourValue activityColor;

            activity_type activityType = "";
            float confidence = 1.0f;
            if(m_highestConfidenceActivityPerTrack.find(personVisualContainer.trackId) != m_highestConfidenceActivityPerTrack.end()) {
                activityType = m_highestConfidenceActivityPerTrack[personVisualContainer.trackId].type;
                confidence   = m_highestConfidenceActivityPerTrack[personVisualContainer.trackId].confidence;
            }
            else {
                if(m_hide_with_no_activity_property->getBool()) confidence = -999;
            }
            activityColor = getActivityColor(activityType, confidence);

            personVisualContainer.personVisual->setColor(activityColor);
        }
    }
}

bool SocialActivitiesDisplay::isActivityTypeHidden(activity_type activityType) {
    boost::algorithm::to_lower(activityType);

    bool isIncluded = m_includedActivityTypes.find(activityType) != m_includedActivityTypes.end();
    if(isIncluded) return false;
    if(!m_includedActivityTypes.empty()) return true;

    return m_excludedActivityTypes.find(activityType) != m_excludedActivityTypes.end();
}

Ogre::ColourValue SocialActivitiesDisplay::getActivityColor(activity_type activityType, float confidence) {
    bool hideActivityType = isActivityTypeHidden(activityType);

    // Determine color
    rviz::ColorProperty* colorProperty = NULL;

    // Add new social activity types here, and also add a property in constructor at top of file!
    if(activityType.empty())
        colorProperty = m_activity_color_none;
    else if(activityType == sr::SocialActivity::TYPE_SHOPPING)
        colorProperty = m_activity_color_shopping;
    else if(activityType == sr::SocialActivity::TYPE_STANDING)
        colorProperty = m_activity_color_standing;
    else if(activityType == sr::SocialActivity::TYPE_INDIVIDUAL_MOVING)
        colorProperty = m_activity_color_individual_moving;
    else if(activityType == sr::SocialActivity::TYPE_WAITING_IN_QUEUE)
        colorProperty = m_activity_color_waiting_in_queue;
    else if(activityType == sr::SocialActivity::TYPE_LOOKING_AT_INFORMATION_SCREEN)
        colorProperty = m_activity_color_looking_at_information_screen;
    else if(activityType == sr::SocialActivity::TYPE_LOOKING_AT_KIOSK)
        colorProperty = m_activity_color_looking_at_kiosk;
    else if(activityType == sr::SocialActivity::TYPE_GROUP_ASSEMBLING)
        colorProperty = m_activity_color_group_assembling;
    else if(activityType == sr::SocialActivity::TYPE_GROUP_MOVING)
        colorProperty = m_activity_color_group_moving;
    else if(activityType == sr::SocialActivity::TYPE_FLOW_WITH_ROBOT)
        colorProperty = m_activity_color_flow;
    else if(activityType == sr::SocialActivity::TYPE_ANTIFLOW_AGAINST_ROBOT)
        colorProperty = m_activity_color_antiflow;
    else if(activityType == sr::SocialActivity::TYPE_WAITING_FOR_OTHERS)
        colorProperty = m_activity_color_waiting_for_others;
    else if(activityType == sr::SocialActivity::TYPE_LOOKING_FOR_HELP)
        colorProperty = m_activity_color_looking_for_help;
    else
        colorProperty = m_activity_color_unknown;

    Ogre::ColourValue activityColor = colorProperty->getOgreColor();
    activityColor.a = 1.0f;

    activityColor.a *= m_commonProperties->alpha->getFloat(); // general alpha
    if(hideActivityType) activityColor.a = 0;

    if(confidence < m_min_confidence_property->getFloat()) activityColor.a = 0;

    return activityColor;
}

// Set the rendering style (cylinders, meshes, ...) of tracked persons
void SocialActivitiesDisplay::personVisualTypeChanged()
{
    m_personVisualMap.clear();
    foreach(PersonVisualContainer& personVisualContainer, m_personVisualMap | boost::adaptors::map_values) {
        personVisualContainer.personVisual.reset();
        createPersonVisualIfRequired(personVisualContainer.sceneNode.get(), personVisualContainer.personVisual);
    }
    stylesChanged();
}

void SocialActivitiesDisplay::updateSocialActivityVisualStyles(boost::shared_ptr<SocialActivityVisual>& socialActivityVisual)
{
    std::stringstream ss;

    Ogre::ColourValue activityColor = getActivityColor(socialActivityVisual->activityType, socialActivityVisual->confidence);
    bool hideActivity = isActivityTypeHidden(socialActivityVisual->activityType) || socialActivityVisual->confidence < m_min_confidence_property->getFloat();
    bool showCircles = m_render_circles_property->getBool();

    foreach(boost::shared_ptr<rviz::Shape> circle, socialActivityVisual->socialActivityAssignmentCircles) {
        circle->setColor(activityColor.r, activityColor.g, activityColor.b, activityColor.a * m_circle_alpha_property->getFloat() * (showCircles ? 1.0f : 0.0f));
        const double circleDiameter = m_circle_radius_property->getFloat() * 2, circleHeight = 0;
        circle->setScale(shapeQuaternion * Ogre::Vector3(circleDiameter, circleDiameter, circleHeight));
    }

    double connectionLineVisibilityAlpha = m_render_intraactivity_connections_property->getBool() ? 1.0 : 0.0;
    foreach(boost::shared_ptr<rviz::BillboardLine> connectionLine, socialActivityVisual->connectionLines) {
        connectionLine->setColor(activityColor.r, activityColor.g, activityColor.b, activityColor.a * connectionLineVisibilityAlpha);
        connectionLine->setLineWidth(m_line_width_property->getFloat());
    }

    // Update text colors, size and visibility
    ss.str(""); ss << socialActivityVisual->activityType;
    if(m_render_confidences_property->getBool()) ss << fixed << setprecision(1) << " (" << 100*socialActivityVisual->confidence << "%)";

    for(int i = 0; i < socialActivityVisual->typeTexts.size(); i++) {
        boost::shared_ptr<TextNode>& typeText = socialActivityVisual->typeTexts[i];

        if(typeText) { // might be not set if center of activity could not be determined
            typeText->setCaption(ss.str());

            Ogre::Vector3 centerAt;
            if(m_activity_type_per_track_property->getBool()) {
                boost::shared_ptr<CachedTrackedPerson> trackedPerson = m_trackedPersonsCache.lookup(socialActivityVisual->trackIds[i]);
                if(!trackedPerson) continue;
                centerAt = Ogre::Vector3(trackedPerson->center.x, trackedPerson->center.y, m_commonProperties->z_offset->getFloat());
            }
            else centerAt = Ogre::Vector3(socialActivityVisual->socialActivityCenter.x, socialActivityVisual->socialActivityCenter.y, socialActivityVisual->socialActivityCenter.z);

            Ogre::ColourValue fontColor = m_commonProperties->font_color_style->getOptionInt() == FONT_COLOR_CONSTANT ? m_commonProperties->constant_font_color->getOgreColor() : activityColor;
            fontColor.a = m_commonProperties->alpha->getFloat();
            if(hideActivity) fontColor.a = 0;
            float characterHeight = 0.23 * m_commonProperties->font_scale->getFloat();
            typeText->setVisible(m_render_activity_types_property->getBool());
            typeText->setCharacterHeight(characterHeight);
            typeText->setColor(fontColor);
            typeText->setPosition(m_frameTransform * Ogre::Vector3(
                centerAt.x,
                centerAt.y,
                centerAt.z + m_activity_type_offset->getFloat() + m_commonProperties->z_offset->getFloat()
                    + socialActivityVisual->declutteringOffset * characterHeight /* this is for decluttering of overlapping labels */));
        }
    }
}

// Helper function for guaranteeing consistent ordering of activity labels
bool CompareActivityByType (const SocialActivitiesDisplay::ActivityWithConfidence& first, const SocialActivitiesDisplay::ActivityWithConfidence& second) {
    return first.type < second.type;
}

// This is our callback to handle an incoming group message.
void SocialActivitiesDisplay::processMessage(const spencer_social_relation_msgs::SocialActivities::ConstPtr& msg)
{
    // Get transforms into fixed frame etc.
    if(!preprocessMessage(msg)) return;

    // Transform into Rviz fixed frame
    m_frameTransform = Ogre::Matrix4(m_frameOrientation);
    m_frameTransform.setTrans(m_framePosition);
    stringstream ss;

    // Clear previous visualization (not very efficient, but easier to implement)
    // Note that person visuals are not cleared to allow walking animations to function properly
    m_socialActivityVisuals.clear();
    m_socialActivitiesSceneNode->removeAndDestroyAllChildren();

    m_highestConfidenceActivityPerTrack.clear(); // used later on to determine color of person visuals
    m_allActivitiesPerTrack.clear();

    unsigned int numTracksWithUnknownPosition = 0;


    //
    // Iterate over all social activities in this message
    //
    foreach (const spencer_social_relation_msgs::SocialActivity& socialActivity, msg->elements)
    {
        // Create a new visual representation of the social activity
        boost::shared_ptr<SocialActivityVisual> socialActivityVisual = boost::shared_ptr<SocialActivityVisual>(new SocialActivityVisual);
        socialActivityVisual->activityType = socialActivity.type;
        socialActivityVisual->confidence = socialActivity.confidence;
        socialActivityVisual->personCount = socialActivity.track_ids.size();
        m_socialActivityVisuals.push_back(socialActivityVisual);

        geometry_msgs::Point socialActivityCenter;
        size_t numGoodTracksInActivity = 0;

        //
        // Assignment circles, person visuals (if enabled) + connections between social activity members
        //

        for(size_t trackIndex = 0; trackIndex < socialActivity.track_ids.size(); trackIndex++)
        {
            const track_id trackId = socialActivity.track_ids[trackIndex];
            boost::shared_ptr<CachedTrackedPerson> trackedPerson = m_trackedPersonsCache.lookup(trackId);

            ActivityWithConfidence activityWithConfidence;
            activityWithConfidence.type = socialActivity.type;
            activityWithConfidence.confidence = socialActivity.confidence;

            // Update map of highest-confidence activity per person
            if(m_highestConfidenceActivityPerTrack.find(trackId) == m_highestConfidenceActivityPerTrack.end()
                    || socialActivity.confidence > m_highestConfidenceActivityPerTrack[trackId].confidence) {
                m_highestConfidenceActivityPerTrack[trackId] = activityWithConfidence;
            }

            // Update map of all activities per person
            if(m_allActivitiesPerTrack.find(trackId) == m_allActivitiesPerTrack.end()) {
                m_allActivitiesPerTrack[trackId] = vector<ActivityWithConfidence>();
            }
            m_allActivitiesPerTrack[trackId].push_back(activityWithConfidence);


            // Get current track position
            if(!trackedPerson) {
                numTracksWithUnknownPosition++;
            }
            else
            {
                socialActivityVisual->trackIds.push_back(trackId);
                numGoodTracksInActivity++;

                Ogre::Vector3 trackCenterAtGroundPlane(trackedPerson->center.x, trackedPerson->center.y, m_commonProperties->z_offset->getFloat());
                socialActivityCenter.x += trackCenterAtGroundPlane.x;
                socialActivityCenter.y += trackCenterAtGroundPlane.y;
                socialActivityCenter.z += trackCenterAtGroundPlane.z;


                //
                // Social activity assignment circles (below tracks)
                //

                if(m_render_circles_property->getBool()) // only create circles if they are enabled, for better performance
                {
                    boost::shared_ptr<rviz::Shape> circle = boost::shared_ptr<rviz::Shape>(new rviz::Shape(rviz::Shape::Cylinder, context_->getSceneManager(), m_socialActivitiesSceneNode.get()));

                    const double circleHeight = 0;
                    circle->setOrientation(shapeQuaternion);

                    Ogre::Vector3 circlePos = trackCenterAtGroundPlane + Ogre::Vector3(0, 0, -0.5*circleHeight - 0.01);
                    circle->setPosition(circlePos);

                    socialActivityVisual->socialActivityAssignmentCircles.push_back(circle);
                }

                //
                // Intra-activity connections
                //

                if(m_render_intraactivity_connections_property->getBool()) // only create circles if they are enabled, for better performance
                {
                    // Iterate over all tracks sharing the same activity to render intra-activity connections
                    for(size_t otherTrackIndex = trackIndex + 1; otherTrackIndex < socialActivity.track_ids.size(); otherTrackIndex++)
                    {
                        const track_id otherTrackId = socialActivity.track_ids[otherTrackIndex];
                        boost::shared_ptr<CachedTrackedPerson> otherTrackedPerson = m_trackedPersonsCache.lookup(otherTrackId);

                        // Get other track's position
                        if(otherTrackedPerson) {
                            // Get positions. These are already in fixed frame coordinates!
                            const Ogre::Vector3 verticalShift(0,0, 0.5 + m_commonProperties->z_offset->getFloat());
                            const Ogre::Vector3& position1 = verticalShift + trackedPerson->center;
                            const Ogre::Vector3& position2 = verticalShift + otherTrackedPerson->center;

                            // Add line connecting the two tracks
                            boost::shared_ptr<rviz::BillboardLine> connectionLine(new rviz::BillboardLine(context_->getSceneManager(), m_socialActivitiesSceneNode.get()));
                            connectionLine->setMaxPointsPerLine(2);
                            connectionLine->addPoint(position1);
                            connectionLine->addPoint(position2);
                            socialActivityVisual->connectionLines.push_back(connectionLine);
                        }
                    } // end loop over other tracks sharing same activity
                }

            } // end if track found
        } // end for loop over tracks

        //
        // Texts
        //
        socialActivityCenter.x /= (double) numGoodTracksInActivity;
        socialActivityCenter.y /= (double) numGoodTracksInActivity;
        socialActivityCenter.z /= (double) numGoodTracksInActivity;
        socialActivityVisual->socialActivityCenter = socialActivityCenter;

        // Social activity type
        if(numGoodTracksInActivity > 0) {
            for(int i = 0; i < (m_activity_type_per_track_property->getBool() ? socialActivityVisual->trackIds.size() : 1); i++) {
                boost::shared_ptr<TextNode> typeText(new TextNode(context_->getSceneManager(), m_socialActivitiesSceneNode.get()));
                typeText->showOnTop();
                socialActivityVisual->typeTexts.push_back(typeText);
            }
        }
    } // end for loop over all social activities in msg


    //
    // Second iteration over all social activities and member tracks, required for decluttering
    //
    ROS_ASSERT(msg->elements.size() == m_socialActivityVisuals.size());
    for(size_t i = 0; i < msg->elements.size(); i++)
    {
        const spencer_social_relation_msgs::SocialActivity& socialActivity = msg->elements[i];
        boost::shared_ptr<SocialActivityVisual> socialActivityVisual = m_socialActivityVisuals[i];
        size_t maxIndexOfThisActivity = 0;

        for(size_t trackIndex = 0; trackIndex < socialActivity.track_ids.size(); trackIndex++)
        {
            const track_id trackId = socialActivity.track_ids[trackIndex];
            vector<ActivityWithConfidence> activitiesOfTrack(m_allActivitiesPerTrack[trackId]);

            // Sort to ensure consistency across multiple runs, even if msg->elements order changes
            std::sort(activitiesOfTrack.begin(), activitiesOfTrack.end(), CompareActivityByType);

            for(size_t j = 0; j < activitiesOfTrack.size(); j++) {
                if(activitiesOfTrack[j].type == socialActivityVisual->activityType) {
                    maxIndexOfThisActivity = std::max(maxIndexOfThisActivity, j);
                    break;
                }
            }
        }

        socialActivityVisual->declutteringOffset = maxIndexOfThisActivity; // got it
    }


    //
    // Create person visuals for all tracked persons (colored in color of activity with highest confidence)
    //
    set<track_id> seenTrackIds;
    foreach(const TrackedPersonsCache::CachedTrackedPersonsMap::value_type& entry, m_trackedPersonsCache.getMap()) {
        const track_id trackId = entry.first;
        const boost::shared_ptr<CachedTrackedPerson> trackedPerson = entry.second;

        PersonVisualContainer personVisualContainer;
        if(m_personVisualMap.find(trackId) != m_personVisualMap.end()) {
            personVisualContainer = m_personVisualMap[trackId];
        }
        else {
            personVisualContainer.trackId = trackId;
            personVisualContainer.sceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode()); // This scene node is the parent of all visualization elements for the tracked person
        }

        // Create new visual for the person itself, if needed
        createPersonVisualIfRequired(personVisualContainer.sceneNode.get(), personVisualContainer.personVisual);

        const double personHeight = personVisualContainer.personVisual ? personVisualContainer.personVisual->getHeight() : 0;
        const Ogre::Matrix3 covXYZinTargetFrame = covarianceXYZIntoTargetFrame(trackedPerson->pose);
        setPoseOrientation(personVisualContainer.sceneNode.get(), trackedPerson->pose, covXYZinTargetFrame, personHeight);

        // Update walking animation if required
        const Ogre::Vector3 velocityVector = getVelocityVector(trackedPerson->twist);
        boost::shared_ptr<MeshPersonVisual> meshPersonVisual = boost::dynamic_pointer_cast<MeshPersonVisual>(personVisualContainer.personVisual);
        if(meshPersonVisual) {
            meshPersonVisual->setWalkingSpeed(velocityVector.length());
        }

        m_personVisualMap[trackId] = personVisualContainer; // to keep visuals alive across multiple frames, for walking animation
        seenTrackIds.insert(trackId);
    }

    // Delete obsolete track visuals of tracks that have disappeared
    set<track_id> trackIdsToDelete;
    foreach(track_id trackId, m_personVisualMap | boost::adaptors::map_keys) {
        if(seenTrackIds.find(trackId) == seenTrackIds.end()) trackIdsToDelete.insert(trackId);
    }
    foreach(track_id trackIdToDelete, trackIdsToDelete) {
        m_personVisualMap.erase(trackIdToDelete);
    }

    //
    // Update all styles (colors etc. which can also be reconfigured at runtime, even if no new messages are received)
    //
    stylesChanged();


    //
    // Update status (shown in property pane)
    //

    ss.str("");
    ss << msg->elements.size() << " activities(s)";
    setStatusStd(rviz::StatusProperty::Ok, "Social activities", ss.str());

    ss.str("");
    ss << numTracksWithUnknownPosition << " track(s) with unknown position";
    setStatusStd(0 == numTracksWithUnknownPosition ? rviz::StatusProperty::Ok : rviz::StatusProperty::Warn, "Track-to-activity assignment", ss.str());
}

} // end namespace spencer_tracking_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spencer_tracking_rviz_plugin::SocialActivitiesDisplay, rviz::Display)
