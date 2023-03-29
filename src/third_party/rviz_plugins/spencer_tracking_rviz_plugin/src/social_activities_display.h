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

#ifndef SOCIAL_ACTIVITIES_DISPLAY_H
#define SOCIAL_ACTIVITIES_DISPLAY_H
#ifndef Q_MOC_RUN
#include <map>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <spencer_social_relation_msgs/SocialActivities.h>
#endif
#include "person_display_common.h"
#include "tracked_persons_cache.h"

namespace spencer_tracking_rviz_plugin
{
    typedef std::string activity_type;

    /// The display which can be added in RViz to display social activities.
    class SocialActivitiesDisplay: public PersonDisplayCommon<spencer_social_relation_msgs::SocialActivities>
    {
    Q_OBJECT
    public:
        // To determine, for persons involved in multiple social activities, their most likely one.
        struct ActivityWithConfidence {
            activity_type type;
            float confidence;
        };

        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        SocialActivitiesDisplay() {};
        virtual ~SocialActivitiesDisplay();

        // Overrides of protected virtual functions from Display.  As much
        // as possible, when Displays are not enabled, they should not be
        // subscribed to incoming data and should not show anything in the
        // 3D view.  These functions are where these connections are made
        // and broken.

        // Called after the constructors have run
        virtual void onInitialize();

        // Called periodically by the visualization manager
        virtual void update(float wall_dt, float ros_dt);

    protected:
        // A helper to clear this display back to the initial state.
        virtual void reset();

        // Must be implemented by derived classes because MOC doesn't work in templates
        virtual rviz::DisplayContext* getContext() {
            return context_;
        }

    private:
        struct SocialActivityVisual {
            vector<boost::shared_ptr<rviz::Shape> > socialActivityAssignmentCircles;
            vector<boost::shared_ptr<rviz::BillboardLine> > connectionLines;
            vector< boost::shared_ptr<TextNode> > typeTexts;
            vector<track_id> trackIds;
            activity_type activityType;
            float confidence;
            geometry_msgs::Point socialActivityCenter;
            size_t personCount;
            size_t declutteringOffset; // for decluttering of labels etc. in case of multiple activities per track, e.g. 1 = shift label by 1 row
        };

        // Functions to handle an incoming ROS message.
        void processMessage(const spencer_social_relation_msgs::SocialActivities::ConstPtr& msg);

        // Helper functions
        void updateSocialActivityVisualStyles(boost::shared_ptr<SocialActivityVisual>& groupVisual);
        bool isActivityTypeHidden(activity_type activityType);
        Ogre::ColourValue getActivityColor(activity_type activityType, float confidence);

        // Scene nodes
        boost::shared_ptr<Ogre::SceneNode> m_socialActivitiesSceneNode;

        // User-editable property variables.
        rviz::StringProperty* m_excluded_activity_types_property;
        rviz::StringProperty* m_included_activity_types_property;

        rviz::BoolProperty* m_render_intraactivity_connections_property;
        rviz::BoolProperty* m_render_activity_types_property;
        rviz::BoolProperty* m_activity_type_per_track_property;
        rviz::BoolProperty* m_render_confidences_property;
        rviz::BoolProperty* m_render_circles_property;
        rviz::BoolProperty* m_hide_with_no_activity_property;

        rviz::FloatProperty* m_occlusion_alpha_property;
        rviz::FloatProperty* m_min_confidence_property;
        rviz::FloatProperty* m_circle_radius_property;
        rviz::FloatProperty* m_circle_alpha_property;
        rviz::FloatProperty* m_line_width_property;
        rviz::FloatProperty* m_activity_type_offset; // z offset of the group ID text

        rviz::Property* m_activity_colors;
        rviz::ColorProperty* m_activity_color_none;
        rviz::ColorProperty* m_activity_color_unknown;
        rviz::ColorProperty* m_activity_color_shopping;
        rviz::ColorProperty* m_activity_color_standing;
        rviz::ColorProperty* m_activity_color_individual_moving;
        rviz::ColorProperty* m_activity_color_waiting_in_queue;
        rviz::ColorProperty* m_activity_color_looking_at_information_screen;
        rviz::ColorProperty* m_activity_color_looking_at_kiosk;
        rviz::ColorProperty* m_activity_color_group_assembling;
        rviz::ColorProperty* m_activity_color_group_moving;
        rviz::ColorProperty* m_activity_color_flow;
        rviz::ColorProperty* m_activity_color_antiflow;
        rviz::ColorProperty* m_activity_color_waiting_for_others;
        rviz::ColorProperty* m_activity_color_looking_for_help;


        // State variables
        struct PersonVisualContainer {
            boost::shared_ptr<PersonVisual> personVisual;
            boost::shared_ptr<Ogre::SceneNode> sceneNode;
            track_id trackId;
        };

        vector<boost::shared_ptr<SocialActivityVisual> > m_socialActivityVisuals;
        map<track_id, PersonVisualContainer > m_personVisualMap; // to keep person visuals alive across multiple frames, for walking animation

        map<track_id, ActivityWithConfidence> m_highestConfidenceActivityPerTrack; // only highest-confidence activity per person
        map<track_id, vector<ActivityWithConfidence> > m_allActivitiesPerTrack; // all activities that a track is involved in

        set<activity_type> m_excludedActivityTypes, m_includedActivityTypes;

        Ogre::Matrix4 m_frameTransform;
        TrackedPersonsCache m_trackedPersonsCache;

    private Q_SLOTS:
        void personVisualTypeChanged();
        virtual void stylesChanged();
    };

} // end namespace spencer_tracking_rviz_plugin

#endif // SOCIAL_ACTIVITIES_DISPLAY_H
