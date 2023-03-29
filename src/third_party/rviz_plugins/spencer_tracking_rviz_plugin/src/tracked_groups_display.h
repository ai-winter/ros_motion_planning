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

#ifndef TRACKED_GROUPS_DISPLAY_H
#define TRACKED_GROUPS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <map>
#include <boost/circular_buffer.hpp>
#endif
#include <spencer_tracking_msgs/TrackedGroups.h>

#include "person_display_common.h"
#include "tracked_persons_cache.h"

namespace spencer_tracking_rviz_plugin
{
    typedef unsigned int group_id;

    /// A single entry in the history of a tracked person, to show group affiliation.
    struct GroupAffiliationHistoryEntry
    {
        group_id groupId;
        boost::shared_ptr<rviz::Shape> shape;
        bool wasOccluded, wasSinglePersonGroup;
    };

    /// History of a tracked person.
    typedef circular_buffer<boost::shared_ptr<GroupAffiliationHistoryEntry> > GroupAffiliationHistory;

    /// The display which can be added in RViz to display tracked groups.
    class TrackedGroupsDisplay: public PersonDisplayCommon<spencer_tracking_msgs::TrackedGroups>
    {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        TrackedGroupsDisplay() {};
        virtual ~TrackedGroupsDisplay();

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
        struct GroupVisual {
            vector<boost::shared_ptr<rviz::Shape> > groupAssignmentCircles;
            vector<boost::shared_ptr<PersonVisual> > personVisuals;
            vector<boost::shared_ptr<Ogre::SceneNode> > personVisualSceneNodes;
            vector<boost::shared_ptr<rviz::BillboardLine> > connectionLines;
            boost::shared_ptr<TextNode> idText;
            group_id groupId;
            geometry_msgs::Point groupCenter;
            size_t personCount;
        };

        // Functions to handle an incoming ROS message.
        void processMessage(const spencer_tracking_msgs::TrackedGroups::ConstPtr& msg);
       
        // Helper functions
        void updateGroupVisualStyles(boost::shared_ptr<GroupVisual>& groupVisual);
        void updateHistoryStyles();
        bool isGroupHidden(group_id groupId);

        // Scene node for group affiliation history visualization
        boost::shared_ptr<Ogre::SceneNode> m_groupAffiliationHistorySceneNode, m_groupsSceneNode;

        std::string m_realFixedFrame;

        // User-editable property variables.
        rviz::StringProperty* m_excluded_group_ids_property;
        rviz::StringProperty* m_included_group_ids_property;

        rviz::BoolProperty* m_render_intragroup_connections_property;
        rviz::BoolProperty* m_render_ids_property;
        rviz::BoolProperty* m_render_history_property;
        rviz::BoolProperty* m_single_person_groups_in_constant_color_property;
        rviz::BoolProperty* m_hide_ids_of_single_person_groups_property;

        rviz::IntProperty*   m_history_length_property;

        rviz::FloatProperty* m_occlusion_alpha_property;        
        rviz::FloatProperty* m_group_id_offset; // z offset of the group ID text

        // State variables
        vector<boost::shared_ptr<GroupVisual> > m_groupVisuals;
        
        map<track_id, group_id> m_groupAffiliations;
        GroupAffiliationHistory m_groupAffiliationHistory;
        set<group_id> m_excludedGroupIDs, m_includedGroupIDs;

        Ogre::Matrix4 m_frameTransform;
        TrackedPersonsCache m_trackedPersonsCache;

    private Q_SLOTS:
        void personVisualTypeChanged();
        virtual void stylesChanged();
    };

} // end namespace spencer_tracking_rviz_plugin

#endif // TRACKED_GROUPS_DISPLAY_H
