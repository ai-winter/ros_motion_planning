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

#ifndef SOCIAL_RELATIONS_DISPLAY_H
#define SOCIAL_RELATIONS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <spencer_social_relation_msgs/SocialRelations.h>
#include "person_display_common.h"
#include "tracked_persons_cache.h"
#endif

namespace spencer_tracking_rviz_plugin
{
    /// The display which can be added in RViz to display social relations.
    class SocialRelationsDisplay: public PersonDisplayCommon<spencer_social_relation_msgs::SocialRelations>
    {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        SocialRelationsDisplay() {};
        virtual ~SocialRelationsDisplay();

        // Overrides of protected virtual functions from Display.  As much
        // as possible, when Displays are not enabled, they should not be
        // subscribed to incoming data and should not show anything in the
        // 3D view.  These functions are where these connections are made
        // and broken.

        // Called after the constructors have run
        virtual void onInitialize();

    protected:
        // A helper to clear this display back to the initial state.
        virtual void reset();

        // Must be implemented by derived classes because MOC doesn't work in templates
        virtual rviz::DisplayContext* getContext() {
            return context_;
        }

    private:
        struct RelationVisual {
            std::string type;
            double relationStrength;
            boost::shared_ptr<rviz::BillboardLine> relationLine;
            boost::shared_ptr<TextNode> relationText;
            track_id trackId1, trackId2; // required to hide certain tracks
        };

        // Functions to handle an incoming ROS message.
        void processMessage(const spencer_social_relation_msgs::SocialRelations::ConstPtr& msg);
        
        // Helper functions
        void updateRelationVisualStyles(boost::shared_ptr<RelationVisual>& relationVisual);
        
        // Scene node for group affiliation history visualization
        boost::shared_ptr<Ogre::SceneNode> m_socialRelationsSceneNode;

        // User-editable property variables.
        rviz::StringProperty* m_relation_type_filter_property;
        
        rviz::BoolProperty* m_render_positive_person_relations_property;
        rviz::BoolProperty* m_render_negative_person_relations_property;

        rviz::FloatProperty* m_positive_person_relation_threshold;
        rviz::ColorProperty* m_positive_person_relations_color;
        rviz::ColorProperty* m_negative_person_relations_color;

        // State variables
        vector<boost::shared_ptr<RelationVisual> > m_relationVisuals;
        TrackedPersonsCache m_trackedPersonsCache;

    private Q_SLOTS:
        virtual void stylesChanged();
    };

} // end namespace spencer_tracking_rviz_plugin

#endif // SOCIAL_RELATIONS_DISPLAY_H
