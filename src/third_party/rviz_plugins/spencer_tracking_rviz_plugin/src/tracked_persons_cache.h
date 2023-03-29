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

#ifndef TRACKED_PERSONS_CACHE_H
#define TRACKED_PERSONS_CACHE_H

#ifndef Q_MOC_RUN
#include <map>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#endif
#include "additional_topic_subscriber.h"

namespace spencer_tracking_rviz_plugin
{    
    typedef unsigned int track_id;

    /// Data structure for storing information about individual person tracks
    struct CachedTrackedPerson
    {
        Ogre::Vector3 center;
        geometry_msgs::PoseWithCovariance pose;
        geometry_msgs::TwistWithCovariance twist;
        bool isOccluded;
    };

    /// Subscribes to a TrackedPersons topic and caches all TrackedPersons of the current cycle, so that
    /// the owning rviz::Display can look up track positions etc for visualization.
    class TrackedPersonsCache {
    public:
        typedef std::map<track_id, boost::shared_ptr<CachedTrackedPerson> > CachedTrackedPersonsMap;

        // Destructor
        ~TrackedPersonsCache();

        /// Create TrackedPersons subscriber and setup RViz properties.
        void initialize(rviz::Display* display, rviz::DisplayContext* context, ros::NodeHandle update_nh);

        /// Clear internal state, including all cached track positions.
        void reset();

        /// Lookup information for the given tracked person ID. Returns a null pointer if no information is available.
        const boost::shared_ptr<CachedTrackedPerson> lookup(track_id trackId);

        /// Return internal map
        const CachedTrackedPersonsMap& getMap() {
            return m_cachedTrackedPersons;
        }

    private:
        // Callback when a new TrackedPersons message has arrived
        void processTrackedPersonsMessage(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg);

        rviz::AdditionalTopicSubscriber<spencer_tracking_msgs::TrackedPersons>* m_tracked_person_subscriber;
        rviz::Display* m_display;
        rviz::DisplayContext* m_context;

        // Our TrackedPerson memory
        CachedTrackedPersonsMap m_cachedTrackedPersons;
    };
    

}

#endif // TRACKED_PERSONS_CACHE_H
