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

#ifndef DETECTED_PERSONS_DISPLAY_H
#define DETECTED_PERSONS_DISPLAY_H

#ifndef Q_MOC_RUN
#include <map>
#include <boost/circular_buffer.hpp>
#include <spencer_tracking_msgs/DetectedPersons.h>
#include "person_display_common.h"
#endif

namespace spencer_tracking_rviz_plugin
{
    /// The visual of a tracked person.
    struct DetectedPersonVisual
    {
        boost::shared_ptr<Ogre::SceneNode> sceneNode;

        boost::shared_ptr<PersonVisual> personVisual;
        boost::shared_ptr<TextNode> detectionIdText, confidenceText, modalityText;
        boost::shared_ptr<rviz::Arrow> orientationArrow;
        boost::shared_ptr<CovarianceVisual> covarianceVisual;

        float confidence;
        bool hasValidOrientation;
        unsigned int detectionId;
    };

    // The DetectedPersonsDisplay class itself just implements a circular buffer,
    // editable parameters, and Display subclass machinery.
    class DetectedPersonsDisplay: public PersonDisplayCommon<spencer_tracking_msgs::DetectedPersons>
    {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        DetectedPersonsDisplay() {};
        virtual ~DetectedPersonsDisplay();

        // Overrides of protected virtual functions from Display.  As much
        // as possible, when Displays are not enabled, they should not be
        // subscribed to incoming data and should not show anything in the
        // 3D view.  These functions are where these connections are made
        // and broken.
        
        virtual void onInitialize();

    protected:
        // A helper to clear this display back to the initial state.
        virtual void reset();

        // Must be implemented by derived classes because MOC doesn't work in templates
        virtual rviz::DisplayContext* getContext() {
            return context_;
        }

    private Q_SLOTS:
        void personVisualTypeChanged();

        // Called whenever one of the properties in PersonDisplayCommonProperties has been changed
        virtual void stylesChanged();

    private:
        // Function to handle an incoming ROS message.
        void processMessage(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg);
       
        // All currently active tracks, with unique track ID as map key
        vector<boost::shared_ptr<DetectedPersonVisual> > m_previousDetections;

        // Properties
        rviz::BoolProperty* m_render_covariances_property;
        rviz::BoolProperty* m_render_detection_ids_property;
        rviz::BoolProperty* m_render_confidences_property;
        rviz::FloatProperty* m_low_confidence_threshold_property;
        rviz::FloatProperty* m_low_confidence_alpha_property;
        rviz::BoolProperty* m_render_orientations_property;
        rviz::BoolProperty* m_render_modality_text_property;

        rviz::FloatProperty* m_text_spacing_property;
        rviz::FloatProperty* m_covariance_line_width_property;
    };

} // end namespace spencer_tracking_rviz_plugin

#endif // DETECTED_PERSONS_DISPLAY_H
