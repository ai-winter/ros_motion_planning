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

#include "detected_persons_display.h"
#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#endif
#define foreach BOOST_FOREACH

namespace spencer_tracking_rviz_plugin
{

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
void DetectedPersonsDisplay::onInitialize()
{
    PersonDisplayCommon::onInitialize();

    QObject::connect(m_commonProperties->style, SIGNAL(changed()), this, SLOT(personVisualTypeChanged()) );

    m_render_covariances_property       = new rviz::BoolProperty( "Render covariances", true, "Render track covariance ellipses", this, SLOT(stylesChanged()) );
    m_render_detection_ids_property     = new rviz::BoolProperty( "Render detection IDs", true, "Render IDs of the detection that a track was matched against, if any", this, SLOT(stylesChanged()));
    m_render_confidences_property       = new rviz::BoolProperty( "Render confidences", false, "Render detection confidences", this, SLOT(stylesChanged()));
    m_render_orientations_property      = new rviz::BoolProperty( "Render orientation arrows", true, "Render orientation arrows (only if orientation covariances are finite!)", this, SLOT(stylesChanged()));
    m_render_modality_text_property     = new rviz::BoolProperty( "Render modality text", false, "Render detection modality as text below detected person", this, SLOT(stylesChanged()));

    m_text_spacing_property = new rviz::FloatProperty( "Text spacing", 1.0, "Factor for vertical spacing betweent texts", this, SLOT(stylesChanged()), this );
    
    m_low_confidence_threshold_property = new rviz::FloatProperty( "Low-confidence threshold", 0.5, "Detection confidence below which alpha will be reduced", this, SLOT(stylesChanged()));
    m_low_confidence_alpha_property     = new rviz::FloatProperty( "Low-confidence alpha", 0.5, "Alpha multiplier for detections with confidence below low-confidence threshold", this, SLOT(stylesChanged()));

    m_covariance_line_width_property = new rviz::FloatProperty( "Line width", 0.1, "Line width of covariance ellipses", m_render_covariances_property, SLOT(stylesChanged()), this );
}

DetectedPersonsDisplay::~DetectedPersonsDisplay()
{
    m_previousDetections.clear();
}

// Clear the visuals by deleting their objects.
void DetectedPersonsDisplay::reset()
{
    PersonDisplayCommon::reset();
    m_previousDetections.clear();
}

// Set the rendering style (cylinders, meshes, ...) of detected persons
void DetectedPersonsDisplay::personVisualTypeChanged()
{
    foreach(boost::shared_ptr<DetectedPersonVisual>& detectedPersonVisual, m_previousDetections)
    {
        detectedPersonVisual->personVisual.reset();
        createPersonVisualIfRequired(detectedPersonVisual->sceneNode.get(), detectedPersonVisual->personVisual);
    }
    stylesChanged();
}

// Update dynamically adjustable properties of all existing detections
void DetectedPersonsDisplay::stylesChanged()
{
    foreach(boost::shared_ptr<DetectedPersonVisual> detectedPersonVisual, m_previousDetections)
    {
        bool personHidden = isPersonHidden(detectedPersonVisual->detectionId);

        // Update common styles to person visual, such as line width
        applyCommonStyles(detectedPersonVisual->personVisual);

        // Get current detection color
        Ogre::ColourValue detectionColor = getColorFromId(detectedPersonVisual->detectionId);
        detectionColor.a *= m_commonProperties->alpha->getFloat(); // general alpha
        if(personHidden) detectionColor.a = 0.0;
        if(detectedPersonVisual->confidence < m_low_confidence_threshold_property->getFloat()) detectionColor.a *= m_low_confidence_alpha_property->getFloat();

        if(detectedPersonVisual->personVisual) {
            detectedPersonVisual->personVisual->setColor(detectionColor);
        }

        // Update texts
        Ogre::ColourValue fontColor = m_commonProperties->font_color_style->getOptionInt() == FONT_COLOR_CONSTANT ? m_commonProperties->constant_font_color->getOgreColor() : detectionColor;
        fontColor.a = m_commonProperties->alpha->getFloat();
        if(personHidden) fontColor.a = 0.0;
        
        float textOffset = 0.0f;
        detectedPersonVisual->detectionIdText->setCharacterHeight(0.18 * m_commonProperties->font_scale->getFloat());
        detectedPersonVisual->detectionIdText->setPosition(Ogre::Vector3(0,0, -0.5*detectedPersonVisual->detectionIdText->getCharacterHeight() - textOffset));
        detectedPersonVisual->detectionIdText->setVisible(m_render_detection_ids_property->getBool());
        detectedPersonVisual->detectionIdText->setColor(fontColor);
        if(m_render_detection_ids_property->getBool()) textOffset += m_text_spacing_property->getFloat() * detectedPersonVisual->detectionIdText->getCharacterHeight();

        detectedPersonVisual->modalityText->setCharacterHeight(0.18 * m_commonProperties->font_scale->getFloat());
        detectedPersonVisual->modalityText->setPosition(Ogre::Vector3(textOffset, 0, -0.5*detectedPersonVisual->modalityText->getCharacterHeight() - textOffset));
        detectedPersonVisual->modalityText->setVisible(m_render_modality_text_property->getBool());
        detectedPersonVisual->modalityText->setColor(fontColor);
        if(m_render_modality_text_property->getBool()) textOffset += m_text_spacing_property->getFloat() * detectedPersonVisual->modalityText->getCharacterHeight();

        detectedPersonVisual->confidenceText->setCharacterHeight(0.13 * m_commonProperties->font_scale->getFloat());
        detectedPersonVisual->confidenceText->setPosition(Ogre::Vector3(textOffset, 0, -0.5*detectedPersonVisual->confidenceText->getCharacterHeight() - textOffset));
        detectedPersonVisual->confidenceText->setVisible(m_render_confidences_property->getBool());
        detectedPersonVisual->confidenceText->setColor(fontColor);
        if(m_render_confidences_property->getBool()) textOffset += m_text_spacing_property->getFloat() * detectedPersonVisual->confidenceText->getCharacterHeight();

        // Set color of covariance visualization
        Ogre::ColourValue covarianceColor = detectionColor;
        if(!m_render_covariances_property->getBool()) covarianceColor.a = 0.0;
        detectedPersonVisual->covarianceVisual->setColor(covarianceColor);
        detectedPersonVisual->covarianceVisual->setLineWidth(m_covariance_line_width_property->getFloat());

        // Update orientation arrow
        double arrowAlpha = m_render_orientations_property->getBool() && detectedPersonVisual->hasValidOrientation ? detectionColor.a : 0.0;
        detectedPersonVisual->orientationArrow->setColor(Ogre::ColourValue(detectionColor.r, detectionColor.g, detectionColor.b, arrowAlpha));
        const double shaftLength = 0.5, shaftDiameter = 0.05, headLength = 0.2, headDiameter = 0.2;
        detectedPersonVisual->orientationArrow->set(shaftLength, shaftDiameter, headLength, headDiameter);
    }
}

// This is our callback to handle an incoming message.
void DetectedPersonsDisplay::processMessage(const spencer_tracking_msgs::DetectedPersons::ConstPtr& msg)
{
    // Get transforms into fixed frame etc.
    if(!preprocessMessage(msg)) return;

    const Ogre::Quaternion shapeQuaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ); // required to fix orientation of any Cylinder shapes
    stringstream ss;

    // Clear previous detections, this will also delete them from the scene graph
    m_previousDetections.clear();

    //
    // Iterate over all detections in this message and create a visual representation
    //
    for (vector<spencer_tracking_msgs::DetectedPerson>::const_iterator detectedPersonIt = msg->detections.begin(); detectedPersonIt != msg->detections.end(); ++detectedPersonIt)
    {
        boost::shared_ptr<DetectedPersonVisual> detectedPersonVisual;

        // Create a new visual representation of the detected person
        detectedPersonVisual = boost::shared_ptr<DetectedPersonVisual>(new DetectedPersonVisual);
        m_previousDetections.push_back(detectedPersonVisual);

        // This scene node is the parent of all visualization elements for the detected person
        detectedPersonVisual->sceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
        detectedPersonVisual->detectionId = detectedPersonIt->detection_id;
        detectedPersonVisual->confidence = detectedPersonIt->confidence;
        Ogre::SceneNode* currentSceneNode = detectedPersonVisual->sceneNode.get();


        //
        // Person visualization
        //

        // Create new visual for the person itself, if needed
        boost::shared_ptr<PersonVisual> &personVisual = detectedPersonVisual->personVisual;
        createPersonVisualIfRequired(currentSceneNode, personVisual);

        const double personHeight = personVisual ? personVisual->getHeight() : 0;
        const double halfPersonHeight = personHeight / 2.0;


        //
        // Position & visibility of entire detection
        //

        const Ogre::Matrix3 covXYZinTargetFrame = covarianceXYZIntoTargetFrame(detectedPersonIt->pose);
        setPoseOrientation(currentSceneNode, detectedPersonIt->pose, covXYZinTargetFrame, personHeight);

        //
        // Texts
        //
        {
            // Detection ID
            if (!detectedPersonVisual->detectionIdText) {
                detectedPersonVisual->detectionIdText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
                detectedPersonVisual->detectionIdText->showOnTop();
            }

            ss.str(""); ss << "det " << detectedPersonIt->detection_id;
            detectedPersonVisual->detectionIdText->setCaption(ss.str());

            // Confidence value
            if (!detectedPersonVisual->confidenceText) {
                detectedPersonVisual->confidenceText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
            }

            ss.str(""); ss << fixed << setprecision(2) << detectedPersonIt->confidence;
            detectedPersonVisual->confidenceText->setCaption(ss.str());
            detectedPersonVisual->confidenceText->showOnTop();

            // Modality text
            if (!detectedPersonVisual->modalityText) {
                detectedPersonVisual->modalityText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
            }

            ss.str(""); ss << detectedPersonIt->modality;
            detectedPersonVisual->modalityText->setCaption(ss.str());
            detectedPersonVisual->modalityText->showOnTop();
        }

        //
        // Covariance visualization
        //
        if(!detectedPersonVisual->covarianceVisual) {
            detectedPersonVisual->covarianceVisual.reset(new ProbabilityEllipseCovarianceVisual(context_->getSceneManager(), currentSceneNode));
        }

        // Update covariance ellipse
        {
            Ogre::Vector3 covarianceMean(0,0,0); // zero mean because parent node is already centered at pose mean
            detectedPersonVisual->covarianceVisual->setOrientation(currentSceneNode->getOrientation().Inverse());
            detectedPersonVisual->covarianceVisual->setMeanCovariance(covarianceMean, covXYZinTargetFrame);
        }


        //
        // Orientation arrows
        //
        if (!detectedPersonVisual->orientationArrow) {
            detectedPersonVisual->orientationArrow.reset(new rviz::Arrow(context_->getSceneManager(), currentSceneNode));
        }

        // Update orientation arrow
        {
            const Ogre::Vector3 forwardVector(1,0,0);

            const double personRadius = 0.2;
            const Ogre::Vector3 arrowAttachPoint(personRadius, 0, halfPersonHeight); // relative to tracked person's scene node
            detectedPersonVisual->orientationArrow->setPosition(arrowAttachPoint);
            detectedPersonVisual->orientationArrow->setOrientation(Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(forwardVector));
            detectedPersonVisual->hasValidOrientation = hasValidOrientation(detectedPersonIt->pose);
        }

    } // end for loop over all detected persons

    // Set all properties that can dynamically be adjusted in the GUI
    stylesChanged();

    //
    // Update status (shown in property pane)
    //
    ss.str("");
    ss << msg->detections.size() << " detections received";
    setStatusStd(rviz::StatusProperty::Ok, "Tracks", ss.str());
}

} // end namespace spencer_tracking_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spencer_tracking_rviz_plugin::DetectedPersonsDisplay, rviz::Display)
