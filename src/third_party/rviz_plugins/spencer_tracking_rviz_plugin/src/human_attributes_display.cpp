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

#include <rviz/frame_manager.h>
#include <rviz/selection/selection_manager.h>

#ifndef Q_MOC_RUN
#include "human_attributes_display.h"
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>
#endif
#define foreach BOOST_FOREACH

namespace spencer_tracking_rviz_plugin
{

void HumanAttributesDisplay::onInitialize()
{
    m_trackedPersonsCache.initialize(this, context_, update_nh_);
    PersonDisplayCommon::onInitialize();
    
    m_render_gender_property = new rviz::BoolProperty( "Render gender", true, "Render gender visual", this, SLOT(stylesChanged()));
    m_render_age_group_property = new rviz::BoolProperty( "Render age group", true, "Render age group visual", this, SLOT(stylesChanged()));
    m_render_person_height_property = new rviz::BoolProperty( "Render person height", true, "Render person height", this, SLOT(stylesChanged()));
    
    m_occlusion_alpha_property = new rviz::FloatProperty( "Occlusion alpha", 0.5, "Alpha multiplier for occluded tracks", this, SLOT(stylesChanged()) );
    m_occlusion_alpha_property->setMin( 0.0 );

    m_commonProperties->z_offset->setFloat(2.7f);
    m_commonProperties->style->setHidden(true);
}

HumanAttributesDisplay::~HumanAttributesDisplay()
{
}

// Clear the visuals by deleting their objects.
void HumanAttributesDisplay::reset()
{
    PersonDisplayCommon::reset();
    m_trackedPersonsCache.reset();
    m_humanAttributeVisuals.clear();
    scene_node_->removeAndDestroyAllChildren(); // not sure if required?
}

void HumanAttributesDisplay::update(float wall_dt, float ros_dt)
{}

void HumanAttributesDisplay::stylesChanged()
{
    foreach(boost::shared_ptr<HumanAttributeVisual> humanAttributeVisual, m_humanAttributeVisuals | boost::adaptors::map_values) {
        updateVisualStyles(humanAttributeVisual);
    }
}

void HumanAttributesDisplay::updateVisualStyles(boost::shared_ptr<HumanAttributeVisual>& humanAttributeVisual)
{
    track_id trackId = humanAttributeVisual->trackId;
    bool personHidden = isPersonHidden(trackId);

    boost::shared_ptr<CachedTrackedPerson> trackedPerson = m_trackedPersonsCache.lookup(trackId);
    float occlusionAlpha = trackedPerson->isOccluded ? m_occlusion_alpha_property->getFloat() : 1.0;

    // Update text colors, size and visibility
    Ogre::ColourValue fontColor = m_commonProperties->font_color_style->getOptionInt() == FONT_COLOR_CONSTANT ? m_commonProperties->constant_font_color->getOgreColor() : getColorFromId(trackId);
    fontColor.a = m_commonProperties->alpha->getFloat() * occlusionAlpha;
    if(personHidden) fontColor.a = 0;

    humanAttributeVisual->ageGroupText->setVisible(m_render_age_group_property->getBool());
    humanAttributeVisual->ageGroupText->setCharacterHeight(0.17 * m_commonProperties->font_scale->getFloat());
    humanAttributeVisual->ageGroupText->setColor(fontColor);
    humanAttributeVisual->ageGroupText->setPosition(Ogre::Vector3(0, 0, 0.17 * m_commonProperties->font_scale->getFloat()) );

    humanAttributeVisual->personHeightText->setVisible(m_render_person_height_property->getBool());
    humanAttributeVisual->personHeightText->setCharacterHeight(0.17 * m_commonProperties->font_scale->getFloat());
    humanAttributeVisual->personHeightText->setColor(fontColor);
    humanAttributeVisual->personHeightText->setPosition(Ogre::Vector3(0, 0, 0) );

    if(humanAttributeVisual->genderMesh) {
        humanAttributeVisual->genderMesh->setPosition(Ogre::Vector3(0, 0, 0.17 * 2 * m_commonProperties->font_scale->getFloat() + 0.3));
        humanAttributeVisual->genderMesh->setVisible(m_render_gender_property->getBool());
    }
}

boost::shared_ptr<HumanAttributesDisplay::HumanAttributeVisual> HumanAttributesDisplay::createVisualIfNotExists(track_id trackId)
{
    if(m_humanAttributeVisuals.find(trackId) == m_humanAttributeVisuals.end()) {
        boost::shared_ptr<HumanAttributeVisual> humanAttributeVisual = boost::shared_ptr<HumanAttributeVisual>(new HumanAttributeVisual);

        humanAttributeVisual->sceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());

        humanAttributeVisual->ageGroupText = boost::shared_ptr<TextNode>(new TextNode(context_->getSceneManager(), humanAttributeVisual->sceneNode.get()));
        humanAttributeVisual->ageGroupText->showOnTop();
        humanAttributeVisual->ageGroupText->setCaption(" ");

        humanAttributeVisual->personHeightText = boost::shared_ptr<TextNode>(new TextNode(context_->getSceneManager(), humanAttributeVisual->sceneNode.get()));
        humanAttributeVisual->personHeightText->showOnTop();
        humanAttributeVisual->personHeightText->setCaption(" ");

        humanAttributeVisual->trackId = trackId;

        m_humanAttributeVisuals[trackId] = humanAttributeVisual;
    }


    return m_humanAttributeVisuals[trackId];
}

// This is our callback to handle an incoming group message.
void HumanAttributesDisplay::processMessage(const spencer_human_attribute_msgs::HumanAttributes::ConstPtr& msg)
{
    // Get transforms into fixed frame etc.
    if(!preprocessMessage(msg)) return;

    // Transform into Rviz fixed frame
    m_frameTransform = Ogre::Matrix4(m_frameOrientation);
    m_frameTransform.setTrans(m_framePosition);

    const Ogre::Quaternion shapeQuaternion( Ogre::Degree(90), Ogre::Vector3(1,0,0) ); // required to fix orientation of any Cylinder shapes
    stringstream ss;

    //
    // Iterate over all categorical attributes in this message
    //
    foreach (const spencer_human_attribute_msgs::CategoricalAttribute& categoricalAttribute, msg->categoricalAttributes)
    {
        // Check if there is already a visual for this particular track
        track_id trackId = categoricalAttribute.subject_id; // assumes subject_id is a track_id (not detection_id)
        boost::shared_ptr<HumanAttributeVisual> humanAttributeVisual = createVisualIfNotExists(trackId);

        if(categoricalAttribute.values.empty()) {
            ROS_ERROR_STREAM("categoricalAttribute.values.empty() for track ID " << trackId << ", attribute " << categoricalAttribute.type);
            continue;
        }
        if(categoricalAttribute.confidences.size() != categoricalAttribute.values.size()) {
            ROS_WARN_STREAM("categoricalAttribute.confidences.size() != categoricalAttribute.values.size() for track ID " << trackId << ", attribute " << categoricalAttribute.type);
        }

        // Find highest-ranking attribute
        size_t highestRankingIndex = 0; float highestConfidence = -999999;
        for(size_t i = 0; i < categoricalAttribute.confidences.size(); i++) {
            if(categoricalAttribute.confidences[i] > highestConfidence) {
                highestConfidence = categoricalAttribute.confidences[i];
                highestRankingIndex = i;
            }
        }

        std::string valueWithHighestConfidence = categoricalAttribute.values[highestRankingIndex];

        // Age group
        if(categoricalAttribute.type == spencer_human_attribute_msgs::CategoricalAttribute::AGE_GROUP) {
            ss.str(""); ss << valueWithHighestConfidence << "yrs";
            humanAttributeVisual->ageGroupText->setCaption(ss.str());
        }

        // Gender
        else if(categoricalAttribute.type == spencer_human_attribute_msgs::CategoricalAttribute::GENDER) {
            ss.str(""); ss << "package://" ROS_PACKAGE_NAME "/media/" << valueWithHighestConfidence << "_symbol.dae";
            std::string meshResource = ss.str();
            
            humanAttributeVisual->genderMesh = boost::shared_ptr<MeshNode>(new MeshNode(context_, humanAttributeVisual->sceneNode.get(), meshResource));
            
            Ogre::ColourValue meshColor(1, 1, 1, 1);
            if(valueWithHighestConfidence == spencer_human_attribute_msgs::CategoricalAttribute::GENDER_MALE) meshColor = Ogre::ColourValue(0, 1, 1, 1);
            if(valueWithHighestConfidence == spencer_human_attribute_msgs::CategoricalAttribute::GENDER_FEMALE) meshColor = Ogre::ColourValue(1, 0, 1, 1);
            humanAttributeVisual->genderMesh->setColor(meshColor);

            humanAttributeVisual->genderMesh->setScale(0.5);
            humanAttributeVisual->genderMesh->setCameraFacing(true);            
        }
    }


    //
    // Iterate over all scalar attributes in this message
    //
    foreach (const spencer_human_attribute_msgs::ScalarAttribute& scalarAttribute, msg->scalarAttributes)
    {
        // Check if there is already a visual for this particular track
        track_id trackId = scalarAttribute.subject_id; // assumes subject_id is a track_id (not detection_id)
        boost::shared_ptr<HumanAttributeVisual> humanAttributeVisual = createVisualIfNotExists(trackId);
        
        if(scalarAttribute.values.empty()) {
            ROS_ERROR_STREAM("scalarAttribute.values.empty() for track ID " << trackId << ", attribute " << scalarAttribute.type);
            continue;
        }
        if(scalarAttribute.confidences.size() != scalarAttribute.values.size()) {
            ROS_WARN_STREAM("scalarAttribute.confidences.size() != scalarAttribute.values.size() for track ID " << trackId << ", attribute " << scalarAttribute.type);
        }

        // Find highest-ranking attribute
        size_t highestRankingIndex = 0; float highestConfidence = -999999;
        for(size_t i = 0; i < scalarAttribute.confidences.size(); i++) {
            if(scalarAttribute.confidences[i] > highestConfidence) {
                highestConfidence = scalarAttribute.confidences[i];
                highestRankingIndex = i;
            }
        }

        float valueWithHighestConfidence = scalarAttribute.values[highestRankingIndex];

        // Person height
        if(scalarAttribute.type == spencer_human_attribute_msgs::ScalarAttribute::PERSON_HEIGHT) {
            ss.str(""); ss << std::fixed << std::setprecision(2) << valueWithHighestConfidence << "m";
            humanAttributeVisual->personHeightText->setCaption(ss.str());
        }            
    }


    //
    // Update position and style of all existing person visuals
    //
    set<track_id> tracksWithUnknownPosition;
    foreach(boost::shared_ptr<HumanAttributeVisual> humanAttributeVisual, m_humanAttributeVisuals | boost::adaptors::map_values)
    {
        boost::shared_ptr<CachedTrackedPerson> trackedPerson = m_trackedPersonsCache.lookup(humanAttributeVisual->trackId);

        // Get current track position
        if(!trackedPerson) {
            tracksWithUnknownPosition.insert(humanAttributeVisual->trackId);
        }
        else
        {   // Track position is known
            humanAttributeVisual->sceneNode->setPosition(trackedPerson->center + Ogre::Vector3(0, 0, m_commonProperties->z_offset->getFloat()));

            // Update styles
            updateVisualStyles(humanAttributeVisual);
        }
    }


    // Remove visuals for tracks with unknown position
    foreach(track_id trackId, tracksWithUnknownPosition) {
        m_humanAttributeVisuals.erase(trackId);
    }


    //
    // Update display status (shown in property pane)
    //

    ss.str("");
    ss << msg->categoricalAttributes.size() << " categorical attribute(s)";
    setStatusStd(rviz::StatusProperty::Ok, "Categorical attributes", ss.str());

    ss.str("");
    ss << msg->scalarAttributes.size() << " scalar attribute(s)";
    setStatusStd(rviz::StatusProperty::Ok, "Scalar attributes", ss.str());

    ss.str("");
    ss << tracksWithUnknownPosition.size() << " track(s) with unknown position";
    setStatusStd(0 == tracksWithUnknownPosition.size() ? rviz::StatusProperty::Ok : rviz::StatusProperty::Warn, "Attribute-to-track assignment", ss.str());
}

} // end namespace spencer_tracking_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spencer_tracking_rviz_plugin::HumanAttributesDisplay, rviz::Display)
