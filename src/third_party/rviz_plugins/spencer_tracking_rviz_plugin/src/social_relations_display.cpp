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
#include "social_relations_display.h"
#include <boost/foreach.hpp>
#endif
#define foreach BOOST_FOREACH

namespace spencer_tracking_rviz_plugin
{

void SocialRelationsDisplay::onInitialize()
{
    m_trackedPersonsCache.initialize(this, context_, update_nh_);
    PersonDisplayCommon::onInitialize();
    
    m_relation_type_filter_property = new rviz::StringProperty( "Relation type filter", "", "Type of social relations to display (see \"type\" field in message). No wildcards allowed. Leave empty to allow any type of relation.", this, SLOT(stylesChanged()));
    
    m_positive_person_relation_threshold = new rviz::FloatProperty( "Positive relation threshold", 0.5, "Above which probability threshold a social relation between tracks is considered as positive", this, SLOT(stylesChanged()));

    m_positive_person_relations_color = new rviz::ColorProperty( "Positive relation color", QColor(0,255,0), "Color for positive track relations", this, SLOT(stylesChanged()));
    m_negative_person_relations_color = new rviz::ColorProperty( "Negative relation color", QColor(255,0,0), "Color for negative track relations", this, SLOT(stylesChanged()));

    m_render_positive_person_relations_property = new rviz::BoolProperty( "Render positive relations", true,  "Render positive person relations", this, SLOT(stylesChanged()));
    m_render_negative_person_relations_property = new rviz::BoolProperty( "Render negative relations", false, "Render negative person relations", this, SLOT(stylesChanged()));
   
    // Create a scene node for visualizing social relations
    m_socialRelationsSceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
}

SocialRelationsDisplay::~SocialRelationsDisplay()
{
}

// Clear the visuals by deleting their objects.
void SocialRelationsDisplay::reset()
{
    PersonDisplayCommon::reset();
    m_trackedPersonsCache.reset();
    m_relationVisuals.clear();
}

void SocialRelationsDisplay::processMessage(const spencer_social_relation_msgs::SocialRelations::ConstPtr& msg)
{
    // Get transforms into fixed frame etc.
    if(!preprocessMessage(msg)) return;

    // Loop over all social relations between tracks
    m_relationVisuals.clear();
    m_socialRelationsSceneNode->removeAndDestroyAllChildren();

    foreach(const spencer_social_relation_msgs::SocialRelation& socialRelation, msg->elements)
    {
        boost::shared_ptr<CachedTrackedPerson> personTrack1 = m_trackedPersonsCache.lookup(socialRelation.track1_id);
        boost::shared_ptr<CachedTrackedPerson> personTrack2 = m_trackedPersonsCache.lookup(socialRelation.track2_id);

        // Cannot draw relations for tracks with unknown position
        if(!personTrack1 || !personTrack2) continue;

        // Create a new visual representation of the tracked person
        boost::shared_ptr<RelationVisual> relationVisual = boost::shared_ptr<RelationVisual>(new RelationVisual);
        relationVisual->type = socialRelation.type;
        relationVisual->relationStrength = socialRelation.strength;
        m_relationVisuals.push_back(relationVisual);

        // Get positions. These are already in fixed frame coordinates!
        const Ogre::Vector3 verticalShift(0,0, 1.0 + m_commonProperties->z_offset->getFloat()), textShift(0,0, 0.3);
        const Ogre::Vector3& position1 = verticalShift + personTrack1->center;
        const Ogre::Vector3& position2 = verticalShift + personTrack2->center;
        const Ogre::Vector3& centerPosition = (position1 + position2) / 2.0;

        // Add line connecting the two tracks
        boost::shared_ptr<rviz::BillboardLine> relationLine(new rviz::BillboardLine(context_->getSceneManager(), m_socialRelationsSceneNode.get()));
        relationLine->setMaxPointsPerLine(2);
        relationLine->addPoint(position1);
        relationLine->addPoint(position2);
        relationVisual->relationLine = relationLine;

        // Add relationship strength text
        stringstream ss;
        boost::shared_ptr<TextNode> relationText(new TextNode(context_->getSceneManager(), m_socialRelationsSceneNode.get()));
        ss.str(""); ss << std::fixed << std::setprecision(0) << socialRelation.strength * 100 << "%";
        relationText->setCaption(ss.str());
        relationText->setPosition(centerPosition + textShift);
        relationText->showOnTop();
        relationVisual->relationText = relationText;

        // Remember to which groups the tracks belong, to be able to hide certain groups and their track-to-track relations
        relationVisual->trackId1 = socialRelation.track1_id;
        relationVisual->trackId2 = socialRelation.track2_id;

        // Update adjustable styles
        updateRelationVisualStyles(relationVisual);
    }
}

void SocialRelationsDisplay::stylesChanged()
{
    foreach(boost::shared_ptr<RelationVisual> relationVisual, m_relationVisuals) {
        updateRelationVisualStyles(relationVisual);
    }
}

void SocialRelationsDisplay::updateRelationVisualStyles(boost::shared_ptr<RelationVisual>& relationVisual)
{
    std::string typeFilter = m_relation_type_filter_property->getStdString();
    bool validRelationType = relationVisual->type.find(typeFilter) != std::string::npos;
    bool hideRelation = !validRelationType || isPersonHidden(relationVisual->trackId1) || isPersonHidden(relationVisual->trackId2);

    // Determine type of the relationship
    bool isPositiveRelation = relationVisual->relationStrength > m_positive_person_relation_threshold->getFloat();

    // Get color
    Ogre::ColourValue relationColor = isPositiveRelation ? m_positive_person_relations_color->getOgreColor() : m_negative_person_relations_color->getOgreColor();
    relationColor.a *= m_commonProperties->alpha->getFloat(); // general alpha
    if(hideRelation) relationColor.a = 0;

    if(isPositiveRelation && !m_render_positive_person_relations_property->getBool()) relationColor.a = 0;
    if(!isPositiveRelation && !m_render_negative_person_relations_property->getBool()) relationColor.a = 0;


    relationVisual->relationLine->setLineWidth(0.03 * (isPositiveRelation ? 1.0 : 0.3));
    relationVisual->relationLine->setColor(relationColor.r, relationColor.g, relationColor.b, relationColor.a);

    relationVisual->relationText->setCharacterHeight(0.15 * m_commonProperties->font_scale->getFloat());
    relationVisual->relationText->setColor(relationColor);
}


} // end namespace spencer_tracking_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spencer_tracking_rviz_plugin::SocialRelationsDisplay, rviz::Display)
