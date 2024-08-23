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

#ifndef Q_MOC_RUN
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include "rviz/selection/selection_manager.h"
#include "tracked_persons_display.h"
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#endif
#define foreach BOOST_FOREACH

namespace spencer_tracking_rviz_plugin
{
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
void TrackedPersonsDisplay::onInitialize()
{
  PersonDisplayCommon::onInitialize();

  m_realFixedFrame = "odom";
  QObject::connect(m_commonProperties->style, SIGNAL(changed()), this, SLOT(personVisualTypeChanged()));

  m_occlusion_alpha_property = new rviz::FloatProperty("Occlusion alpha", 0.3, "Alpha multiplier for occluded tracks",
                                                       this, SLOT(stylesChanged()));
  m_occlusion_alpha_property->setMin(0.0);

  m_missed_alpha_property =
      new rviz::FloatProperty("Missed alpha", 0.5, "Alpha multiplier for missed tracks", this, SLOT(stylesChanged()));
  m_missed_alpha_property->setMin(0.0);

  m_history_length_property = new rviz::IntProperty("History size", 100, "Number of prior track positions to display.",
                                                    this, SLOT(stylesChanged()));
  m_history_length_property->setMin(1);
  m_history_length_property->setMax(10000000);

  m_delete_after_ncycles_property = new rviz::IntProperty("Delete after no. cycles", 100,
                                                          "After how many time steps to delete an old track that has "
                                                          "not been seen again, including its history",
                                                          this, SLOT(stylesChanged()));
  m_delete_after_ncycles_property->setMin(0);
  m_delete_after_ncycles_property->setMax(10000000);

  m_show_deleted_property = new rviz::BoolProperty(
      "Show DELETED tracks", false, "Show tracks which have been marked as deleted", this, SLOT(stylesChanged()));
  m_show_occluded_property = new rviz::BoolProperty("Show OCCLUDED tracks", true,
                                                    "Show tracks which could not be matched to an detection due to "
                                                    "sensor occlusion",
                                                    this, SLOT(stylesChanged()));
  m_show_missed_property = new rviz::BoolProperty("Show MISSED tracks", true,
                                                  "Show tracks which could not be matched to an detection but should "
                                                  "be observable by the sensor",
                                                  this, SLOT(stylesChanged()));
  m_show_matched_property = new rviz::BoolProperty(
      "Show MATCHED tracks", true, "Show tracks which could be matched to an detection", this, SLOT(stylesChanged()));

  m_render_history_property =
      new rviz::BoolProperty("Render history", true, "Render prior track positions", this, SLOT(stylesChanged()));
  m_render_history_as_line_property = new rviz::BoolProperty(
      "History as line", true, "Display history as line instead of dots", this, SLOT(stylesChanged()));
  m_render_person_property =
      new rviz::BoolProperty("Render person visual", true, "Render person visualization", this, SLOT(stylesChanged()));
  m_render_covariances_property = new rviz::BoolProperty("Render covariances", true, "Render track covariance ellipses",
                                                         this, SLOT(stylesChanged()));
  m_render_velocities_property =
      new rviz::BoolProperty("Render velocities", true, "Render track velocity arrows", this, SLOT(stylesChanged()));
  m_render_ids_property =
      new rviz::BoolProperty("Render track IDs", true, "Render track IDs as text", this, SLOT(stylesChanged()));
  m_render_detection_ids_property = new rviz::BoolProperty("Render detection IDs", true,
                                                           "Render IDs of the detection that a track was matched "
                                                           "against, if any",
                                                           this, SLOT(stylesChanged()));
  m_render_track_state_property =
      new rviz::BoolProperty("Render track state", true, "Render track state text", this, SLOT(stylesChanged()));

  m_history_min_point_distance_property = new rviz::FloatProperty("Min. history point distance", 0.4,
                                                                  "Minimum distance between history points before a "
                                                                  "new one is placed",
                                                                  this, SLOT(stylesChanged()));
  m_history_line_width_property = new rviz::FloatProperty(
      "Line width", 0.05, "Line width of history", m_render_history_as_line_property, SLOT(stylesChanged()), this);
  m_covariance_line_width_property =
      new rviz::FloatProperty("Line width", 0.1, "Line width of covariance ellipses", m_render_covariances_property,
                              SLOT(stylesChanged()), this);

  // TODO: Implement functionality
  // m_render_state_prediction_property  = new rviz::BoolProperty( "Render state prediction", true, "Render state
  // prediction from Kalman filter", this, SLOT( updateRenderFlags() ));

  // Create a scene node for visualizing track history
  m_trackHistorySceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
}

TrackedPersonsDisplay::~TrackedPersonsDisplay()
{
  m_cachedTracks.clear();
}

// Clear the visuals by deleting their objects.
void TrackedPersonsDisplay::reset()
{
  PersonDisplayCommon::reset();
  m_cachedTracks.clear();
}

void TrackedPersonsDisplay::update(float wall_dt, float ros_dt)
{
  // Move map scene node
  Ogre::Vector3 mapFramePosition;
  Ogre::Quaternion mapFrameOrientation;
  getContext()->getFrameManager()->getTransform(m_realFixedFrame, ros::Time(0), mapFramePosition, mapFrameOrientation);
  Ogre::Matrix4 mapFrameTransform(mapFrameOrientation);
  mapFrameTransform.setTrans(mapFramePosition);
  m_trackHistorySceneNode->setPosition(mapFramePosition);
  m_trackHistorySceneNode->setOrientation(mapFrameOrientation);

  // Update position of deleted tracks (because they are not being updated by ROS messages any more)
  foreach (const track_map::value_type& entry, m_cachedTracks)
  {
    const boost::shared_ptr<TrackedPersonVisual>& trackedPersonVisual = entry.second;
    if (trackedPersonVisual->isDeleted)
    {
      Ogre::Matrix4 poseInCurrentFrame = mapFrameTransform * trackedPersonVisual->lastObservedPose;
      Ogre::Vector3 position = poseInCurrentFrame.getTrans();
      Ogre::Quaternion orientation = poseInCurrentFrame.extractQuaternion();
      if (!position.isNaN() && !orientation.isNaN())
      {
        trackedPersonVisual->sceneNode->setPosition(position);
        trackedPersonVisual->sceneNode->setOrientation(orientation);
      }
    }
    else
    {
      // Update animation etc.
      if (trackedPersonVisual->personVisual)
        trackedPersonVisual->personVisual->update(ros_dt);
    }
  }
}

/// Update all dynamically adjusted visualization properties (colors, font sizes etc.) of all currently tracked persons
void TrackedPersonsDisplay::stylesChanged()
{
  const Ogre::Quaternion shapeQuaternion(Ogre::Degree(90), Ogre::Vector3(1, 0, 0));

  // Update each track
  foreach (const track_map::value_type& entry, m_cachedTracks)
  {
    const track_id trackId = entry.first;
    const boost::shared_ptr<TrackedPersonVisual>& trackedPersonVisual = entry.second;

    // Update common styles to person visual, such as line width
    applyCommonStyles(trackedPersonVisual->personVisual);

    // Update track visibility
    bool trackVisible = !isPersonHidden(trackId);

    if (trackedPersonVisual->isDeleted)
      trackVisible &= m_show_deleted_property->getBool();
    else if (trackedPersonVisual->isOccluded)
      trackVisible &= m_show_occluded_property->getBool();
    else if (trackedPersonVisual->isMissed)
      trackVisible &= m_show_missed_property->getBool();
    else
      trackVisible &= m_show_matched_property->getBool();

    trackedPersonVisual->sceneNode->setVisible(trackVisible);
    trackedPersonVisual->historySceneNode->setVisible(trackVisible && !m_render_history_as_line_property->getBool());
    trackedPersonVisual->historyLineSceneNode->setVisible(trackVisible && m_render_history_as_line_property->getBool());

    // Get current track color
    Ogre::ColourValue trackColorWithFullAlpha = getColorFromId(trackId);
    Ogre::ColourValue trackColor = getColorFromId(trackId);
    trackColor.a *= m_commonProperties->alpha->getFloat();  // general alpha
    if (trackedPersonVisual->isOccluded)
      trackColor.a *= m_occlusion_alpha_property->getFloat();  // occlusion alpha
    if (trackedPersonVisual->isMissed)
      trackColor.a *= m_missed_alpha_property->getFloat();  // occlusion alpha

    // Update person color
    Ogre::ColourValue personColor = trackColor;
    if (!m_render_person_property->getBool())
      personColor.a = 0.0;

    if (trackedPersonVisual->personVisual)
    {
      trackedPersonVisual->personVisual->setColor(personColor);
    }

    // Update history size
    trackedPersonVisual->history.rset_capacity(m_history_length_property->getInt());

    // Update history color
    foreach (boost::shared_ptr<TrackedPersonHistoryEntry> historyEntry, trackedPersonVisual->history)
    {
      const double historyShapeDiameter = 0.1;
      Ogre::ColourValue historyColor = trackColorWithFullAlpha;
      historyColor.a *= m_commonProperties->alpha->getFloat();  // general alpha
      if (historyEntry->wasOccluded)
        historyColor.a *= m_occlusion_alpha_property->getFloat();
      if (isPersonHidden(trackId) || m_render_history_as_line_property->getBool())
        historyColor.a = 0;

      if (historyEntry->shape)
      {
        historyEntry->shape->setColor(historyColor);
        historyEntry->shape->setScale(shapeQuaternion *
                                      Ogre::Vector3(historyShapeDiameter, historyShapeDiameter, 0.05));
      }
    }

    if (trackedPersonVisual->historyLine)
    {  // history-as-line mode (as opposed to history-as-dots)
      Ogre::ColourValue historyColor = trackColorWithFullAlpha;
      historyColor.a *= m_commonProperties->alpha->getFloat();  // general alpha
      if (isPersonHidden(trackId))
        historyColor.a = 0;
      trackedPersonVisual->historyLine->setColor(historyColor.r, historyColor.g, historyColor.b, historyColor.a);
    }

    // Update text colors, font size and visibility
    const double personHeight = trackedPersonVisual->personVisual ? trackedPersonVisual->personVisual->getHeight() : 0;
    Ogre::ColourValue fontColor = m_commonProperties->font_color_style->getOptionInt() == FONT_COLOR_CONSTANT ?
                                      m_commonProperties->constant_font_color->getOgreColor() :
                                      trackColor;
    fontColor.a = m_commonProperties->alpha->getFloat();

    trackedPersonVisual->detectionIdText->setCharacterHeight(0.18 * m_commonProperties->font_scale->getFloat());
    trackedPersonVisual->detectionIdText->setVisible(!trackedPersonVisual->isOccluded &&
                                                     m_render_detection_ids_property->getBool() && trackVisible);
    trackedPersonVisual->detectionIdText->setColor(fontColor);
    trackedPersonVisual->detectionIdText->setPosition(
        Ogre::Vector3(0, 0, -trackedPersonVisual->detectionIdText->getCharacterHeight()));

    trackedPersonVisual->stateText->setCharacterHeight(0.18 * m_commonProperties->font_scale->getFloat());
    trackedPersonVisual->stateText->setVisible(m_render_track_state_property->getBool() && trackVisible);
    trackedPersonVisual->stateText->setColor(fontColor);
    trackedPersonVisual->stateText->setPosition(
        Ogre::Vector3(0, 0, personHeight + trackedPersonVisual->stateText->getCharacterHeight()));

    const double stateTextOffset =
        m_render_track_state_property->getBool() ? 1.2 * trackedPersonVisual->stateText->getCharacterHeight() : 0;
    trackedPersonVisual->idText->setCharacterHeight(0.25 * m_commonProperties->font_scale->getFloat());
    trackedPersonVisual->idText->setVisible(m_render_ids_property->getBool() && trackVisible);
    trackedPersonVisual->idText->setColor(fontColor);
    trackedPersonVisual->idText->setPosition(
        Ogre::Vector3(0, 0, personHeight + trackedPersonVisual->idText->getCharacterHeight() + stateTextOffset));

    // Update velocity arrow color
    double arrowAlpha = m_render_velocities_property->getBool() ? trackColor.a : 0.0;
    if (trackedPersonVisual->hasZeroVelocity)
      arrowAlpha = 0.0;
    trackedPersonVisual->velocityArrow->setColor(
        Ogre::ColourValue(trackColor.r, trackColor.g, trackColor.b, arrowAlpha));

    // Set color of covariance visualization
    Ogre::ColourValue covarianceColor = trackColor;
    if (!m_render_covariances_property->getBool())
      covarianceColor.a = 0.0;
    trackedPersonVisual->covarianceVisual->setColor(covarianceColor);
    trackedPersonVisual->covarianceVisual->setLineWidth(m_covariance_line_width_property->getFloat());
  }

  // Update global history visibility
  m_trackHistorySceneNode->setVisible(m_render_history_property->getBool());
}

// Set the rendering style (cylinders, meshes, ...) of tracked persons
void TrackedPersonsDisplay::personVisualTypeChanged()
{
  foreach (const track_map::value_type& entry, m_cachedTracks)
  {
    const boost::shared_ptr<TrackedPersonVisual>& trackedPersonVisual = entry.second;
    trackedPersonVisual->personVisual.reset();
    createPersonVisualIfRequired(trackedPersonVisual->sceneNode.get(), trackedPersonVisual->personVisual);
  }
  stylesChanged();
}

// This is our callback to handle an incoming message.
void TrackedPersonsDisplay::processMessage(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg)
{
  // Get transforms into fixed frame etc.
  if (!preprocessMessage(msg))
    return;

  // Transform from map/odometry frame into fixed frame, required to display track history if the fixed frame is not
  // really "fixed" (e.g. base_link)
  Ogre::Vector3 mapFramePosition;
  Ogre::Quaternion mapFrameOrientation;
  getContext()->getFrameManager()->getTransform(m_realFixedFrame, msg->header.stamp, mapFramePosition,
                                                mapFrameOrientation);
  Ogre::Matrix4 mapFrameTransform(mapFrameOrientation);
  mapFrameTransform.setTrans(mapFramePosition);

  // Transform required to fix orientation of any Cylinder shapes
  const Ogre::Quaternion shapeQuaternion(Ogre::Degree(90), Ogre::Vector3(1, 0, 0));
  stringstream ss;

  //
  // Iterate over all tracks in this message, see if we have a cached visual (then update it) or create a new one.
  //
  set<unsigned int> encounteredTrackIds;
  for (vector<spencer_tracking_msgs::TrackedPerson>::const_iterator trackedPersonIt = msg->tracks.begin();
       trackedPersonIt != msg->tracks.end(); ++trackedPersonIt)
  {
    boost::shared_ptr<TrackedPersonVisual> trackedPersonVisual;

    // See if we encountered this track ID before in this loop (means duplicate track ID)
    if (encounteredTrackIds.find(trackedPersonIt->track_id) != encounteredTrackIds.end())
    {
      ROS_ERROR_STREAM("spencer_tracking_msgs::TrackedPersons contains duplicate track ID "
                       << trackedPersonIt->track_id << "! Skipping duplicate track.");
      continue;
    }
    else
    {
      encounteredTrackIds.insert(trackedPersonIt->track_id);
    }

    // See if we have cached a track with this ID
    if (m_cachedTracks.find(trackedPersonIt->track_id) != m_cachedTracks.end())
    {
      trackedPersonVisual = m_cachedTracks[trackedPersonIt->track_id];
    }
    else
    {
      // Create a new visual representation of the tracked person
      trackedPersonVisual = boost::shared_ptr<TrackedPersonVisual>(new TrackedPersonVisual);
      m_cachedTracks[trackedPersonIt->track_id] = trackedPersonVisual;

      // This scene node is the parent of all visualization elements for the tracked person
      trackedPersonVisual->sceneNode = boost::shared_ptr<Ogre::SceneNode>(scene_node_->createChildSceneNode());
      trackedPersonVisual->historySceneNode =
          boost::shared_ptr<Ogre::SceneNode>(m_trackHistorySceneNode->createChildSceneNode());
      trackedPersonVisual->historyLineSceneNode =
          boost::shared_ptr<Ogre::SceneNode>(m_trackHistorySceneNode->createChildSceneNode());
    }

    // These values need to be remembered for later use in stylesChanged()
    if (trackedPersonIt->is_occluded && !trackedPersonIt->is_matched)
    {
      trackedPersonVisual->isOccluded = true;
      trackedPersonVisual->isMissed = false;
    }
    else if (!trackedPersonIt->is_occluded && !trackedPersonIt->is_matched)
    {
      trackedPersonVisual->isOccluded = false;
      trackedPersonVisual->isMissed = true;
    }
    else
    {
      trackedPersonVisual->isOccluded = false;
      trackedPersonVisual->isMissed = false;
    }

    trackedPersonVisual->isDeleted = false;
    trackedPersonVisual->numCyclesNotSeen = 0;

    Ogre::SceneNode* currentSceneNode = trackedPersonVisual->sceneNode.get();

    //
    // Person visualization
    //

    // Create new visual for the person itself, if needed
    boost::shared_ptr<PersonVisual>& personVisual = trackedPersonVisual->personVisual;
    createPersonVisualIfRequired(currentSceneNode, personVisual);

    const double personHeight = personVisual ? personVisual->getHeight() : 0;
    const double halfPersonHeight = personHeight / 2.0;

    //
    // Position of entire track
    //

    const Ogre::Matrix3 covXYZinTargetFrame = covarianceXYZIntoTargetFrame(trackedPersonIt->pose);
    setPoseOrientation(currentSceneNode, trackedPersonIt->pose, covXYZinTargetFrame, personHeight);

    //
    // Track history
    //

    Ogre::Vector3 newHistoryEntryPosition = mapFrameTransform.inverse() * currentSceneNode->getPosition();

    const float MIN_HISTORY_ENTRY_DISTANCE = m_history_min_point_distance_property->getFloat();  // in meters
    if ((trackedPersonVisual->positionOfLastHistoryEntry - newHistoryEntryPosition).length() >
        MIN_HISTORY_ENTRY_DISTANCE)
    {
      // General history
      boost::shared_ptr<TrackedPersonHistoryEntry> newHistoryEntry(new TrackedPersonHistoryEntry);
      newHistoryEntry->trackId = trackedPersonIt->track_id;
      newHistoryEntry->position = newHistoryEntryPosition;  // used by history lines (below) even if no shape is set
      newHistoryEntry->wasOccluded = trackedPersonIt->is_occluded;
      trackedPersonVisual->history.push_back(newHistoryEntry);

      // Always need to reset history line since history is like a queue, oldest element has to be removed but
      // BillboardLine doesn't offer that functionality
      trackedPersonVisual->historyLine.reset(
          new rviz::BillboardLine(context_->getSceneManager(), trackedPersonVisual->historyLineSceneNode.get()));

      if (m_render_history_as_line_property->getBool())
      {
        // History lines
        if (trackedPersonVisual->history.size() >= 2)
        {
          trackedPersonVisual->historyLine->setLineWidth(m_history_line_width_property->getFloat());
          trackedPersonVisual->historyLine->setMaxPointsPerLine(trackedPersonVisual->history.size());

          foreach (const boost::shared_ptr<TrackedPersonHistoryEntry>& historyEntry, trackedPersonVisual->history)
          {
            historyEntry->shape.reset();  // remove existing dot shapes, if any, for better performance
            trackedPersonVisual->historyLine->addPoint(historyEntry->position);
          }
        }
      }
      else
      {
        // History dots
        newHistoryEntry->shape = boost::shared_ptr<rviz::Shape>(new rviz::Shape(
            rviz::Shape::Cylinder, context_->getSceneManager(), trackedPersonVisual->historySceneNode.get()));
        newHistoryEntry->shape->setPosition(newHistoryEntryPosition);
        newHistoryEntry->shape->setOrientation(shapeQuaternion);
      }

      trackedPersonVisual->positionOfLastHistoryEntry = newHistoryEntryPosition;
    }

    //
    // Texts
    //
    {
      if (!trackedPersonVisual->idText)
      {
        trackedPersonVisual->idText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
        trackedPersonVisual->stateText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
        trackedPersonVisual->detectionIdText.reset(new TextNode(context_->getSceneManager(), currentSceneNode));
      }

      // Detection ID
      ss.str("");
      ss << "det " << trackedPersonIt->detection_id;
      trackedPersonVisual->detectionIdText->setCaption(ss.str());

      // Track state
      ss.str("");

      if (trackedPersonIt->is_occluded && !trackedPersonIt->is_matched)
        ss << "OCCLUDED";
      else if (!trackedPersonIt->is_occluded && !trackedPersonIt->is_matched)
        ss << "MISSED";
      else
        ss << "MATCHED";

      trackedPersonVisual->stateText->setCaption(ss.str());

      // Track ID
      ss.str("");
      ss << "Human " << trackedPersonIt->track_id;
      trackedPersonVisual->idText->setCaption(ss.str());
    }

    //
    // Velocity arrows
    //
    if (!trackedPersonVisual->velocityArrow)
    {
      trackedPersonVisual->velocityArrow.reset(new rviz::Arrow(context_->getSceneManager(), currentSceneNode));
    }

    // Update velocity arrow
    {
      const Ogre::Vector3 velocityVector = getVelocityVector(trackedPersonIt->twist);

      if (velocityVector.isZeroLength() || velocityVector.length() > 100 || velocityVector.isNaN())
      {
        if (!velocityVector.isZeroLength())
        {  // do not show warning for zero velocity
          ROS_WARN("Track %lu has suspicious velocity (%.1f m/s), not showing velocity vector!",
                   trackedPersonIt->track_id, velocityVector.length());
        }
      }
      else
      {
        const double personRadius = 0.2;
        const Ogre::Vector3 velocityArrowAttachPoint(personRadius, 0,
                                                     halfPersonHeight);  // relative to tracked person's scene node
        trackedPersonVisual->velocityArrow->setPosition(velocityArrowAttachPoint);
        trackedPersonVisual->velocityArrow->setOrientation(
            m_frameOrientation * currentSceneNode->getOrientation().Inverse() *
            Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo(velocityVector));

        const double shaftLength = velocityVector.length(), shaftDiameter = 0.05, headLength = 0.2, headDiameter = 0.2;
        trackedPersonVisual->velocityArrow->set(shaftLength, shaftDiameter, headLength, headDiameter);
        trackedPersonVisual->hasZeroVelocity = velocityVector.length() < 0.05;
      }

      boost::shared_ptr<MeshPersonVisual> meshPersonVisual =
          boost::dynamic_pointer_cast<MeshPersonVisual>(personVisual);
      if (meshPersonVisual)
      {
        meshPersonVisual->setWalkingSpeed(velocityVector.length());
      }
    }

    //
    // Covariance visualization
    //
    if (!trackedPersonVisual->covarianceVisual)
    {
      trackedPersonVisual->covarianceVisual.reset(
          new ProbabilityEllipseCovarianceVisual(context_->getSceneManager(), currentSceneNode));
    }

    // Update covariance ellipse
    {
      Ogre::Vector3 covarianceMean(0, 0, 0);  // zero mean because parent node is already centered at pose mean
      trackedPersonVisual->covarianceVisual->setOrientation(currentSceneNode->getOrientation().Inverse());
      trackedPersonVisual->covarianceVisual->setMeanCovariance(covarianceMean, covXYZinTargetFrame);
    }

  }  // end for loop over all tracked persons

  // Set all properties which can be dynamically in the GUI. This iterates over all tracks.
  stylesChanged();

  //
  // First hide, then delete old cached tracks which have not been seen for a while
  //
  set<unsigned int> trackIdsToDelete;
  for (map<unsigned int, boost::shared_ptr<TrackedPersonVisual> >::const_iterator cachedTrackIt =
           m_cachedTracks.begin();
       cachedTrackIt != m_cachedTracks.end(); ++cachedTrackIt)
  {
    if (encounteredTrackIds.end() == encounteredTrackIds.find(cachedTrackIt->first))
    {
      const boost::shared_ptr<TrackedPersonVisual>& trackedPersonVisual = cachedTrackIt->second;

      // Update state and visibility
      if (!trackedPersonVisual->isDeleted)
      {
        trackedPersonVisual->stateText->setCaption("DELETED");
        trackedPersonVisual->isDeleted = true;

        Ogre::Matrix4 lastObservedPose(trackedPersonVisual->sceneNode->getOrientation());
        lastObservedPose.setTrans(trackedPersonVisual->sceneNode->getPosition());
        trackedPersonVisual->lastObservedPose = mapFrameTransform.inverse() * lastObservedPose;
      }

      if (!m_show_deleted_property->getBool())
        trackedPersonVisual->sceneNode->setVisible(false);

      // Delete if too old
      if (++trackedPersonVisual->numCyclesNotSeen > m_delete_after_ncycles_property->getInt())
      {
        trackIdsToDelete.insert(cachedTrackIt->first);
      }
    }
  }

  for (set<unsigned int>::const_iterator setIt = trackIdsToDelete.begin(); setIt != trackIdsToDelete.end(); ++setIt)
  {
    m_cachedTracks.erase(*setIt);
  }

  //
  // Update status (shown in property pane)
  //
  ss.str("");
  ss << msg->tracks.size() << " tracks received";
  setStatusStd(rviz::StatusProperty::Ok, "Tracks", ss.str());
}

}  // end namespace spencer_tracking_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spencer_tracking_rviz_plugin::TrackedPersonsDisplay, rviz::Display)
