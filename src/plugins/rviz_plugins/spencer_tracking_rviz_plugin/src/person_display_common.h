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

#ifndef PERSON_DISPLAY_COMMON_H
#define PERSON_DISPLAY_COMMON_H

#ifndef Q_MOC_RUN
#include <map>
#include <set>
#include <boost/circular_buffer.hpp>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <geometry_msgs/Twist.h>
#include <rviz/message_filter_display.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include "visuals/person_visual.h"
#include "visuals/text_node.h"
#include "visuals/covariance_visual.h"
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>
#endif







using namespace std;
using namespace boost;

namespace spencer_tracking_rviz_plugin
{
    typedef unsigned int person_id;

    /// Visualization style for a person
    enum Styles {
        STYLE_SIMPLE,
        STYLE_CYLINDER,
        STYLE_PERSON_MESHES,
        STYLE_BOUNDING_BOXES,
        STYLE_CROSSHAIRS
    };

    /// How to color persons
    enum ColorTransforms {
        COLORS_SRL,
        COLORS_SRL_ALTERNATIVE,
        COLORS_RAINBOW,
        COLORS_RAINBOW_BW,
        COLORS_FLAT,
        COLORS_VINTAGE,
        COLORS_CONSTANT,
    };

    /// Which font colors to use
    enum FontColorStyle {
        FONT_COLOR_FROM_PERSON,
        FONT_COLOR_CONSTANT
    };

    /// Subclasses of PersonDisplayCommon can override stylesChanged() to get notified when one of the properties in PersonDisplayCommonProperties has changed.
    class StylesChangedSubscriber {
    public:
        virtual ~StylesChangedSubscriber() {}
        virtual void stylesChanged() {}
    };

    /// Common properties shared by multiple displays.
    class PersonDisplayCommonProperties : public QObject {
    Q_OBJECT
    public:
        PersonDisplayCommonProperties(rviz::Display* display, StylesChangedSubscriber* stylesChangedSubscriber);

        // User-editable property variables.
        rviz::EnumProperty* style;
        rviz::EnumProperty* color_transform;
        rviz::IntProperty* color_map_offset;

        rviz::ColorProperty* constant_color;
        rviz::FloatProperty* alpha;

        rviz::FloatProperty* line_width;
        rviz::FloatProperty* z_offset;
        rviz::FloatProperty* scaling_factor;

        rviz::BoolProperty* use_actual_z_position;

        rviz::EnumProperty*  font_color_style;
        rviz::ColorProperty* constant_font_color;
        rviz::FloatProperty* font_scale;

        rviz::StringProperty* m_excluded_person_ids_property;
        rviz::StringProperty* m_included_person_ids_property;

        /// These sets get updated automatically whenever the corresponding properties are updated.
        set<person_id> m_excludedPersonIDs, m_includedPersonIDs;

    private:
        rviz::Display* m_display;
        StylesChangedSubscriber* m_stylesChangedSubscriber;

        void hideIrrelevantProperties();

    private Q_SLOTS:
        void stylesChanged();
    };

    /// A display with common properties that are shared by multiple specializations.
    template<typename MessageType>
    class PersonDisplayCommon: public rviz::MessageFilterDisplay<MessageType>, public StylesChangedSubscriber
    {
    public:
        /// Constructor.  pluginlib::ClassLoader creates instances by calling
        /// the default constructor, so make sure you have one.
        PersonDisplayCommon() : m_commonProperties(0), m_veryLargeVariance(99999) {}
        virtual ~PersonDisplayCommon() {}

        /// Overrides base class method
        virtual void onInitialize()
        {
            rviz::MessageFilterDisplay<MessageType>::onInitialize();
            m_commonProperties = new PersonDisplayCommonProperties(this, this);
        }

    protected:
        /// Common message processing. This method needs to be called by derived classes
        bool preprocessMessage(const typename MessageType::ConstPtr& msg)
        {
            // Here we call the rviz::FrameManager to get the transform from the
            // fixed frame to the frame in the header of this Imu message.  If
            // it fails, we can't do anything else so we return.
            if (!getContext()->getFrameManager()->getTransform(msg->header, m_framePosition, m_frameOrientation)) {
                ROS_ERROR_THROTTLE(5.0, "Error transforming from frame '%s' into fixed frame!", msg->header.frame_id.c_str());
                return false;
            }

            m_frameOrientation.ToRotationMatrix(m_frameRotationMatrix);
            return true;
        }

        /// Create a visual representation of the person itself, if not set yet
        void createPersonVisualIfRequired(Ogre::SceneNode* sceneNode, boost::shared_ptr<PersonVisual> &personVisual)
        {
            if (!personVisual) {
                PersonVisualDefaultArgs defaultArgs(getContext()->getSceneManager(), sceneNode);
                PersonVisual* newPersonVisual = 0;

                if (m_commonProperties->style->getOptionInt() == STYLE_CYLINDER) newPersonVisual = new CylinderPersonVisual(defaultArgs);
                if (m_commonProperties->style->getOptionInt() == STYLE_PERSON_MESHES) newPersonVisual = new MeshPersonVisual(defaultArgs);
                if (m_commonProperties->style->getOptionInt() == STYLE_BOUNDING_BOXES) newPersonVisual = new BoundingBoxPersonVisual(defaultArgs);
                if (m_commonProperties->style->getOptionInt() == STYLE_CROSSHAIRS) newPersonVisual = new CrosshairPersonVisual(defaultArgs);
                personVisual.reset(newPersonVisual);
            }

            // Update position of the person visual
            if (personVisual) {
                personVisual->setPosition(Ogre::Vector3(0,0, personVisual->getHeight() * 0.5));
            }
        }

        /// Applies common styles which apply to person visuals, such as line width etc.
        void applyCommonStyles(boost::shared_ptr<PersonVisual> &personVisual) {
            if(!personVisual) return;

            // Set line width of wireframe visualization
            HasLineWidth* hasLineWidth = dynamic_cast<HasLineWidth*>(personVisual.get());
            if(hasLineWidth) {
                hasLineWidth->setLineWidth(m_commonProperties->line_width->getFloat());
            }

            // Set scaling factor
            personVisual->setScalingFactor(m_commonProperties->scaling_factor->getFloat());
        }

        // Builds velocity vector for a person from a twist message
        Ogre::Vector3 getVelocityVector(const geometry_msgs::TwistWithCovariance& twist) {
            const double zVelocityVariance = twist.covariance[2 * 6 + 2];
            const double zVelocity = (isnan(zVelocityVariance) || isinf(zVelocityVariance)) ? 0.0 : twist.twist.linear.z;
            return Ogre::Vector3(twist.twist.linear.x, twist.twist.linear.y, zVelocity);
        }

        /// Returns true if all xyz rotation variances are finite
        bool hasValidOrientation(const geometry_msgs::PoseWithCovariance& pose)
        {
            // Check if quaternion has not been initialized, then it's invalid (all-zero elements)
            if(pose.pose.orientation.x == 0 && pose.pose.orientation.y == 0 && pose.pose.orientation.z == 0 && pose.pose.orientation.w == 0) return false;

            // According to ROS conventions, orientation covariance is always fixed-frame
            // so no transform necessary!
            const double xRotVariance = pose.covariance[3 * 6 + 3];
            const double yRotVariance = pose.covariance[4 * 6 + 4];
            const double zRotVariance = pose.covariance[5 * 6 + 5];
            // Using logical OR instead of AND here because the orientation is given as a quaternion, instead of independent
            // RPY angles. We assume if at least one RPY angle is valid (according to its covariance), the other angles are set to a
            // reasonable default (as part of the quaternion) if their variance is non-finite.
            return xRotVariance < m_veryLargeVariance || yRotVariance < m_veryLargeVariance || zRotVariance < m_veryLargeVariance;
        }

        /// Rotate the position (xyz) part of a pose covariance matrix into the fixed frame used for visualization
        /// The covariance matrix needs to be transformed from the source (e.g. sensor) into the target (e.g. odometry) frame
        /// This is mainly be a problem if the sensor is rotated vertically compared to the odometry frame, so that axes get swapped
        Ogre::Matrix3 covarianceXYZIntoTargetFrame(const geometry_msgs::PoseWithCovariance& pose) {
            Ogre::Matrix3 xyzcov;
            for(int row = 0; row < 3; row++) for(int col = 0; col < 3; col++) xyzcov[row][col] = pose.covariance[row*6 + col]; // 6 = dimension of ROS covariance matrix
            if(!isfinite(xyzcov.Determinant())) ROS_WARN_STREAM("Covariance matrix supplied to covarianceXYZIntoTargetFrame() contains non-finite elements: " << xyzcov);
            return m_frameRotationMatrix * xyzcov * m_frameRotationMatrix.Transpose(); // cov(AX + a) = A cov(X) A^T
        }

        /// Set pose and orientation of person visual
        void setPoseOrientation(Ogre::SceneNode* sceneNode, const geometry_msgs::PoseWithCovariance& pose, const Ogre::Matrix3& covXYZinTargetFrame, double personVisualHeight)
        {
            const geometry_msgs::Point& position = pose.pose.position;
            const geometry_msgs::Quaternion& orientation = pose.pose.orientation;

            Ogre::Matrix4 transform(m_frameOrientation);
            transform.setTrans(m_framePosition);

            Ogre::Vector3 originalPosition(position.x, position.y, position.z);
            if(!isfinite(originalPosition.x) || !isfinite(originalPosition.y) || !isfinite(originalPosition.z)) {
                ROS_WARN("Detected or tracked person has non-finite position! Something is wrong!");
                return;
            }

            Ogre::Vector3 positionInTargetFrame = transform * originalPosition;

            if(hasValidOrientation(pose)) {
                Ogre::Quaternion detectionOrientation(orientation.w, orientation.x, orientation.y, orientation.z);
                detectionOrientation.FromAngleAxis(detectionOrientation.getRoll(), Ogre::Vector3(0,0,1)); // only use yaw angle, ignore roll and pitch
                sceneNode->setOrientation(m_frameOrientation * detectionOrientation);
            }
            else {
                Ogre::Quaternion rotateTowardsCamera;
                rotateTowardsCamera.FromAngleAxis(Ogre::Degree(180), Ogre::Vector3(0,0,1));
                sceneNode->setOrientation(rotateTowardsCamera);
            }

            const double zVariance = covXYZinTargetFrame[2][2];
            bool useActualZPosition = m_commonProperties->use_actual_z_position->getBool() && isfinite(zVariance) && zVariance >= 0 && zVariance < m_veryLargeVariance;

            positionInTargetFrame.z = useActualZPosition ? positionInTargetFrame.z - personVisualHeight/2.0: 0.0;
            positionInTargetFrame.z += m_commonProperties->z_offset->getFloat();

            sceneNode->setPosition(positionInTargetFrame);
        }

        /// Get a color based upon track / detection ID.
        Ogre::ColourValue getColorFromId(unsigned int object_id)
        {
            Ogre::ColourValue color;
            const int colorScheme = m_commonProperties->color_transform->getOptionInt();
            object_id += max(0, m_commonProperties->color_map_offset->getInt());

            if(colorScheme == COLORS_SRL || colorScheme == COLORS_SRL_ALTERNATIVE)
            {
                // SRL People Tracker colors
                const size_t NUM_SRL_COLORS = 6, NUM_SRL_COLOR_SHADES = 4, NUM_SRL_TOTAL_COLORS = NUM_SRL_COLORS * NUM_SRL_COLOR_SHADES;
                const unsigned int spencer_colors[NUM_SRL_TOTAL_COLORS] = {
                    0xC00000, 0xFF0000, 0xFF5050, 0xFFA0A0, // red
                    0x00C000, 0x00FF00, 0x50FF50, 0xA0FFA0, // green
                    0x0000C0, 0x0000FF, 0x5050FF, 0xA0A0FF, // blue
                    0xF20A86, 0xFF00FF, 0xFF50FF, 0xFFA0FF, // magenta
                    0x00C0C0, 0x00FFFF, 0x50FFFF, 0xA0FFFF, // cyan
                    0xF5A316, 0xFFFF00, 0xFFFF50, 0xFFFFA0  // yellow
                };

                unsigned int rgb = 0;
                const unsigned int tableId = object_id % NUM_SRL_TOTAL_COLORS;
                if(m_commonProperties->color_transform->getOptionInt() == COLORS_SRL) {
                    // Colors in original order (first vary shade, then vary color)
                    rgb = spencer_colors[tableId];
                }
                else if(m_commonProperties->color_transform->getOptionInt() == COLORS_SRL_ALTERNATIVE) {
                    // Colors in alternative order (first vary color, then vary shade)
                    unsigned int shadeIndex = tableId / NUM_SRL_COLORS;
                    unsigned int colorIndex = tableId % NUM_SRL_COLORS;
                    rgb = spencer_colors[colorIndex * NUM_SRL_COLOR_SHADES + shadeIndex];
                }

                float r = ((rgb >> 16) & 0xff) / 255.0f,
                       g = ((rgb >> 8)  & 0xff) / 255.0f,
                       b = ((rgb >> 0)  & 0xff) / 255.0f;

                color = Ogre::ColourValue(r, g, b, 1.0);
            }
            else if(colorScheme == COLORS_RAINBOW || colorScheme == COLORS_RAINBOW_BW)
            {
                const size_t NUM_COLOR = 10, NUM_BW = 4;
                const unsigned int rainbow_colors[NUM_COLOR + NUM_BW] = {
                    0xaf1f90, 0x000846, 0x00468a, 0x00953d, 0xb2c908, 0xfcd22a, 0xffa800, 0xff4500, 0xe0000b, 0xb22222,
                    0xffffff, 0xb8b8b8, 0x555555, 0x000000
                };

                color.setAsARGB(rainbow_colors[object_id % (colorScheme == COLORS_RAINBOW ? NUM_COLOR : (NUM_COLOR+NUM_BW))]);
            }
            else if(colorScheme == COLORS_FLAT)
            {
                const size_t NUM_COLOR = 10;
                const unsigned int flat_colors[NUM_COLOR] = {
                    0x990033, 0xa477c4, 0x3498db, 0x1abc9c, 0x55e08f, 0xfff054, 0xef5523, 0xfe374a, 0xbaa891, 0xad5f43
                };

                color.setAsARGB(flat_colors[object_id % NUM_COLOR]);
            }
            else if(colorScheme == COLORS_VINTAGE)
            {
                const size_t NUM_COLOR = 10;
                const unsigned int vintage_colors[NUM_COLOR] = {
                    0xd05e56, 0x797d88, 0x448d7a, 0xa5d1cd, 0x88a764, 0xebe18c, 0xd8a027, 0xffcc66, 0xdc3f1c, 0xff9966
                };

                color.setAsARGB(vintage_colors[object_id % NUM_COLOR]);
            }
            else
            {
                // Constant color for all tracks
                color = m_commonProperties->constant_color->getOgreColor();
            }

            color.a = 1.0f; // force full opacity
            return color;
        }

        /// Checks if a person shall be hidden (can be set using include/exclude person ID properties in GUI)
        bool isPersonHidden(person_id personId)
        {
            bool isIncluded = m_commonProperties->m_includedPersonIDs.find(personId) != m_commonProperties->m_includedPersonIDs.end();
            if(isIncluded) return false;
            if(!m_commonProperties->m_includedPersonIDs.empty()) return true;
            return m_commonProperties->m_excludedPersonIDs.find(personId) != m_commonProperties->m_excludedPersonIDs.end();
        }

        /// Must be implemented by derived classes because MOC doesn't work in templates
        virtual rviz::DisplayContext* getContext() = 0;

        /// Common properties for the displays in this plugin
        PersonDisplayCommonProperties* m_commonProperties;

    protected:
        Ogre::Quaternion m_frameOrientation;
        Ogre::Matrix3 m_frameRotationMatrix;
        Ogre::Vector3 m_framePosition;
        const double m_veryLargeVariance;
    };
} // end namespace spencer_tracking_rviz_plugin


#endif // PERSON_DISPLAY_COMMON_H
