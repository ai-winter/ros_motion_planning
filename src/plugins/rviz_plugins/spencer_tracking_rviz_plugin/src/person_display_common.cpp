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
#include "person_display_common.h"
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#endif
#define foreach BOOST_FOREACH


namespace spencer_tracking_rviz_plugin
{
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
PersonDisplayCommonProperties::PersonDisplayCommonProperties(rviz::Display* display, StylesChangedSubscriber* stylesChangedSubscriber)
    : m_display(display), m_stylesChangedSubscriber(stylesChangedSubscriber)
{
    style = new rviz::EnumProperty( "Style", "Cylinders", "Rendering mode to use, in order of computational complexity.", m_display, SLOT(stylesChanged()), this );
    style->addOption( "Simple", STYLE_SIMPLE );
    style->addOption( "Cylinders", STYLE_CYLINDER );
    style->addOption( "Person meshes", STYLE_PERSON_MESHES );
    style->addOption( "Bounding boxes", STYLE_BOUNDING_BOXES );
    style->addOption( "Crosshairs", STYLE_CROSSHAIRS );

    color_transform = new rviz::EnumProperty( "Color transform", "Rainbow", "How to color the tracked persons", m_display, SLOT(stylesChanged()), this );
    color_transform->addOption( "SRL Tracking Colors", COLORS_SRL );
    color_transform->addOption( "Alternative SRL colors", COLORS_SRL_ALTERNATIVE );
    color_transform->addOption( "Rainbow", COLORS_RAINBOW );
    color_transform->addOption( "Rainbow + B/W", COLORS_RAINBOW_BW );
    color_transform->addOption( "Flat", COLORS_FLAT );
    color_transform->addOption( "Vintage", COLORS_VINTAGE );
    color_transform->addOption( "Constant color", COLORS_CONSTANT );

    constant_color = new rviz::ColorProperty("Color", QColor( 130, 130, 130 ), "Color for tracked persons if using constant color transform.", m_display, SLOT(stylesChanged()), this );

    color_map_offset = new rviz::IntProperty( "Color map offset", 0, "By how many indices to shift the offset in the color map (useful if not happy with the current colors)", m_display, SLOT(stylesChanged()), this);
    color_map_offset->setMin( 0 );

    alpha = new rviz::FloatProperty( "Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", m_display, SLOT(stylesChanged()), this);
    alpha->setMin( 0.0 );
    alpha->setMax( 1.0 );

    line_width = new rviz::FloatProperty( "Line width", 0.05, "Line width for person visual", style, SLOT(stylesChanged()), this);
    line_width->setMin( 0.0 );
    line_width->setMax( 1.0 );
    
    scaling_factor = new rviz::FloatProperty( "Scaling factor", 1.0, "Scaling factor for person visual", style);
    scaling_factor->setMin( 0.0 );
    scaling_factor->setMax( 100.0 );

    font_color_style = new rviz::EnumProperty( "Font color style", "Same color", "Which type of font coloring to use", m_display, SLOT(stylesChanged()), this );
    font_color_style->addOption( "Same color", FONT_COLOR_FROM_PERSON );
    font_color_style->addOption( "Constant color", FONT_COLOR_CONSTANT );

    constant_font_color = new rviz::ColorProperty("Font color", QColor( 255, 255, 255 ), "Font color if using a constant color", m_display, SLOT(stylesChanged()), this );

    font_scale = new rviz::FloatProperty( "Font scale", 2.0, "Larger values mean bigger font", m_display);
    font_scale->setMin( 0.0 );

    z_offset = new rviz::FloatProperty( "Z offset", 0.0, "Offset of all visualizations on the z (height) axis", m_display, SLOT(stylesChanged()), this);

    use_actual_z_position = new rviz::BoolProperty( "Use Z position from message", false, "Use Z position from message (otherwise place above ground plane)", z_offset, SLOT(stylesChanged()), this);

    m_excluded_person_ids_property = new rviz::StringProperty( "Excluded person IDs", "", "Comma-separated list of person IDs whose visualization should be hidden", m_display, SLOT(stylesChanged()), this );
    m_included_person_ids_property = new rviz::StringProperty( "Included person IDs", "", "Comma-separated list of person IDs whose visualization should be visible. Overrides excluded IDs.", m_display, SLOT(stylesChanged()), this );

    hideIrrelevantProperties();
}

void PersonDisplayCommonProperties::hideIrrelevantProperties()
{
    constant_color->setHidden(color_transform->getOptionInt() != COLORS_CONSTANT);
    color_map_offset->setHidden(color_transform->getOptionInt() == COLORS_CONSTANT);
    constant_font_color->setHidden(font_color_style->getOptionInt() != FONT_COLOR_CONSTANT);

    line_width->setHidden(style->getOptionInt() != STYLE_BOUNDING_BOXES && style->getOptionInt() != STYLE_CROSSHAIRS);
}

// Callback for any changed style
void PersonDisplayCommonProperties::stylesChanged()
{
    hideIrrelevantProperties();

    // Update list of person IDs that shall be hidden or visible
    m_excludedPersonIDs.clear();
    {
        string personIDString = m_excluded_person_ids_property->getStdString();
        char_separator<char> separator(",");
        tokenizer< char_separator<char> > tokens(personIDString, separator);
        foreach(const string& token, tokens) {
         try { m_excludedPersonIDs.insert(lexical_cast<person_id>(token)); }
         catch(bad_lexical_cast &) {}
        }
    }
    m_includedPersonIDs.clear();
    {
        string personIDString = m_included_person_ids_property->getStdString();
        char_separator<char> separator(",");
        tokenizer< char_separator<char> > tokens(personIDString, separator);
        foreach(const string& token, tokens) {
            try { m_includedPersonIDs.insert(lexical_cast<person_id>(token)); }
            catch(bad_lexical_cast &) {}
        }
    }

    // Relay change to other subscribers
    m_stylesChangedSubscriber->stylesChanged();
}


} // end namespace spencer_tracking_rviz_plugin
