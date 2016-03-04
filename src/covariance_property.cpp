/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "covariance_property.h"
#include "covariance_visual.h"

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>

#include <QColor>

using namespace rviz;

namespace rviz_plugin_covariance
{

CovarianceProperty::CovarianceProperty( const QString& name,
                            bool default_value,
                            const QString& description,
                            Property* parent,
                            const char *changed_slot,
                            QObject* receiver )
  // NOTE: changed_slot and receiver aren't passed to BoolProperty here, but initialized at the end of this constructor
  : BoolProperty( name, default_value, description, parent )
{

  position_property_ = new BoolProperty( "Position", true,
                                       "Whether or not to show the position part of covariances",
                                       this, SLOT( updateVisibility() ));
  position_property_->setDisableChildrenIfFalse( true );

  position_color_property_ = new ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the covariance ellipse.",
                                             position_property_, SLOT( updateColorAndAlphaAndScale() ), this );
  
  position_alpha_property_ = new FloatProperty( "Alpha", 0.3f,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             position_property_, SLOT( updateColorAndAlphaAndScale() ), this );
  position_alpha_property_->setMin( 0 );
  position_alpha_property_->setMax( 1 );
  
  position_scale_property_ = new FloatProperty( "Scale", 1.0f,
                                             "Scale factor to be applied to covariance ellipse",
                                             position_property_, SLOT( updateColorAndAlphaAndScale() ), this );

  orientation_property_ = new BoolProperty( "Orientation", true,
                                          "Whether or not to show the orientation part of covariances",
                                          this, SLOT( updateVisibility() ));
  orientation_property_->setDisableChildrenIfFalse( true );

  orientation_frame_property_ = new EnumProperty( "Frame", "Rotating", "The frame used to display the orientation covariance.",
                                      orientation_property_, SLOT( updateOrientationFrame() ), this );
  orientation_frame_property_->addOption( "Rotating", Rotating );
  orientation_frame_property_->addOption( "Static", Static );

  orientation_color_property_ = new ColorProperty( "Color", QColor( 255, 255, 127 ),
                                             "Color to draw the covariance ellipse.",
                                             orientation_property_, SLOT( updateColorAndAlphaAndScale() ), this );
  
  orientation_alpha_property_ = new FloatProperty( "Alpha", 0.5f,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             orientation_property_, SLOT( updateColorAndAlphaAndScale() ), this );
  orientation_alpha_property_->setMin( 0 );
  orientation_alpha_property_->setMax( 1 );
  
  orientation_scale_property_ = new FloatProperty( "Scale", 1.0f,
                                             "Scale factor to be applied to covariance ellipse",
                                             orientation_property_, SLOT( updateColorAndAlphaAndScale() ), this );

  connect(this, SIGNAL( changed() ), this, SLOT( updateVisibility() ));

  // Connect changed() signal here instead of doing it through the initialization of BoolProperty().
  // We do this here to make changed_slot be called _after_ updateVisibility()
  if(changed_slot && (parent || receiver))
  {
    if(receiver)
      connect(this, SIGNAL( changed() ), receiver, changed_slot);
    else
      connect(this, SIGNAL( changed() ), parent, changed_slot);
  }

  setDisableChildrenIfFalse( true );
}

CovarianceProperty::~CovarianceProperty()
{
}

void CovarianceProperty::updateColorAndAlphaAndScale()
{
  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
    updateColorAndAlphaAndScale(*it_cov);
}

void CovarianceProperty::updateColorAndAlphaAndScale(const CovarianceVisualPtr& visual)
{
  QColor pos_color = position_color_property_->getColor();
  float pos_alpha = position_alpha_property_->getFloat();
  float pos_scale = position_scale_property_->getFloat();

  QColor ori_color = orientation_color_property_->getColor();
  float ori_alpha = orientation_alpha_property_->getFloat();
  float ori_scale = orientation_scale_property_->getFloat();

  visual->setPositionColor( pos_color.redF(), pos_color.greenF(), pos_color.blueF(), pos_alpha );
  visual->setPositionScale( pos_scale );
  visual->setOrientationColor( ori_color.redF(), ori_color.greenF(), ori_color.blueF(), ori_alpha );
  visual->setOrientationScale( ori_scale );
}

void CovarianceProperty::updateVisibility()
{
  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
    updateVisibility(*it_cov);
}

void CovarianceProperty::updateVisibility(const CovarianceVisualPtr& visual)
{
  bool show_covariance = getBool();
  if( !show_covariance )
  {
    visual->setVisible( false );
  }
  else
  {
    bool show_position_covariance = position_property_->getBool();;
    bool show_orientation_covariance = orientation_property_->getBool();
    visual->setPositionVisible( show_position_covariance );
    visual->setOrientationVisible( show_orientation_covariance );
  }
}

void CovarianceProperty::updateOrientationFrame()
{
  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
    updateOrientationFrame(*it_cov);
}

void CovarianceProperty::updateOrientationFrame(const CovarianceVisualPtr& visual)
{
  bool use_rotating_frame = ( orientation_frame_property_->getOptionInt() == Rotating );
  visual->setRotatingFrame( use_rotating_frame );
}

void CovarianceProperty::pushBackVisual( const CovarianceVisualPtr& visual )
{
  updateVisibility(visual);
  updateOrientationFrame(visual);
  updateColorAndAlphaAndScale(visual);
  covariances_.push_back(visual);
}

void CovarianceProperty::popFrontVisual()
{
  covariances_.pop_front();
}

void CovarianceProperty::clearVisual()
{
  covariances_.clear();
}

size_t CovarianceProperty::sizeVisual()
{
  return covariances_.size();
}

} // end namespace rviz_plugin_covariance
