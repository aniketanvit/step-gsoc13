/* This file is part of Step.
 * Copyright (C) 2013 Aniket Anvit <seeanvit@gmail.com>
 * 
 * Step is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * Step is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Step; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef FRICTIONFORCE_H
#define FRICTIONFORCE_H

#include "stepcore/world.h"
#include "worldmodel.h"
#include "worldscene.h"
#include <kaction.h>
#include "worldgraphics.h"

namespace Ui {
 class WidgetCreateFrictionForce;
}

class FrictionForceKDialog;
class FrictionForceMenuHandler : public QObject
{
  Q_OBJECT
  
public:
  
  FrictionForceMenuHandler(WorldModel* worldModel, QObject* parent= 0) :
  QObject(parent), _worldModel(worldModel){}
  
public:  
  void arrangeWidgets();
  
protected slots :
  bool fillValues();

protected:
  Ui::WidgetCreateFrictionForce*  _createFrictionForceUi;
  FrictionForceKDialog*           _createFrictionForceDialog;
  
  WorldModel* _worldModel;
  friend class FrictionForceKDialog;
 // friend class FrictionForceCreator;
};

#endif