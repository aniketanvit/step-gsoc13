/* This file is part of StepCore library.
 * Copyright (C) 2013 Aniket Anvit <seeanvit@gmail.com>
 * 
 * StepCore library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * StepCore library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with StepCore; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef PULLEYCORDGRAPHICS_H
#define PULLEYCORDGRAPHICS_H

#include "worldgraphics.h"
#include "stepcore/pulleycord.h"

class PulleyCordCreator : public ItemCreator
{
public:
  PulleyCordCreator(const QString& className, WorldModel* worldModel, WorldScene* worldScene):
                  ItemCreator(className, worldModel, worldScene){}

  bool sceneEvent(QEvent* event);                
};

class PulleyCordHandlerGraphicsItem : public WorldGraphicsItem
{
public:
  PulleyCordHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel,
		    QGraphicsItem* parent, int num) ;
		    
  void viewScaleChanged();
  void worldDataChanged(bool dynamicOnly);
protected:
  void mouseSetPos(const QPointF& pos, const QPointF& diff, MovingState movingState);
  int _num;
  bool _moving;
  
};

class PulleyCordGraphicsItem : public WorldGraphicsItem
{
public:
  
  PulleyCordGraphicsItem(StepCore::Item* item, WorldModel* worldModel);
  
  QPainterPath shape() const;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
  void viewScaleChanged();
  void stateChanged();
  void worldDataChanged(bool dynamicOnly);
  
protected:
  StepCore::PulleyCord* pulleyCord() const {
    return static_cast<StepCore::PulleyCord*>(_item); }  
  QPainterPath _painterPath;
  double _rnorm;
  double _rscale;
  double _radius;
  
  PulleyCordHandlerGraphicsItem* _handler1;
  PulleyCordHandlerGraphicsItem* _handler2;
  friend class PulleyCordCreator;
};

#endif



