/* This file is part of Step.
   Copyright (C) 2007 Vladimir Kuznetsov <ks.vladimir@gmail.com>

   Step is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   Step is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Step; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef STEP_MOTORGRAPHICS_H
#define STEP_MOTORGRAPHICS_H

#include "worldgraphics.h"
#include <QGraphicsTextItem>
#include <QAbstractItemModel>
#include <QWidget>
#include <KComboBox>
#include <limits.h>

class KPlotWidget;
class KPlotObject;
class KAction;
class KDialog;
class QSlider;
class QLabel;

namespace StepCore {
    class LinearMotor;
    class CircularMotor;
}

class LinearMotorCreator: public ItemCreator
{
public:
    LinearMotorCreator(const QString& className, WorldModel* worldModel, WorldScene* worldScene)
                        : ItemCreator(className, worldModel, worldScene) {}
    bool sceneEvent(QEvent* event);

protected:
    void tryAttach(const QPointF& pos);
};

class LinearMotorGraphicsItem: public WorldGraphicsItem
{
public:
    LinearMotorGraphicsItem(StepCore::Item* item, WorldModel* worldModel);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    void viewScaleChanged();
    void worldDataChanged(bool dynamicOnly);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
protected:
    StepCore::LinearMotor* motor() const;
    QPainterPath _path;
    ArrowHandlerGraphicsItem* _velocityHandler;
    bool      _moving;
    static const int RADIUS = 5;

};
/////////////////////////////////////////////////////////////////////////////////////////////

class CircularMotorCreator: public ItemCreator
{
public:
    CircularMotorCreator(const QString& className, WorldModel* worldModel, WorldScene* worldScene)
                        : ItemCreator(className, worldModel, worldScene) {}
    bool sceneEvent(QEvent* event);

protected:
    void tryAttach(const QPointF& pos);
};

class CircularMotorGraphicsItem: public WorldGraphicsItem
{
public:
    CircularMotorGraphicsItem(StepCore::Item* item, WorldModel* worldModel);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    void viewScaleChanged();
    void worldDataChanged(bool dynamicOnly);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
protected:
    StepCore::CircularMotor* motor() const;
    QPainterPath _path;
    bool      _moving;//
    static const int RADIUS = 5;

};

#endif
