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

#ifndef STEP_SPRINGGRAPHICS_H
#define STEP_SPRINGGRAPHICS_H

#include "worldgraphics.h"
#include <stepcore/spring.h>

class SpringCreator: public ItemCreator
{
public:
    SpringCreator(const QString& className, WorldModel* worldModel, WorldScene* worldScene)
                        : ItemCreator(className, worldModel, worldScene) {}
    bool sceneEvent(QEvent* event);
    void start();
};

class SpringHandlerGraphicsItem: public WorldGraphicsItem {
public:
    SpringHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel,
                                QGraphicsItem* parent, int num);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void viewScaleChanged();
    void worldDataChanged(bool);

protected:
    void mouseSetPos(const QPointF&, const QPointF& pos, const QPointF& diff, MovingState movingState);

    int _num;
    bool _moving;
};

class SpringGraphicsItem: public WorldGraphicsItem {
public:
    SpringGraphicsItem(StepCore::Item* item, WorldModel* worldModel);

    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void viewScaleChanged();
    void stateChanged();
    void worldDataChanged(bool);

protected:
    static void tryAttach(StepCore::Item* item, WorldScene* worldScene, const QPointF& pos, int num);

    void mouseSetPos(const QPointF&, const QPointF& pos, const QPointF& diff, MovingState);
    StepCore::Spring* spring() const {
        return static_cast<StepCore::Spring*>(_item); }

    QPainterPath _painterPath;
    double _rnorm;
    double _rscale;
    double _radius;

    SpringHandlerGraphicsItem* _handler1;
    SpringHandlerGraphicsItem* _handler2;

    static const int RADIUS = 6;

    friend class SpringCreator;
};

#endif

