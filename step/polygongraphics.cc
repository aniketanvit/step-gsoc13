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

#include "polygongraphics.h"

#include <stepcore/constants.h>
#include <stepcore/types.h>
#include "worldmodel.h"
#include "worldfactory.h"
#include <QItemSelectionModel>
#include <QEvent>
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>
#include <QPainter>
#include <KLocale>

void PolygonCreator::fixCenterOfMass()
{
    StepCore::Polygon::VertexList v = static_cast<StepCore::Polygon*>(_item)->vertexes();
    StepCore::Vector2d position = static_cast<StepCore::Polygon*>(_item)->position();

    StepCore::Vector2d center(0, 0);
    double area_i, area = 0;
    unsigned int i;

    if(v.size() == 1) center = v[0];
    else {
        if(v.size() > 2) {
            for(i=0; i+1<v.size(); ++i) {
                area_i = (v[i][0]*v[i+1][1] - v[i][1]*v[i+1][0]) / 2;
                center += (v[i] + v[i+1]) * (area_i/3);
                area += area_i;
            }
            area_i = (v[i][0]*v[0][1] - v[i][1]*v[0][0]) / 2;
            center += (v[i] + v[0]) * (area_i/3);
            area += area_i;
        }

        if(area == 0) { // all vertexes on one line
            center.setZero();
            for(i=0; i+1<v.size(); ++i) {
                area_i = (v[i+1] - v[i]).norm();
                center += (v[i] + v[i+1]) * (area_i/2);
                area += area_i;
            }
        }

        if(area == 0) center = v[0]; // all vertexes are at one point
        else center /= area;
    }

    for(i=0; i<v.size(); ++i) v[i] -= center;
    _worldModel->setProperty(_item, _item->metaObject()->property("position"), QVariant::fromValue(position + center));
    _worldModel->setProperty(_item, _item->metaObject()->property("vertexes"), QVariant::fromValue(v));
}

void PolygonCreator::fixInertia()
{
    // XXX: unite it with fixCenterOfMass
    const StepCore::Polygon::VertexList& v = static_cast<StepCore::Polygon*>(_item)->vertexes();
    double mass = static_cast<StepCore::Polygon*>(_item)->mass();
    double area_i, area = 0;
    double inertia = 0;
    unsigned int i;

    if(v.size() > 2) {
        if(v.size() > 2) {
            for(i=0; i+1<v.size(); ++i) {
                area_i = (v[i][0]*v[i+1][1] - v[i][1]*v[i+1][0]) / 2;
                inertia += (v[i].norm2() + v[i].innerProduct(v[i+1]) + v[i+1].norm2())*(area_i/6);
                area += area_i;
            }
            area_i = (v[i][0]*v[0][1] - v[i][1]*v[0][0]) / 2;
            inertia += (v[i].norm2() + v[i].innerProduct(v[0]) + v[0].norm2())*(area_i/6);
            area += area_i;
        }
    }

    if(area == 0) { // all vertexes on one line
        inertia = 0;
        for(i=0; i+1<v.size(); ++i) {
            area_i = (v[i+1] - v[i]).norm();
            inertia += area_i*area_i*area_i / 12 + area_i * (v[i]+v[i+1]).norm2() / 4;
            area += area_i;
        }

        if(area == 0) inertia = 0; // all vertexes are at one point
        else inertia /= area;
    }

    inertia = inertia * mass; // 1 = 1m XXX XXX XXX
    _worldModel->setProperty(_item, _item->metaObject()->property("inertia"), QVariant::fromValue(inertia));
}

bool PolygonCreator::sceneEvent(QEvent* event)
{
    QGraphicsSceneMouseEvent* mouseEvent = static_cast<QGraphicsSceneMouseEvent*>(event);

    if(!_item && event->type() == QEvent::GraphicsSceneMousePress && mouseEvent->button() == Qt::LeftButton) {
        QPointF pos = mouseEvent->scenePos();
        QVariant vpos = QVariant::fromValue(WorldGraphicsItem::pointToVector(pos));

        _worldModel->simulationPause();
        _worldModel->beginMacro(i18n("Create %1", _className));
        _item = _worldModel->newItem(_className); Q_ASSERT(_item != NULL);
        _worldModel->setProperty(_item, _item->metaObject()->property("position"), vpos);
        _worldModel->setProperty(_item, _item->metaObject()->property("vertexes"), QString("(0,0)"));
        _worldModel->selectionModel()->setCurrentIndex(_worldModel->objectIndex(_item),
                                                    QItemSelectionModel::ClearAndSelect);

        event->accept();
        return false;

    } else if(_item && event->type() == QEvent::GraphicsSceneMousePress) {
        event->accept();
        return false;

    } else if(_item && (event->type() == QEvent::GraphicsSceneMouseMove ||
                        event->type() == QEvent::GraphicsSceneMouseRelease)) {

        QPointF pos = mouseEvent->scenePos();
        StepCore::Vector2d v = WorldGraphicsItem::pointToVector(pos);

        _worldModel->simulationPause();
        // XXX: don't use strings !
        QString vertexes = _item->metaObject()->property("vertexes")->readString(_item).section(',', 0, -3);
        if(vertexes.isEmpty()) {
            _worldModel->setProperty(_item, _item->metaObject()->property("position"), QVariant::fromValue(v));
            vertexes = QString("(0,0)"); v.setZero();
        } else {
            v -= static_cast<StepCore::Polygon*>(_item)->position();
            vertexes += QString(",(%1,%2)").arg(v[0]).arg(v[1]);
            _worldModel->setProperty(_item, _item->metaObject()->property("vertexes"), vertexes);
        }

        if(event->type() == QEvent::GraphicsSceneMouseRelease) {
            vertexes += QString(",(%1,%2)").arg(v[0]).arg(v[1]);
            _worldModel->setProperty(_item, _item->metaObject()->property("vertexes"), vertexes);
        }
        
        //fixCenterOfMass();
        //fixInertia();
        event->accept();
        return false;

    } else if(_item && event->type() == QEvent::KeyPress &&
                static_cast<QKeyEvent*>(event)->key() == Qt::Key_Return) {
        fixCenterOfMass();
        fixInertia();
        _worldModel->endMacro();
        event->accept();
        return true;
    }
    return false;
}

PolygonGraphicsItem::PolygonGraphicsItem(StepCore::Item* item, WorldModel* worldModel)
    : WorldGraphicsItem(item, worldModel)
{
    Q_ASSERT(dynamic_cast<StepCore::Polygon*>(_item) != NULL);
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemIsMovable);
    setAcceptsHoverEvents(true);
    _velocityHandler = new ArrowHandlerGraphicsItem(item, worldModel, this,
                   _item->metaObject()->property("velocity"));
    _velocityHandler->setVisible(false);
    //scene()->addItem(_velocityHandler);
}

inline StepCore::Polygon* PolygonGraphicsItem::polygon() const
{
    return static_cast<StepCore::Polygon*>(_item);
}

QPainterPath PolygonGraphicsItem::shape() const
{
    return _painterPath;
    //QPainterPath path;
    //double radius = (RADIUS+1)/currentViewScale();
    //path.addEllipse(QRectF(-radius,-radius,radius*2,radius*2));
    //return path;
}

void PolygonGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
    //painter->setPen(QPen(Qt::green, 0));
    //painter->drawRect(boundingRect());

    //painter->save();
    int renderHints = painter->renderHints();
    painter->setRenderHint(QPainter::Antialiasing, true);
    //painter->setPen(QPen(Qt::black, 0));
    painter->setPen(Qt::NoPen);
    painter->setBrush(QBrush(Qt::black));
    painter->drawPath(_painterPath);
    //painter->rotate(polygon()->angle() * 180 / StepCore::Constants::Pi);
    //painter->drawRect(QRectF(-radius,-radius,radius*2,radius*2));
    //painter->setBrush(QBrush());
    painter->setRenderHint(QPainter::Antialiasing, renderHints & QPainter::Antialiasing);
    //painter->restore();

    if(_isSelected) {
        // XXX: do it better
        double s = currentViewScale();
        QRectF rect = _painterPath.boundingRect();
        rect.adjust(-SELECTION_MARGIN/s, -SELECTION_MARGIN/s, SELECTION_MARGIN/s, SELECTION_MARGIN/s);
        //double bs = qMin((b.width() + 2*SELECTION_MARGIN/s)/b.width(), (b.height() + 2*SELECTION_MARGIN/s)/b.height());
        //QPainterPath selPainterPath = QMatrix().scale(bs, bs).map(_painterPath);
        //
        painter->setRenderHint(QPainter::Antialiasing, true);
        painter->setPen(QPen(SELECTION_COLOR, 0, Qt::DashLine));
        painter->setBrush(QBrush());
        painter->drawRect(rect);
        //radius = (RADIUS+SELECTION_MARGIN)/s;
        //painter->drawEllipse(QRectF(-radius, -radius, radius*2, radius*2));
        //painter->drawPath(_painterPath);
        painter->setRenderHint(QPainter::Antialiasing, renderHints & QPainter::Antialiasing);
    }

    if(_isSelected || _isMouseOverItem) {
        painter->setPen(QPen(Qt::blue, 0));
        drawArrow(painter, polygon()->velocity());
        painter->setPen(QPen(Qt::red, 0));
        drawArrow(painter, polygon()->force()/polygon()->mass());
    }
}

void PolygonGraphicsItem::viewScaleChanged()
{
    /// XXX: optimize it !
    prepareGeometryChange();

    const StepCore::Vector2d& v = polygon()->velocity();
    const StepCore::Vector2d  a = polygon()->force() / polygon()->mass();
    double s = currentViewScale();

    _painterPath = QPainterPath();
    _painterPath.setFillRule(Qt::WindingFill);

    if(polygon()->vertexes().size() > 0) {
        _painterPath.moveTo(vectorToPoint( polygon()->vertexes()[0] ));
        for(unsigned int i=1; i<polygon()->vertexes().size(); ++i) {
            _painterPath.lineTo(vectorToPoint( polygon()->vertexes()[i] ));
        }
        _painterPath.closeSubpath();
        _painterPath = QMatrix().rotate(polygon()->angle() * 180 / StepCore::Constants::Pi).map(_painterPath);
    } else _painterPath.addEllipse(-1/s, -1/s, 2/s, 2/s);

    _boundingRect = _painterPath.boundingRect() 
                    | QRectF(0, 0, v[0], v[1]).normalized()
                    | QRectF(0, 0, a[0], a[1]).normalized();
    double adjust = (ARROW_STROKE+SELECTION_MARGIN)/s;
    _boundingRect.adjust(-adjust,-adjust, adjust, adjust);
}

void PolygonGraphicsItem::worldDataChanged(bool dynamicOnly)
{
    Q_UNUSED(dynamicOnly)
    // XXX: TODO do not redraw everything each time
    setPos(vectorToPoint(polygon()->position()));
    viewScaleChanged();
    update();
}

void PolygonGraphicsItem::stateChanged()
{
    if(_isSelected) _velocityHandler->setVisible(true);
    else _velocityHandler->setVisible(false);
    viewScaleChanged();
    update();
}

