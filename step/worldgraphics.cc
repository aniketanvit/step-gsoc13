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

#include "worldgraphics.h"

#include "settings.h"

#include "worldmodel.h"
#include <stepcore/object.h>
#include <stepcore/world.h>
#include <stepcore/particle.h>
#include <QItemSelectionModel>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QTimer>
#include <QMenu>
#include <KActionCollection>
#include <KIcon>
#include <KLocale>

#include <cmath>

// XXX
#include "worldscene.h"
#include <QDebug>

//XXX
const QColor WorldGraphicsItem::SELECTION_COLOR = QColor(0xff, 0x70, 0x70);

void ItemCreator::showMessage(MessageFrame::Type type, const QString& text, MessageFrame::Flags flags)
{
    if(Settings::showCreationTips()) {
        if(!(flags & MessageFrame::CloseButton) && !(flags & MessageFrame::CloseTimer)) {
            _messageId = _worldScene->changeMessage(_messageId, type, text, flags);
        } else {
            _worldScene->showMessage(type, text, flags);
        }
    }
}

void ItemCreator::closeMessage()
{
    _worldScene->closeMessage(_messageId);
}

void ItemCreator::start()
{
    showMessage(MessageFrame::Information,
            i18n("Click on the scene to create a %1", classNameTr()));
}

bool ItemCreator::sceneEvent(QEvent* event)
{
    if(event->type() == QEvent::GraphicsSceneMousePress) {
        _worldModel->simulationPause();

        _worldModel->beginMacro(i18n("Create %1", _worldModel->newItemName(_className)));
        _item = _worldModel->createItem(_className); Q_ASSERT(_item != NULL);
        const StepCore::MetaProperty* property = _item->metaObject()->property("position");
        if(property != NULL) {
            QGraphicsSceneMouseEvent* mouseEvent = static_cast<QGraphicsSceneMouseEvent*>(event);
            QPointF pos = mouseEvent->scenePos();
            QVariant vpos = QVariant::fromValue(WorldGraphicsItem::pointToVector(pos));
            _worldModel->setProperty(_item, property, vpos);
        }
        _worldModel->addItem(_item);

        _worldModel->endMacro();

        _worldModel->selectionModel()->setCurrentIndex(_worldModel->objectIndex(_item),
                                                    QItemSelectionModel::ClearAndSelect);

        showMessage(MessageFrame::Information,
                i18n("%1 named '%2' created", classNameTr(), _item->name()),
                MessageFrame::CloseButton | MessageFrame::CloseTimer);

        setFinished();
        return true;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////

void AttachableItemCreator::start()
{
    if(_twoEnds)
        showMessage(MessageFrame::Information,
            i18n("Press left mouse button to position first end of a %1\n"
                 "then drag and release it to position the second end", classNameTr()));
    else
        showMessage(MessageFrame::Information,
            i18n("Click on the scene to create a %1", classNameTr()));
}

bool AttachableItemCreator::sceneEvent(QEvent* event)
{
    QGraphicsSceneMouseEvent* mouseEvent = static_cast<QGraphicsSceneMouseEvent*>(event);

    if(event->type() == QEvent::GraphicsSceneMouseMove && _item == NULL) {
        _worldScene->snapHighlight(mouseEvent->scenePos(), _snapFlags, _snapTypes);
        return false;

    } else if(event->type() == QEvent::GraphicsSceneMousePress && mouseEvent->button() == Qt::LeftButton) {
        QPointF pos = mouseEvent->scenePos();
        _worldModel->simulationPause();
        _worldModel->beginMacro(i18n("Create %1", _worldModel->newItemName(_className)));
        _item = _worldModel->createItem(className()); Q_ASSERT(_item != NULL);

        if(_twoEnds) {
            _worldScene->snapItem(pos, _snapFlags, _snapTypes, WorldGraphicsItem::Finished, _item, 1);
            _worldModel->setProperty(_item, "localPosition2",
                            QVariant::fromValue(WorldGraphicsItem::pointToVector(pos)));
            _worldModel->setProperty(_item, "restLength", 0);

            showMessage(MessageFrame::Information,
                i18n("Release left mouse button to position second end of the %1", classNameTr()));
        } else {
            _worldScene->snapItem(pos, _snapFlags, _snapTypes, WorldGraphicsItem::Finished, _item);
            showMessage(MessageFrame::Information,
                i18n("%1 named '%2' created", classNameTr(), _item->name()),
                MessageFrame::CloseButton | MessageFrame::CloseTimer);
            _worldModel->endMacro();
            setFinished();
        }
        _worldModel->addItem(_item);
        _worldModel->selectionModel()->setCurrentIndex(_worldModel->objectIndex(_item),
                                                       QItemSelectionModel::ClearAndSelect);
        return true;

    } else if(event->type() == QEvent::GraphicsSceneMouseMove &&
                    (mouseEvent->buttons() & Qt::LeftButton) && _twoEnds) {

        QPointF pos = mouseEvent->scenePos();
        _worldScene->snapItem(pos, _snapFlags, _snapTypes, WorldGraphicsItem::Moving, _item, 2);

        double length =
            (_item->metaObject()->property("position2")->readVariant(_item).value<StepCore::Vector2d>() -
             _item->metaObject()->property("position1")->readVariant(_item).value<StepCore::Vector2d>()).norm();
        _worldModel->setProperty(_item, "restLength", length);
        return true;

    } else if(event->type() == QEvent::GraphicsSceneMouseRelease &&
                    (mouseEvent->button() == Qt::LeftButton) && _twoEnds) {

        QPointF pos = mouseEvent->scenePos();
        _worldScene->snapItem(pos, _snapFlags, _snapTypes, WorldGraphicsItem::Finished, _item, 2);

        double length =
            (_item->metaObject()->property("position2")->readVariant(_item).value<StepCore::Vector2d>() -
             _item->metaObject()->property("position1")->readVariant(_item).value<StepCore::Vector2d>()).norm();
        _worldModel->setProperty(_item, "restLength", length);
        _worldModel->endMacro();

        showMessage(MessageFrame::Information,
            i18n("%1 named '%2' created", classNameTr(), _item->name()),
            MessageFrame::CloseButton | MessageFrame::CloseTimer);

        setFinished();
        return true;
    }

    return false;
}

void AttachableItemCreator::abort()
{
    _worldScene->snapClear();
}

/////////////////////////////////////////////////////////////////////////////////////////

WorldGraphicsItem::WorldGraphicsItem(StepCore::Item* item, WorldModel* worldModel, QGraphicsItem* parent)
    : QGraphicsItem(parent), _item(item), _worldModel(worldModel), _exclusiveMoving(false),
      _onHoverHandlerEnabled(false), _isHighlighted(false), _isMouseOverItem(false), _isSelected(false),
      _isMoving(false), _onHoverHandler(0), _onHoverHandlerTimer(false)
{
    // XXX: use persistant indexes here and in propertiesbrowser
    setZValue(BODY_ZVALUE);
}

double WorldGraphicsItem::currentViewScale() const
{
    if(!scene()) return 1;
    return static_cast<WorldScene*>(scene())->currentViewScale();
}

QColor WorldGraphicsItem::highlightColor(const QColor& color)
{
    qreal h, s, v, a;
    QColor hsv = color.toHsv();
    hsv.getHsvF(&h, &s, &v, &a);

    v += float(COLOR_HIGHLIGHT_AMOUNT)/100;
    if (v > 1.0) {
        // overflow... adjust saturation
        s -= v - 1.0;
        if (s < 0) s = 0.0;
        v = 1.0;
    }

    hsv.setHsvF(h, s, v, a);

    // convert back to same color spec as original color
    return hsv.convertTo(color.spec());
}

void WorldGraphicsItem::drawArrow(QPainter* painter, const StepCore::Vector2d& r,
                                                     const StepCore::Vector2d& v)
{
    double s = currentViewScale();
    if(v.squaredNorm()*s*s > ARROW_STROKE*ARROW_STROKE) { // do not draw too small vectors
        StepCore::Vector2d vv = r+v;
        painter->drawLine(QLineF(r[0], r[1], vv[0], vv[1]));

        const StepCore::Vector2d vn = v * (ARROW_STROKE / s / v.norm());
        painter->drawLine(QLineF(vv[0], vv[1], vv[0] - 0.866*vn[0] - 0.5  *vn[1],
                                               vv[1] + 0.5  *vn[0] - 0.866*vn[1]));
        painter->drawLine(QLineF(vv[0], vv[1], vv[0] - 0.866*vn[0] + 0.5  *vn[1],
                                               vv[1] - 0.5  *vn[0] - 0.866*vn[1]));
    }
}

void WorldGraphicsItem::drawCircularArrow(QPainter* painter, const StepCore::Vector2d& r,
                                                    double angle, double radius)
{
    double s = currentViewScale();
    double rs = radius/s;
    double x0 = rs*cos(angle)+r[0]/s;
    double y0 = rs*sin(angle)+r[1]/s;
    double xAr1 = CIRCULAR_ARROW_STROKE*cos(2*M_PI/3 + angle)/s;
    double yAr1 = CIRCULAR_ARROW_STROKE*sin(2*M_PI/3 + angle)/s;
    double xAr2 = CIRCULAR_ARROW_STROKE*cos(M_PI/3 + angle)/s;
    double yAr2 = CIRCULAR_ARROW_STROKE*sin(M_PI/3 + angle)/s;

    QRectF rr(-rs, -rs, 2*rs, 2*rs);

    if(angle > 2*M_PI || angle < -2*M_PI) {
        painter->drawArc(rr, int(-angle*180*16/M_PI-150*16), 300*16);
        for(int i=1; i<5; ++i)
            painter->drawArc(rr, int(-angle*180*16/M_PI-150*16-i*12*16), 1*16);
    } else if(angle > 0) {
        painter->drawArc(rr, -int(angle*180*16/M_PI), int(angle*180*16/M_PI));
    } else {
        painter->drawArc(rr, 0, int(-angle*180*16/M_PI));
    }

    // do not draw too small vectors
    if(angle > 0 && angle*radius > CIRCULAR_ARROW_STROKE) {
        painter->drawLine(QLineF(x0, y0, x0-xAr1, y0-yAr1));
        painter->drawLine(QLineF(x0, y0, x0-xAr2, y0-yAr2));
    } if(angle < 0 && -angle*radius > CIRCULAR_ARROW_STROKE) {
        painter->drawLine(QLineF(x0, y0, x0+xAr1, y0+yAr1));
        painter->drawLine(QLineF(x0, y0, x0+xAr2, y0+yAr2));
    }
}

void WorldGraphicsItem::drawArrow(QPainter* painter, const StepCore::Vector2d& v)
{
    drawArrow(painter, StepCore::Vector2d::Zero(), v);
}

void WorldGraphicsItem::drawCircularArrow(QPainter* painter, double angle, double radius)
{
    drawCircularArrow(painter, StepCore::Vector2d::Zero(), angle, radius);
}

void WorldGraphicsItem::mouseSetPos(const QPointF& pos, const QPointF&, MovingState)
{
    const StepCore::MetaProperty* property = _item->metaObject()->property("position");
    if(property != NULL) {
        _worldModel->simulationPause();
        _worldModel->setProperty(_item, property,
                                QVariant::fromValue( pointToVector(pos) ));
    } else {
        Q_ASSERT(false);
    }
}

void WorldGraphicsItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    // Workaround for bug in Qt
    if (scene()->itemAt(event->scenePos()) != this) {
        event->ignore();
        return;
    }
    
    if(event->button() == Qt::LeftButton && (flags() & ItemIsSelectable)) {
        bool multiSelect = (event->modifiers() & Qt::ControlModifier) != 0;
        if(!multiSelect && !isSelected()) {
            if(scene()) scene()->clearSelection();
            _worldModel->selectionModel()->clearSelection();
            setSelected(true);
        }
    } else if (!(flags() & ItemIsMovable)) {
        event->ignore();
    }
}

void WorldGraphicsItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    // Workaround for bug in Qt
    if (scene()->mouseGrabberItem() != this &&
        scene()->itemAt(event->scenePos()) != this) {
        event->ignore();
        return;
    }
    
    if((event->buttons() & Qt::LeftButton) && (flags() & ItemIsMovable)) {
        QPointF pdiff(mapToParent(event->pos()) - mapToParent(event->lastPos()));
        QPointF newPos(mapToParent(event->pos()) - matrix().map(event->buttonDownPos(Qt::LeftButton)));

        QPointF diff = newPos - pos();
        if(diff == QPointF(0, 0)) return;

        MovingState movingState = Moving;
        if(!_isMoving) {
            if(_exclusiveMoving) {
                if(!_exclusiveMovingMessage.isEmpty()) _worldModel->beginMacro(_exclusiveMovingMessage);
                else _worldModel->beginMacro(i18n("Move %1", _item->name()));

            } else {
                int count = 0;
                foreach(QGraphicsItem *item, scene()->selectedItems())
                    if(item != this && (item->flags() & ItemIsMovable) &&
                                (!item->parentItem() || !item->parentItem()->isSelected()) &&
                                dynamic_cast<WorldGraphicsItem*>(item)) {
                        ++count;
                    }
                if(!this->parentItem() || !this->parentItem()->isSelected()) ++count;

                _worldModel->beginMacro(i18n("Move %1", count == 1 ? _item->name() : i18n("several objects")));
            }

            movingState = Started;
            _isMoving = true;
        }

        if(_exclusiveMoving) {
            mouseSetPos(newPos, pdiff, movingState);
        } else {
            // Move all selected items
            foreach(QGraphicsItem *item, scene()->selectedItems()) {
                if(item != this && (item->flags() & ItemIsMovable) &&
                            (!item->parentItem() || !item->parentItem()->isSelected())) {
                    WorldGraphicsItem* worldItem = dynamic_cast<WorldGraphicsItem*>(item);
                    if(worldItem) worldItem->mouseSetPos(item->pos() + diff, pdiff, movingState);
                }
            }
            if(!this->parentItem() || !this->parentItem()->isSelected())
                mouseSetPos(newPos, pdiff, movingState);
        }
    } else {
        event->ignore();
    }
}

void WorldGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    // Workaround for bug in Qt
    if (scene()->itemAt(event->scenePos()) != this) {
        event->ignore();
        return;
    }
    
    if(_isMoving && event->button() == Qt::LeftButton) {
        QPointF pdiff(mapToParent(event->pos()) - mapToParent(event->lastPos()));
        QPointF newPos(mapToParent(event->pos()) - matrix().map(event->buttonDownPos(Qt::LeftButton)));
        QPointF diff = newPos - pos();

        if(_exclusiveMoving) {
            mouseSetPos(newPos, pdiff, Finished);
        } else {
            foreach(QGraphicsItem *item, scene()->selectedItems()) {
                if(item != this && (item->flags() & ItemIsMovable) &&
                            (!item->parentItem() || !item->parentItem()->isSelected())) {
                    WorldGraphicsItem* worldItem = dynamic_cast<WorldGraphicsItem*>(item);
                    if(worldItem) worldItem->mouseSetPos(item->pos() + diff, pdiff, Finished);
                }
            }
            if(!this->parentItem() || !this->parentItem()->isSelected())
                mouseSetPos(newPos, pdiff, Finished);
        }

        _worldModel->endMacro();
        _isMoving = false;
    }
    if(flags() & ItemIsSelectable) {
        bool multiSelect = (event->modifiers() & Qt::ControlModifier) != 0;
        if(event->scenePos() == event->buttonDownScenePos(Qt::LeftButton)) {
            // The item didn't move
            if (multiSelect) {
                setSelected(!isSelected());
            } else {
                if(scene()) scene()->clearSelection();
                _worldModel->selectionModel()->clearSelection();
                setSelected(true);
            }
        }
    }
}

void WorldGraphicsItem::hoverMoveEvent(QGraphicsSceneHoverEvent* event)
{
    if(_onHoverHandlerEnabled) {
        OnHoverHandlerGraphicsItem* newOnHoverHandler = createOnHoverHandler(event->scenePos());
        if(_onHoverHandler && !newOnHoverHandler) {
             if(!_onHoverHandlerTimer) {
                _onHoverHandler->setDeleteTimerEnabled(true);
                _onHoverHandlerTimer = true;
             }
        } else if(_onHoverHandler == newOnHoverHandler) {
            if(_onHoverHandler && _onHoverHandlerTimer) {
                _onHoverHandler->setDeleteTimerEnabled(false);
                _onHoverHandlerTimer = false;
            }
        } else {
            delete _onHoverHandler;
            _onHoverHandler = newOnHoverHandler;
            _onHoverHandlerTimer = false;
        }
    }
}

void WorldGraphicsItem::hoverEnterEvent(QGraphicsSceneHoverEvent* /*event*/)
{
    if(_onHoverHandlerEnabled && _onHoverHandler && !_onHoverHandlerTimer)
        _onHoverHandler->setDeleteTimerEnabled(false);
    _isMouseOverItem = true;
    stateChanged();
    //update(_boundingRect);
}

void WorldGraphicsItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* /*event*/)
{
    if(_onHoverHandlerEnabled && _onHoverHandler && !_onHoverHandlerTimer)
        _onHoverHandler->setDeleteTimerEnabled(true);
    _isMouseOverItem = false;
    stateChanged();
    //update(_boundingRect);
}

QVariant WorldGraphicsItem::itemChange(GraphicsItemChange change, const QVariant& value)
{
    if(change == ItemSelectedHasChanged && value.toBool() != _isSelected && scene()) {
        _isSelected = value.toBool();
        if(_isSelected) setZValue(zValue() + 1);
        else setZValue(zValue() - 1);

        QModelIndex index = _worldModel->objectIndex(_item);
        if(_isSelected && !_worldModel->selectionModel()->isSelected(index)) {
            _worldModel->selectionModel()->setCurrentIndex(index, QItemSelectionModel::Select);
        } else if(!_isSelected && _worldModel->selectionModel()->isSelected(index)) {
            _worldModel->selectionModel()->select(index, QItemSelectionModel::Deselect);
        }

        stateChanged();
    }
    return QGraphicsItem::itemChange(change, value);
}

void WorldGraphicsItem::setOnHoverHandlerEnabled(bool enabled)
{
    _onHoverHandlerEnabled = enabled;
    if(!_onHoverHandlerEnabled) {
        _onHoverHandlerTimer = false;
        delete _onHoverHandler;
    }
}

void WorldGraphicsItem::viewScaleChanged()
{
}

void WorldGraphicsItem::worldDataChanged(bool)
{
}

void WorldGraphicsItem::stateChanged()
{
}

void WorldGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*)
{
    painter->setPen(QPen(Qt::gray, 0));
    painter->drawRect(_boundingRect);
}

void WorldGraphicsItem::contextMenuEvent(QGraphicsSceneContextMenuEvent* event)
{
    event->accept();

    QModelIndex index = _worldModel->objectIndex(_item);
    if(flags() & QGraphicsItem::ItemIsSelectable)
        _worldModel->selectionModel()->setCurrentIndex(index, QItemSelectionModel::ClearAndSelect);

    QMenu* menu = _worldModel->createContextMenu(index);
    menu->exec(event->screenPos());
    delete menu;
}

/////////////////////////////////////////////////////////////////////////////////////////

ArrowHandlerGraphicsItem::ArrowHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel, 
                         QGraphicsItem* parent, const StepCore::MetaProperty* property,
                         const StepCore::MetaProperty* positionProperty)
    : WorldGraphicsItem(item, worldModel, parent), _property(property), _positionProperty(positionProperty)
{
    Q_ASSERT(!_property || _property->userTypeId() == qMetaTypeId<StepCore::Vector2d>());
    setFlag(QGraphicsItem::ItemIsMovable);
    setZValue(HANDLER_ZVALUE);
    _exclusiveMoving = true;
    if(_property) _exclusiveMovingMessage = i18n("Change %1.%2", _item->name(), _property->nameTr());
    else _exclusiveMovingMessage = i18n("Change %1", _item->name());
}

void ArrowHandlerGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
    painter->setPen(QPen(Qt::gray, 0));
    painter->drawRect(_boundingRect);
}

void ArrowHandlerGraphicsItem::viewScaleChanged()
{
    if(isVisible()) {
        prepareGeometryChange();
        double w = HANDLER_SIZE/currentViewScale()/2;
        _boundingRect = QRectF(-w, -w, w*2, w*2);
    }
}

void ArrowHandlerGraphicsItem::worldDataChanged(bool)
{
    if(isVisible()) {
        //kDebug() << "ArrowHandlerGraphicsItem::worldDataChanged()" << endl;
        setPos(vectorToPoint(value()));
    }
}

QVariant ArrowHandlerGraphicsItem::itemChange(GraphicsItemChange change, const QVariant& value)
{
    if(change == QGraphicsItem::ItemVisibleHasChanged) {
        if(isVisible()) {
            viewScaleChanged();
            worldDataChanged(false);
        }
    }
    return WorldGraphicsItem::itemChange(change, value);
}

void ArrowHandlerGraphicsItem::mouseSetPos(const QPointF& pos, const QPointF&, MovingState)
{
    setValue(pointToVector(pos));
}

/*
void ArrowHandlerGraphicsItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if ((event->buttons() & Qt::LeftButton) && (flags() & ItemIsMovable)) {
        if(!_isMoving) {
            if(_property)
                _worldModel->beginMacro(i18n("Change %1.%2", _item->name(), _property->nameTr()));
            else
                _worldModel->beginMacro(i18n("Change %1", _item->name()));
            _isMoving = true;
        }
        QPointF newPos(mapToParent(event->pos()) - matrix().map(event->buttonDownPos(Qt::LeftButton)));
        setValue(pointToVector(newPos));
        //_worldModel->simulationPause();
        //_worldModel->setProperty(_item, _property, QVariant::fromValue(pointToVector(newPos)));
        //Q_ASSERT(_property->writeVariant(_item, QVariant::fromValue(v)));
        //_worldModel->setData(_worldModel->objectIndex(_item), QVariant(), WorldModel::ObjectRole);
    } else  event->ignore();
}*/

StepCore::Vector2d ArrowHandlerGraphicsItem::value()
{
    if(_property) {
        StepCore::Vector2d ret = _property->readVariant(_item).value<StepCore::Vector2d>();
        if(_positionProperty)
            ret += _positionProperty->readVariant(_item).value<StepCore::Vector2d>();
        return ret;
    } else {
        return StepCore::Vector2d::Zero();
    }
}

void ArrowHandlerGraphicsItem::setValue(const StepCore::Vector2d& value)
{
    if(_property) {
        _worldModel->simulationPause();
        StepCore::Vector2d v = value;
        if(_positionProperty)
            v -= _positionProperty->readVariant(_item).value<StepCore::Vector2d>();
        _worldModel->setProperty(_item, _property, QVariant::fromValue(v));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

CircularArrowHandlerGraphicsItem::CircularArrowHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel, 
                         QGraphicsItem* parent, double radius, const StepCore::MetaProperty* property,
                         const StepCore::MetaProperty* positionProperty)
    : WorldGraphicsItem(item, worldModel, parent), _property(property), _positionProperty(positionProperty), _radius(radius)
{
    Q_ASSERT(!_property || _property->userTypeId() == qMetaTypeId<double>());
    setFlag(QGraphicsItem::ItemIsMovable);
    setZValue(HANDLER_ZVALUE);
}

void CircularArrowHandlerGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
    painter->setPen(QPen(Qt::gray, 0));
    painter->drawRect(_boundingRect);
}

void CircularArrowHandlerGraphicsItem::viewScaleChanged()
{
    if(isVisible()) {
        prepareGeometryChange();
        double w = HANDLER_SIZE/currentViewScale()/2;
        _boundingRect = QRectF(-w, -w, w*2, w*2);
        worldDataChanged(true);
    }
}

void CircularArrowHandlerGraphicsItem::worldDataChanged(bool)
{
    if(isVisible()) {
        double s = currentViewScale();
        double angle = value();
        setPos(_radius*cos(angle)/s, _radius*sin(angle)/s);
    }
}

QVariant CircularArrowHandlerGraphicsItem::itemChange(GraphicsItemChange change, const QVariant& value)
{
    if(change == QGraphicsItem::ItemVisibleHasChanged) {
        if(isVisible()) {
            viewScaleChanged();
            worldDataChanged(false);
        }
    }
    return WorldGraphicsItem::itemChange(change, value);
}

void CircularArrowHandlerGraphicsItem::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    if ((event->buttons() & Qt::LeftButton) && (flags() & ItemIsMovable)) {
        if(!_isMoving) {
            if(_property)
                _worldModel->beginMacro(i18n("Change %1.%2", _item->name(), _property->nameTr()));
            else
                _worldModel->beginMacro(i18n("Change %1", _item->name()));
            _isMoving = true;
        }

        QPointF newPos(mapToParent(event->pos()) - matrix().map(event->buttonDownPos(Qt::LeftButton)));
        double newValue = atan2(newPos.y(),newPos.x());
        if(newValue < 0) newValue += 2*M_PI;

        double v = value();
        double b = 2*M_PI * int(v / (2*M_PI) - (v<0 ? 1 : 0));
        double f = v - b;

        if(f < M_PI_2 && newValue > 3*M_PI_2) newValue -= 2*M_PI;
        else if(f > 3*M_PI_2 && newValue < M_PI_2) newValue += 2*M_PI;

        setValue(b + newValue);
    } else event->ignore();
}

void CircularArrowHandlerGraphicsItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if(_isMoving && event->button() == Qt::LeftButton) {
        _worldModel->endMacro();
        _isMoving = false;
    }
}

double CircularArrowHandlerGraphicsItem::value()
{
    if(_property) return _property->readVariant(_item).value<double>();
    else return 0;
}

void CircularArrowHandlerGraphicsItem::setValue(double value)
{
    if(_property) {
        _worldModel->simulationPause();
        _worldModel->setProperty(_item, _property, QVariant::fromValue(value));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

const StepCore::Vector2d OnHoverHandlerGraphicsItem::corners[4] = {
    StepCore::Vector2d(-0.5,-0.5), StepCore::Vector2d( 0.5,-0.5),
    StepCore::Vector2d(-0.5, 0.5), StepCore::Vector2d( 0.5, 0.5)
};

const StepCore::Vector2d OnHoverHandlerGraphicsItem::scorners[4] = {
    StepCore::Vector2d(0,-1), StepCore::Vector2d( 1,0),
    StepCore::Vector2d(0, 1), StepCore::Vector2d(-1,0)
};

OnHoverHandlerGraphicsItem::OnHoverHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel,
                    QGraphicsItem* parent, const StepCore::MetaProperty* property,
                    const StepCore::MetaProperty* positionProperty, int vertexNum)
    : ArrowHandlerGraphicsItem(item, worldModel, parent, property, positionProperty),
      _vertexNum(vertexNum)
{
    _deleteTimer = new QTimer(this);
    _deleteTimer->setInterval(500);
    _deleteTimer->setSingleShot(true);
    _deleteTimerEnabled = false;
    setAcceptsHoverEvents(true);
    connect(_deleteTimer, SIGNAL(timeout()), this, SLOT(deleteLater()));
}

void OnHoverHandlerGraphicsItem::setDeleteTimerEnabled(bool enabled)
{
    _deleteTimerEnabled = enabled;
    if(_deleteTimerEnabled && !isMouseOverItem()) _deleteTimer->start();
    else _deleteTimer->stop();
}

void OnHoverHandlerGraphicsItem::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
    if(_deleteTimerEnabled) _deleteTimer->stop();
    ArrowHandlerGraphicsItem::hoverEnterEvent(event);
}

void OnHoverHandlerGraphicsItem::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    if(_deleteTimerEnabled) _deleteTimer->start();
    ArrowHandlerGraphicsItem::hoverLeaveEvent(event);
}

/////////////////////////////////////////////////////////////////////////////////////////

ItemMenuHandler::ItemMenuHandler(StepCore::Object* object, WorldModel* worldModel, QObject* parent)
    : QObject(parent), _object(object), _worldModel(worldModel)
{
}

void ItemMenuHandler::populateMenu(QMenu* menu, KActionCollection* actions)
{
    StepCore::Item* item = dynamic_cast<StepCore::Item*>(_object);
    
    if (item && item->world() != item) {
        menu->addAction(actions->action("edit_cut"));
        menu->addAction(actions->action("edit_copy"));
        menu->addAction(actions->action("edit_delete"));
    }
}
