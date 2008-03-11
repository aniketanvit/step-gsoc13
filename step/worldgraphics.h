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

#ifndef STEP_WORLDGRAPHICS_H
#define STEP_WORLDGRAPHICS_H

#include "worldscene.h"

#include <stepcore/vector.h>
#include <QGraphicsItem>
#include <QRectF>
#include <QColor>
#include <QMenu>

namespace StepCore {
    class Object;
    class Item;
    class Particle;
    class Spring;
    class MetaProperty;
}
class WorldModel;
class WorldScene;
class QEvent;

class ItemCreator
{
public:
    ItemCreator(const QString& className, WorldModel* worldModel, WorldScene* worldScene)
           : _className(className), _worldModel(worldModel),
             _worldScene(worldScene), _item(NULL), _finished(false), _messageId(-1) {}
    virtual ~ItemCreator() { closeMessage(); }

    QString className() const { return _className; }
    StepCore::Item* item() const { return _item; }

    virtual bool sceneEvent(QEvent* event);
    virtual void abort() {}
    virtual void start();

    bool finished() { return _finished; }

protected:
    void showMessage(MessageFrame::Type type, const QString& text, MessageFrame::Flags flags = 0);
    void closeMessage();
    void setFinished(bool finished = true) { _finished = finished; }
    
    QString     _className;
    WorldModel* _worldModel;
    WorldScene* _worldScene;
    StepCore::Item* _item;
    bool _finished;
    int _messageId;
};

/** \brief Parent class for all graphics items on the scene.
 *
 * This class provides interface for WorldScene and
 * some common functionality to simplify subclassing. */
class WorldGraphicsItem: public QGraphicsItem
{
public:
    /** Constructs WorldGraphicsItem */
    WorldGraphicsItem(StepCore::Item* item, WorldModel* worldModel, QGraphicsItem* parent = 0);

    /** Get StepCore::Item which is represented by this graphicsItem */
    StepCore::Item* item() const { return _item; }

    /** Get item bounding rect. Default implementation returns
     *  value set by setBoundingRect function */
    QRectF boundingRect() const { return _boundingRect; }

    /** Set current bounding rect. Should be called by subclass. */
    void setBoundingRect(const QRectF& rect) { _boundingRect = rect; }

    /** Virtual function which is called when view scale is changed */
    virtual void viewScaleChanged();

    /** Virtual function which is called when:
     *  - item selection state changed
     *  - item mouse hover state changed
     */
    virtual void stateChanged();

    /** Virtual function which is called when something in StepCore::World was changed
     *  \arg dynamicOnly Indicated whether only dynamic variables was changed
     *  \note Dynamic variables are variables that can change during simulation,
     *        non-dynamic variables can change only by user action
     */
    virtual void worldDataChanged(bool dynamicOnly);

    /** Get item highlight state */
    bool isItemHighlighted() { return _isHighlighted; }
    /** Set item highlight state */
    void setItemHighlighted(bool highlighted) {
        _isHighlighted = highlighted; update();
    }

    /** Get selection state of the item. This function reflects selection state of
     *  StepCore::Item and differs from QGraphicsItem::isSelected when selection state
     *  of QGraphicsItem was not yet updated (for example in stateChanged() function) */
    bool isItemSelected() { return _isSelected; }
    /** Return true if item is hovered by the mouse */
    bool isMouseOverItem() { return _isMouseOverItem; }

    /** Converts QPointF to StepCore::Vector2d */
    static StepCore::Vector2d pointToVector(const QPointF& point) {
        return StepCore::Vector2d(point.x(), point.y());
    }
    /** Converts StepCore::Vector2d to QPointF */
    static QPointF vectorToPoint(const StepCore::Vector2d& vector) {
        return QPointF(vector[0], vector[1]);
    }

protected:
    enum MovingState { Started, Moving, Finished };

    /** Virtual function which is called when item is moved by the mouse. Default implementation
     *  tries to set "position" property of _item */
    virtual void mouseSetPos(const QPointF& pos, const QPointF& diff, MovingState movingState);

    /** Returns current view scale of the scene */
    double currentViewScale() const;

    /** Returns highlighted copy of the color */
    QColor highlightColor(const QColor& color);

    ///* Draw handler item */
    //void drawHandler(QPainter* painter, const StepCore::Vector2d& v);

    /** Draw an arrow starting at r and ending at v */
    void drawArrow(QPainter* painter, const StepCore::Vector2d& r,
                                        const StepCore::Vector2d& v);
    /** Draw an arrow starting at (0,0) and ending at v */
    void drawArrow(QPainter* painter, const StepCore::Vector2d& v);

    /** Draw circular arrow with the center at r and with given radius and angle */
    void drawCircularArrow(QPainter* painter, const StepCore::Vector2d& r,
                                        double angle, double radius);
    /** Draw circular arrow with the center at (0,0) and with given radius and angle */
    void drawCircularArrow(QPainter* painter, double angle, double radius);

    /** Set to true if the item should be moved alone (without other selected items) */
    void setExclusiveMoving(bool value) { _exclusiveMoving = value; }

    /** Set custum test for undo command for moving item. Works only if exclusiveMoving is true */
    void setExclusiveMovingMessage(const QString& message) { _exclusiveMovingMessage = message; }

protected:
    StepCore::Item* _item;
    WorldModel* _worldModel;

    QRectF  _boundingRect;
    QString _exclusiveMovingMessage; ///< Custom Undo message for moving item
    bool    _exclusiveMoving; ///< Set to true for items that should be moved alone

    bool    _isHighlighted; 
    bool    _isMouseOverItem;
    bool    _isSelected;
    bool    _isMoving;

    void mousePressEvent(QGraphicsSceneMouseEvent* event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);

    void hoverEnterEvent(QGraphicsSceneHoverEvent* event);
    void hoverLeaveEvent(QGraphicsSceneHoverEvent* event);

    void contextMenuEvent(QGraphicsSceneContextMenuEvent* event);

    QVariant itemChange(GraphicsItemChange change, const QVariant& value);

    static const QColor SELECTION_COLOR;     ///< Default color for selection rectangle
    static const int SELECTION_MARGIN = 4;   ///< Default distance from object to selection rectangle
    static const int ARROW_STROKE = 6;       ///< Default size of an arrow stroke
    static const int CIRCULAR_ARROW_STROKE = 6; ///< Default size of circular arrow stroke

    static const int HANDLER_SIZE = 6;      ///< Default size of the handler

    static const int ANGLE_HANDLER_RADIUS = 15;     ///< Default radius of the angle handler for RigidBody
    static const int ANGULAR_VELOCITY_RADIUS = 30;  ///< Default radius of the angularVelocity handler for RigidBody
    static const int ANGULAR_ACCELERATION_RADIUS = 34; ///< Default radius of the angularAcceleration handler

    static const int BODY_ZVALUE = 100;     ///< Default ZValue for bodies
    static const int FORCE_ZVALUE = 200;    ///< Default ZValue for forces
    static const int HANDLER_ZVALUE = 500;  ///< Default ZValue for handlers

    static const int COLOR_HIGHLIGHT_AMOUNT = 30; ///< Highligh amount (in percent for value component)
};

class ArrowHandlerGraphicsItem: public WorldGraphicsItem
{
public:
    ArrowHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel,
                        QGraphicsItem* parent, const StepCore::MetaProperty* property,
                        const StepCore::MetaProperty* positionProperty = NULL);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void viewScaleChanged();
    void worldDataChanged(bool);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant& value);
    virtual StepCore::Vector2d value();
    virtual void setValue(const StepCore::Vector2d& value);

    void mouseSetPos(const QPointF& pos, const QPointF& diff, MovingState movingState);
    const StepCore::MetaProperty* _property;
    const StepCore::MetaProperty* _positionProperty;
    bool  _isVisible;
};

class CircularArrowHandlerGraphicsItem: public WorldGraphicsItem
{
public:
    CircularArrowHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel,
                        QGraphicsItem* parent, double radius, const StepCore::MetaProperty* property,
                        const StepCore::MetaProperty* positionProperty = NULL);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void viewScaleChanged();
    void worldDataChanged(bool);

protected:
    QVariant itemChange(GraphicsItemChange change, const QVariant& value);
    virtual double value();
    virtual void setValue(double value);

    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    const StepCore::MetaProperty* _property;
    const StepCore::MetaProperty* _positionProperty;
    bool   _isVisible;
    double _radius;
};

class ItemMenuHandler: public QObject
{
    Q_OBJECT

public:
    ItemMenuHandler(StepCore::Object* object, WorldModel* worldModel, QObject* parent = 0);
    virtual void populateMenu(QMenu* menu);

protected slots:
    void deleteItem();

protected:
    StepCore::Object* _object;
    WorldModel* _worldModel;
};

#endif

