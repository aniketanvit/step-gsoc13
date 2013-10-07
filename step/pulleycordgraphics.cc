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

#include "stepcore/pulleycord.h"
#include "pulleycordgraphics.h"
#include "worldscene.h"
#include "worldmodel.h"
#include "worldfactory.h"

#include <QPainter>
#include <QEvent>
#include <QGraphicsSceneMouseEvent>
#include <QItemSelectionModel>
#include <KLocale> 
#include <QDebug>

bool PulleyCordCreator::sceneEvent(QEvent* event)
{
  QGraphicsSceneMouseEvent* mouseEvent = static_cast<QGraphicsSceneMouseEvent*>(event); 
  if(event->type() == QEvent::GraphicsSceneMousePress && mouseEvent->button() == Qt::LeftButton) {
    
    QPointF pos = mouseEvent->scenePos();
    QVariant vpos = QVariant::fromValue(WorldGraphicsItem::pointToVector(pos));
    _worldModel->simulationPause();
    _worldModel->beginMacro(i18n("Create %1", _worldModel->newItemName(_className)));
    _item = _worldModel->createItem(_className); Q_ASSERT(_item != NULL);
    _worldModel->setProperty(_item, "position", vpos);
    _worldModel->selectionModel()->setCurrentIndex(_worldModel->objectIndex(_item),
						   QItemSelectionModel::ClearAndSelect);
    _worldModel->addItem(_item);
    showMessage(MessageFrame::Information,
		i18n("Move mouse and release left mouse button to define a radius of the %1", classNameTr()));
    
    return true;
  } else if(event->type() == QEvent::GraphicsSceneMouseMove && mouseEvent->buttons() & Qt::LeftButton) {
    
    _worldModel->simulationPause();
  StepCore::Vector2d pos = WorldGraphicsItem::pointToVector(mouseEvent->scenePos());
  double radius = (pos - static_cast<StepCore::PulleyCord*>(_item)->position()).norm();
  _worldModel->setProperty(_item, "radius", QVariant::fromValue(radius));
  return true;
    
  } else if(event->type() == QEvent::GraphicsSceneMouseRelease && mouseEvent->button() == Qt::LeftButton) {
    
    _worldModel->simulationPause();
  StepCore::Vector2d pos = WorldGraphicsItem::pointToVector(mouseEvent->scenePos());
  StepCore::PulleyCord* pulley = static_cast<StepCore::PulleyCord*>(_item);
  double radius = (pos - pulley->position()).norm();
  if(radius == 0) radius = 0.5;
  _worldModel->setProperty(_item, "radius", QVariant::fromValue(radius));
  
  _worldModel->selectionModel()->setCurrentIndex(_worldModel->objectIndex(_item),
						 QItemSelectionModel::ClearAndSelect);
  _worldModel->endMacro();
  
  setFinished();
  return true;
    
  }
  
  return false;
  
}

PulleyCordHandlerGraphicsItem::PulleyCordHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel, 
						     QGraphicsItem* parent, int num): WorldGraphicsItem(item, 
						      worldModel, parent), _num(num)
{
  Q_ASSERT(_num == 1 || _num == 2);
  setFlag(QGraphicsItem::ItemIsMovable);
  setFlag(QGraphicsItem::ItemIsSelectable);
  setZValue(HANDLER_ZVALUE);
  setExclusiveMoving(true);
  setExclusiveMovingMessage(i18n("Move cord of %1", _item->name()));
  setPos(0,0);
}

void PulleyCordHandlerGraphicsItem::viewScaleChanged()
{
  prepareGeometryChange();
  double w = HANDLER_SIZE/currentViewScale()/2;
  _boundingRect = QRectF(-w, -w, w*2, w*2);
}

void PulleyCordHandlerGraphicsItem::worldDataChanged(bool)
{
 if(_num == 1){
    setPos(vectorToPoint(static_cast<StepCore::PulleyCord*>(_item)->position1() - static_cast<StepCore::PulleyCord*>(_item)->position()));
 }else if(_num == 2) {
    setPos(vectorToPoint(static_cast<StepCore::PulleyCord*>(_item)->position2() - static_cast<StepCore::PulleyCord*>(_item)->position()));
 }
                        
}

void PulleyCordHandlerGraphicsItem::mouseSetPos(const QPointF& pos, const QPointF&, MovingState movingState)
{
  static_cast<WorldScene*>(scene())->snapItem(parentItem()->mapToParent(pos),
                                     WorldScene::SnapRigidBody | WorldScene::SnapParticle | WorldScene::SnapSetPosition |
				     WorldScene::SnapSetLocalPosition | WorldScene::SnapSetPosition,
				     0, movingState, _item, _num);


}

PulleyCordGraphicsItem::PulleyCordGraphicsItem(StepCore::Item* item, WorldModel* worldModel)
                           : WorldGraphicsItem(item, worldModel)
{
  Q_ASSERT(dynamic_cast<StepCore::PulleyCord*>(_item) != NULL);
  setFlag(QGraphicsItem::ItemIsMovable);
  setFlag(QGraphicsItem::ItemIsSelectable);
  setZValue(JOINT_ZVALUE);
  _handler1 = new PulleyCordHandlerGraphicsItem(item, worldModel, this, 1);
  _handler2 = new PulleyCordHandlerGraphicsItem(item, worldModel, this, 2);
  _handler1->setVisible(true);
  _handler2->setVisible(true);
  
}

QPainterPath PulleyCordGraphicsItem::shape() const
{
  return _painterPath;
}

void PulleyCordGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
  painter->setPen(QPen(QColor::fromRgba(pulleyCord()->color()), 0));
  painter->setBrush(QColor::fromRgba(pulleyCord()->color()));
  painter->drawPath(_painterPath);
  
  if(isSelected()) {
      painter->setRenderHint(QPainter::Antialiasing, true);
      painter->setPen(QPen(SELECTION_COLOR, 0, Qt::DashLine));
      painter->setBrush(Qt::NoBrush);
      painter->drawRect(_boundingRect);
    }
}

void PulleyCordGraphicsItem::worldDataChanged(bool dynamicOnly)
{
  Q_UNUSED(dynamicOnly)
  setPos(vectorToPoint(pulleyCord()->position()));
  viewScaleChanged();
  update();
}

void PulleyCordGraphicsItem::stateChanged()
{
  _handler1->setVisible(true);
  _handler2->setVisible(true);
  
  viewScaleChanged();
  update();
}

void PulleyCordGraphicsItem::viewScaleChanged()
{
  prepareGeometryChange();
  _painterPath = QPainterPath();
  _painterPath.setFillRule(Qt::WindingFill);
  QPointF p = vectorToPoint(pulleyCord()->position());
  double rad = pulleyCord()->radius();
  QPointF p1(-rad, 0);
  QPointF p2(rad, 0);
  double s = currentViewScale();
  double radius = pulleyCord()->radius();
  _painterPath.addEllipse(QRectF(-radius, -radius, 2*radius, 2*radius));
  _painterPath.moveTo(p1);
  _painterPath.lineTo(vectorToPoint(pulleyCord()->position1()-pulleyCord()->position()));
  _painterPath.moveTo(p2);
  _painterPath.lineTo(vectorToPoint(pulleyCord()->position2()-pulleyCord()->position()));
  _boundingRect = _painterPath.boundingRect();
  _boundingRect.adjust(-1/2,-1/2,1/2,1/2);
}

