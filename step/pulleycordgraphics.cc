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


/*
PulleyCordHandlerGraphicsItem::PulleyCordHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel, 
						     QGraphicsItem* parent, int num): WorldGraphicsItem(item, 
						      worldModel, parent), _num(num)
{
  Q_ASSERT(_num == 1 || _num == 2);
  setFlag(QGraphicsItem::ItemIsMovable);
  setZValue(HANDLER_ZVALUE);
  setExclusiveMoving(true);
  setExclusiveMovingMessage(i18n("Move end of %1", _item->name()));
  setPos(0, 0);  
}

void PulleyCordHandlerGraphicsItem::viewScaleChanged()
{
  prepareGeometryChange();
  double w = HANDLER_SIZE/currentViewScale()/2;
  _boundingRect = QRectF(-w, -w, w*2, w*3);
}

void PulleyCordHandlerGraphicsItem::worldDataChanged(bool)
{
 // if(_num == 2) /// this will need to be reimplemented
    setPos(vectorToPoint(static_cast<StepCore::PulleyCord*>(_item)->position2()-
                         static_cast<StepCore::PulleyCord*>(_item)->position1()));
}

void PulleyCordHandlerGraphicsItem::mouseSetPos(const QPointF& pos, const QPointF&, MovingState movingState)
{
  static_cast<WorldScene*>(scene())->snapItem(parentItem()->mapToParent(pos),
			   WorldScene::SnapParticle | WorldScene::SnapRigidBody |
			   WorldScene::SnapSetLocalPosition, 0, movingState, _item, _num);
  
}
*/
PulleyCordGraphicsItem::PulleyCordGraphicsItem(StepCore::Item* item, WorldModel* worldModel)
                           : WorldGraphicsItem(item, worldModel)
{
  Q_ASSERT(dynamic_cast<StepCore::PulleyCord*>(_item) != NULL);
  setFlag(QGraphicsItem::ItemIsSelectable);
  setFlag(QGraphicsItem::ItemIsMovable);
  setZValue(FORCE_ZVALUE);
}

QPainterPath PulleyCordGraphicsItem::shape() const
{
  double s = currentViewScale();
  double radius = pulleyCord()->radius();
  if(radius > 1/s) {
    _painterPath.addEllipse(-radius, -radius, 2*radius, 2*radius);
    //_painterPath = QMatrix().rotate(disk()->angle() * 180 / StepCore::Constants::Pi).map(_painterPath);
  } else {
    _painterPath.addEllipse(-1/s, -1/s, 2/s, 2/s);
  }
      _painterPath = QMatrix().map(_painterPath);
  return _painterPath;
}

void PulleyCordGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
  //painter->setPen(QPen(Qt::blue));
  QColor color(Qt::blue);
  painter->setPen(Qt::NoPen);
  painter->setBrush(QBrush(color));
  
  StepCore::Vector2d p1c, p2c;
  StepCore::Vector2d lCord, rCord;
  if(pulleyCord()->rigidBody1()) p1c = pulleyCord()->rigidBody1()->position();
  else if(pulleyCord()->particle1()) p1c = pulleyCord()->particle1()->position();
  else p1c = pulleyCord()->position1();
  
  if(pulleyCord()->rigidBody2()) p2c = pulleyCord()->rigidBody2()->position();
  else if(pulleyCord()->particle2()) p2c = pulleyCord()->particle2()->position();
  else p2c = pulleyCord()->position2();
  
  StepCore::Vector2d lEnd(pulleyCord()->position()[0]-pulleyCord()->radius(), pulleyCord()->position()[1]);
  StepCore::Vector2d rEnd(pulleyCord()->position()[0]+pulleyCord()->radius(), pulleyCord()->position()[1]);
  
  //painter->drawEllipse(lEnd[0], lEnd[1]-pulleyCord()->radius(), pulleyCord()->radius()*2, pulleyCord()->radius()*2 );
  painter->drawPath(_painterPath);
  StepCore::Vector2d l(lEnd - p1c);
    double lDist = l.norm();
  lCord = lDist*lCord;
  StepCore::Vector2d r(rEnd - p2c);
  double rDist = r.norm();
  rCord = rCord*rDist;
  
  painter->drawLine(vectorToPoint(lEnd), vectorToPoint(lEnd+lCord));
  painter->drawLine(vectorToPoint(rEnd), vectorToPoint(rEnd+rCord));
  
  /*
   if(isSelected()) {
      painter->setRenderHint(QPainter::Antialiasing, true);
      painter->setPen(QPen(SELECTION_COLOR, 0, Qt::DashLine));
      double m = SELECTION_MARGIN / currentViewScale();
      painter->drawRect(QRectF(-m, -_radius-m, _rnorm+m*2,  (_radius+m)*2));
    }*/
}

void PulleyCordGraphicsItem::worldDataChanged(bool dynamicOnly)
{
  Q_UNUSED(dynamicOnly)
  setPos(vectorToPoint(pulleyCord()->position1()));
  viewScaleChanged();
  update();
}


void PulleyCordGraphicsItem::stateChanged()
{
  /*if(_isSelected) {
    _handler1->setVisible(true);
    _handler2->setVisible(true);
  }
  else {
    _handler1->setVisible(false);
    _handler2->setVisible(false);
  }*/
  viewScaleChanged();
  update();
}
/*
void PulleyCordGraphicsItem::mouseSetPos(const QPointF& /*pos, const QPointF& diff, MovingState)
{
  _worldModel->simulationPause();
  
  if(pulleyCord()->body1()) {
    Q_ASSERT(spring()->body1()->metaObject()->inherits<StepCore::Item>());
    WorldGraphicsItem* gItem = static_cast<WorldScene*>(
      scene())->graphicsFromItem(static_cast<StepCore::Item*>(spring()->body1()));
      Q_ASSERT(gItem != NULL);
      if(!gItem->isSelected()) {
	_worldModel->setProperty(_item, "localPosition1",
				 _item->metaObject()->property("position1")->readVariant(_item));
				 _worldModel->setProperty(_item, "body1",
                                 QVariant::fromValue<StepCore::Object*>(NULL), WorldModel::UndoNoMerge);
      }
  } else {
    _worldModel->setProperty(_item, "localPosition1", 
			     QVariant::fromValue( (spring()->position1() + pointToVector(diff)).eval() ));
  }
  
      if(spring()->body2()) {
	Q_ASSERT(spring()->body2()->metaObject()->inherits<StepCore::Item>());
	WorldGraphicsItem* gItem = static_cast<WorldScene*>(
	  scene())->graphicsFromItem(static_cast<StepCore::Item*>(pulleyCord()->body2()));
	  Q_ASSERT(gItem != NULL);
	  if(!gItem->isSelected()) {
	    _worldModel->setProperty(_item, "localPosition2",
				     _item->metaObject()->property("position2")->readVariant(_item));
				     _worldModel->setProperty(_item, "body2", QString(), WorldModel::UndoNoMerge);
	  }
      } else {
	_worldModel->setProperty(_item, "localPosition2",
				 QVariant::fromValue( (spring()->position2() + pointToVector(diff)).eval() ));
      }*/
//}

void PulleyCordGraphicsItem::viewScaleChanged()
{
  prepareGeometryChange();
  _painterPath = QPainterPath();
  _painterPath.setFillRule(Qt::WindingFill);
  
  double s = currentViewScale();
  double radius = pulleyCord()->radius();
  if(radius > 1/s) {
    _painterPath.addEllipse(-radius, -radius, 2*radius, 2*radius);
    //_painterPath = QMatrix().rotate(disk()->angle() * 180 / StepCore::Constants::Pi).map(_painterPath);
  } else {
    _painterPath.addEllipse(-1/s, -1/s, 2/s, 2/s);
  }
    _painterPath = QMatrix().map(_painterPath);
    _boundingRect = _painterPath.boundingRect();
  
  
}






