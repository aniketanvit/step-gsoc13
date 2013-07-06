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
    qDebug()<<" pulley created"<<endl;
    _worldModel->selectionModel()->setCurrentIndex(_worldModel->objectIndex(_item),
						   QItemSelectionModel::ClearAndSelect);
    _worldModel->addItem(_item); qDebug()<<" pulley added to world"<<endl;
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
  setZValue(HANDLER_ZVALUE);
  setExclusiveMoving(true);
  setExclusiveMovingMessage(i18n("Move cord of %1", _item->name()));
  double radius = (pulleyCord()->radius());
  if(_num == 1){
    setPos(-(radius)*cos(pulleyCord()->angle()), radius*sin(pulleyCord()->angle()));
  }else if(_num == 2){
    setPos((radius)*cos(pulleyCord()->angle()), -radius*sin(pulleyCord()->angle()));
  }
}

void PulleyCordHandlerGraphicsItem::viewScaleChanged()
{
  prepareGeometryChange();
  double w = HANDLER_SIZE/currentViewScale()/2;
  _boundingRect = QRectF(-w, -w, w*2, w*3);
}

void PulleyCordHandlerGraphicsItem::worldDataChanged(bool)
{
 if(_num == 1){
    setPos(vectorToPoint(static_cast<StepCore::PulleyCord*>(_item)->position1()));
 }else if(_num == 2) {
    setPos(vectorToPoint(static_cast<StepCore::PulleyCord*>(_item)->position2()));
 }
                        
}

void PulleyCordHandlerGraphicsItem::mouseSetPos(const QPointF& pos, const QPointF& diff, MovingState movingState)
{
  static_cast<WorldScene*>(scene())->snapItem(parentItem()->mapToParent(pos),
			   WorldScene::SnapParticle | WorldScene::SnapRigidBody |
			   WorldScene::SnapSetLocalPosition, 0, movingState, _item, _num);
  
}

PulleyCordGraphicsItem::PulleyCordGraphicsItem(StepCore::Item* item, WorldModel* worldModel)
                           : WorldGraphicsItem(item, worldModel)
{
  Q_ASSERT(dynamic_cast<StepCore::PulleyCord*>(_item) != NULL);
  setFlag(QGraphicsItem::ItemIsSelectable);
  setFlag(QGraphicsItem::ItemIsMovable);
  setZValue(JOINT_ZVALUE);
}

QPainterPath PulleyCordGraphicsItem::shape() const
{
  return _painterPath;
}

void PulleyCordGraphicsItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* /*option*/, QWidget* /*widget*/)
{
  //painter->setPen(QPen(Qt::blue));
  QColor color(Qt::blue);
  painter->setPen(Qt::red);
  painter->setBrush(QBrush(color));
  double radius = pulleyCord()->radius();
  painter->drawEllipse(QRectF(-radius, -radius, 2*radius, 2*radius));
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
  //setPos(vectorToPoint(pulleyCord()->position()));
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

void PulleyCordGraphicsItem::mouseSetPos(const QPointF& pos, const QPointF& diff, MovingState)
{/*
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
}

void PulleyCordGraphicsItem::viewScaleChanged()
{
  prepareGeometryChange();
  _painterPath = QPainterPath();
  _painterPath.setFillRule(Qt::WindingFill);
  
  double s = currentViewScale();
  double radius = pulleyCord()->radius()/s;
  QPointF pos = WorldGraphicsItem::vectorToPoint(pulleyCord()->position());
  
    _painterPath.addEllipse(-radius, -radius, 2*radius, 2*radius);
    _boundingRect = QRectF(pos.x()-radius, pos.y()-radius, 2*radius, 2*radius).normalized();
    _boundingRect.adjust(0.2,0.2,0.2,0.2);
  
  
}






