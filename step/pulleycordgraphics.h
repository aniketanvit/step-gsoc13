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
/*
class PulleyCordHandlerGraphicsItem : public WorldGraphicsItem
{
public:
  PulleyCordHandlerGraphicsItem(StepCore::Item* item, WorldModel* worldModel,
		    QGraphicsItem* parent, int num) ;
		    
  void mouseSetPos(const QPointF& pos, const QPointF&, MovingState movingState);
  void viewScaleChanged();
  void worldDataChanged(bool dynamicOnly);
  void mouseSetPos(QPointF pos, QPointF diff, MovingState state);
  int _num;
  
};
*/
class PulleyCordGraphicsItem : public WorldGraphicsItem
{
public:
  
  PulleyCordGraphicsItem(StepCore::Item* item, WorldModel* worldModel);
  
  QPainterPath shape() const;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
  StepCore::PulleyCord* pulleyCord() const {
    return static_cast<StepCore::PulleyCord*>(_item); }
  //void mouseSetPos(const QPointF& pos, const QPointF& diff, MovingState);
  void viewScaleChanged();
  void stateChanged();
  void worldDataChanged(bool dynamicOnly);
  
protected:
  
  QPainterPath _painterPath;
  double _rnorm;
  double _rscale;
  double _radius;
  
  //PulleyCordHandlerGraphicsItem* _handler1;
  //PulleyCordHandlerGraphicsItem* _handler2;
  friend class PulleyCordCreator;
};

#endif



