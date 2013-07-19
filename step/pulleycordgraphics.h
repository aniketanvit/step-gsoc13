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
  void mouseSetPos(const QPointF& pos, const QPointF& diff, MovingState);
  StepCore::PulleyCord* pulleyCord() const {
    return static_cast<StepCore::PulleyCord*>(_item); }  
  QPainterPath _painterPath;
  double _rnorm;
  double _rscale;
  double _radius;
  static void tryAttach(StepCore::Item* item, WorldScene* worldScene, const QPointF& pos, int num);
  PulleyCordHandlerGraphicsItem* _handler1;
  PulleyCordHandlerGraphicsItem* _handler2;
  friend class PulleyCordCreator;
};

#endif



