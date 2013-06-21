#ifndef FRICTIONFORCE_H
#define FRICTIONFORCE_H

#include "stepcore/world.h"
#include "worldmodel.h"
#include "worldscene.h"
#include <kaction.h>
#include "worldgraphics.h"


class ItemCreator;

class FrictionForceCreator : public ItemCreator
{
public:
  FrictionForceCreator(const QString& className, WorldModel* worldModel, WorldScene* scene):
		   ItemCreator(className, worldModel, scene){}
  bool sceneEvent(QEvent* event);
  void start();
  
  ~FrictionForceCreator(){}
  
  int clickCount;
  
};

namespace Ui {
 class WidgetCreateFrictionForce;
}
class FrictionForceGraphicsItem : public WorldGraphicsItem
{
public:
  FrictionForceGraphicsItem(StepCore::Item* item, WorldModel* worldModel) : 
           WorldGraphicsItem(item, worldModel){}
  
  void viewScaleChanged(){}
  void stateChanged(){}
  void worldDataChanged(bool dynamicOnly){}
protected:
  
  void mouseSetPos(const QPointF& pos, const QPointF& diff, MovingState movingState){
    Q_UNUSED(pos);
    Q_UNUSED(diff);
    Q_UNUSED(movingState);
    
  }
};

class FrictionForceKDialog;

class FrictionForceMenuHandler : public ItemMenuHandler
{
  Q_OBJECT
  
public:
  
  FrictionForceMenuHandler(StepCore::Object* object, WorldModel* worldModel, QObject* parent= 0) :
         ItemMenuHandler(object, worldModel, parent){}
  ~FrictionForceMenuHandler(){}
  
public slots:
  
  void setupDialog();
  
protected slots :
  
  bool createFrictionApply();

protected:
  Ui::WidgetCreateFrictionForce*  _createFrictionForceUi;
  FrictionForceKDialog*           _createFrictionForceDialog;
  
  StepCore::Body* _body1;
  StepCore::Body* _body2;
  
  friend class FrictionForceKDialog;
 // friend class FrictionForceCreator;
};

#endif