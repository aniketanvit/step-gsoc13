#ifndef FRICTIONFORCE_H
#define FRICTIONFORCE_H

#include "stepcore/world.h"
#include "worldmodel.h"
#include "worldscene.h"
#include <kaction.h>
#include "worldgraphics.h"

namespace Ui {
 class WidgetCreateFrictionForce;
}

class FrictionForceKDialog;
class FrictionForceMenuHandler : public QObject
{
  Q_OBJECT
  
public:
  
  FrictionForceMenuHandler(WorldModel* worldModel, QObject* parent= 0) :
  QObject(parent), _worldModel(worldModel){}
  
public:  
  void arrangeWidgets();
  
protected slots :
  bool fillValues();

protected:
  Ui::WidgetCreateFrictionForce*  _createFrictionForceUi;
  FrictionForceKDialog*           _createFrictionForceDialog;
  
  WorldModel* _worldModel;
  friend class FrictionForceKDialog;
 // friend class FrictionForceCreator;
};

#endif