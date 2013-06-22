#include <KLocale>
#include "frictionforce.h"
#include "worldgraphics.h"
#include "worldmodel.h"
#include "worldscene.h"

#include "ui_create_friction_force.h"

#include <KDialog>
#include <QDoubleValidator>
#include <QValidator>
#include <QEvent>
#include "worldfactory.h"
#include <QItemSelectionModel>
#include <QGraphicsSceneMouseEvent>
#include <QApplication>
#include <KMessageBox>
#include <QGraphicsScene>

#include <float.h>

#include "frictionforce.moc"
using namespace Ui;



void FrictionForceCreator::start()
{
  showMessage(MessageFrame::Information,
	      i18n("Select the first out of two bodies to create a %1", classNameTr()));
  
  clickCount = 0;
}

bool FrictionForceCreator::sceneEvent(QEvent* event) 
{
  QGraphicsSceneMouseEvent* mouseEvent = static_cast<QGraphicsSceneMouseEvent*>(event);
  
  if(event->type() == QEvent::GraphicsSceneMousePress && mouseEvent->button() == Qt::LeftButton)
  {
    if(clickCount == 0) {
      
      _worldModel->simulationPause();
      _worldModel->beginMacro(i18n("Create %1", _worldModel->newItemName(_className)));
          
    //QPointF pos = mouseEvent->scenePos();
    QModelIndex firstIndex =   _worldModel->selectionModel()->currentIndex();
    StepCore::Item* firstItem = _worldModel->item(firstIndex);
    _worldModel->world()->_body1 = dynamic_cast<StepCore::Body*>(firstItem);
        
    clickCount = 1;
     
    return true;
    }
    else if (clickCount == 1){
      
    //QPointF pos = mouseEvent->scenePos();
    QModelIndex secondIndex = _worldModel->selectionModel()->currentIndex();
    StepCore::Item* secondItem = _worldModel->item(secondIndex);
    _worldModel->world()->_body2 = dynamic_cast<StepCore::Body*>(secondItem);
          
    FrictionForceMenuHandler* menuHandler = new FrictionForceMenuHandler(_item, _worldModel, NULL);
    menuHandler->setupDialog();
    menuHandler->deleteLater();
    
    _worldModel->endMacro();
    
    showMessage(MessageFrame::Information,
		i18n("%1 named '%2' created", classNameTr(), _item->name()),
		MessageFrame::CloseButton | MessageFrame::CloseTimer);
    
    setFinished();
    return true;
    }
    else
      return false ;
  
}
else 
  return 
    false;

}
 
 class FrictionForceKDialog : public KDialog
 {
 public:
   
   FrictionForceKDialog(FrictionForceMenuHandler* menuHandler, QWidget* parent = 0, Qt::WFlags flags=0):
            KDialog( parent, flags), _handler(menuHandler){}
	    
 public slots:
   void slotButtonClicked(int num)
   {
     if(num == KDialog::Ok)
     {
       if(_handler->createFrictionApply())
	 accept();
       else
	 KDialog::slotButtonClicked(num);
     }
   }
       
   FrictionForceMenuHandler* _handler;	     
	     
 };
 
 void FrictionForceMenuHandler::setupDialog()
 {
   if(_worldModel->isSimulationActive())
     _worldModel->simulationStop();
   
   _createFrictionForceDialog = new FrictionForceKDialog(this);
   _createFrictionForceDialog->setCaption(i18n("Create a Friction Force"));
     _createFrictionForceDialog->setButtons(KDialog::Ok | KDialog::Cancel);
     
     _createFrictionForceUi = new Ui::WidgetCreateFrictionForce;
     _createFrictionForceUi->setupUi(_createFrictionForceDialog);
     
     _createFrictionForceUi->_frictionLineEdit->setValidator(new QDoubleValidator(0, 1, DBL_DIG, _createFrictionForceUi->_frictionLineEdit ));
     _createFrictionForceUi->_restitutionLineEdit->setValidator( new QDoubleValidator(
       0,1, DBL_DIG, _createFrictionForceUi->_restitutionLineEdit));
     
     connect(_createFrictionForceDialog, SIGNAL(okClicked()), this, SLOT(createFrictionApply()));
     
     _createFrictionForceDialog->exec();
     
     delete _createFrictionForceDialog; 
     _createFrictionForceDialog = 0;
     delete _createFrictionForceUi;
     _createFrictionForceUi = 0;
     
 }
 
 
 bool FrictionForceMenuHandler::createFrictionApply()
 {/*
   _worldModel->world()->_frCoeff = _createFrictionForceUi->_frictionLineEdit->text().toDouble();
   _worldModel->world()->_rrCoeff = _createFrictionForceUi->_restitutionLineEdit->text().toDouble();
   
   _worldModel->world()->fillFrictionHash();*/
   return true;
   
   
 }
 
 
 
 
 
 
 
 