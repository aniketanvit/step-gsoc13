#include <KLocale>
#include "frictionforce.h"
#include "worldgraphics.h"
#include "worldmodel.h"
#include "worldscene.h"
#include "stepcore/world.h"
#include "ui_create_friction_force.h"
#include "stepcore/rigidbody.h"

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
#include <QDebug>
#include <float.h>
#include "stepcore/collisionsolver.h"

using namespace Ui;
using namespace StepCore;

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
       if(_handler->fillValues())
	 accept();
       else
	 KDialog::slotButtonClicked(num);
     }
   }
       
   FrictionForceMenuHandler* _handler;	     
	     
 };
 
 void FrictionForceMenuHandler::arrangeWidgets()
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
     
     connect(_createFrictionForceDialog, SIGNAL(okClicked()), this, SLOT(fillValues()));
     
     _createFrictionForceDialog->exec();
     
     delete _createFrictionForceDialog; 
     _createFrictionForceDialog = 0;
     delete _createFrictionForceUi;
     _createFrictionForceUi = 0;
     
 }

 bool FrictionForceMenuHandler::fillValues()
 {std::cout<<"filling values"<<std::endl;
   double f = _createFrictionForceUi->_frictionLineEdit->text().toDouble();
   double r = _createFrictionForceUi->_restitutionLineEdit->text().toDouble();
  
   StepCore::GJKCollisionSolver* cs = dynamic_cast<StepCore::GJKCollisionSolver*>(_worldModel->world()->collisionSolver());
   
   StepCore::RigidBody* _b1 = static_cast<StepCore::RigidBody*>(_worldModel->item(_worldModel->selectionModel()->selectedIndexes()[0]));
   StepCore::RigidBody* _b2 = static_cast<StepCore::RigidBody*>(_worldModel->item(_worldModel->selectionModel()->selectedIndexes()[1]));
   
   StepCore::ContactValueList::const_iterator it_end = cs->_contacts.end();
   StepCore::ContactValueList::iterator it_begin = cs->_contacts.begin();
   
   for(it_begin; it_begin != it_end; it_begin++)
   {std::cout<<"in outer loop"<<std::endl;
     if((_b1->variablesOffset() == (*it_begin).body0->variablesOffset()) || (_b1->variablesOffset() == (*it_begin).body1->variablesOffset())){
       std::cout<<" one body present in this contact"<<_b1->variablesOffset()<<std::endl;
       if((_b2->variablesOffset() == (*it_begin).body0->variablesOffset()) || (_b2->variablesOffset() == (*it_begin).body0->variablesOffset()))
       {
	 QPair<int, int> keyPair1 = qMakePair(_b1->variablesOffset(), _b2->variablesOffset());
	 QPair<int, int> keyPair2 = qMakePair(_b2->variablesOffset(), _b1->variablesOffset());
	 QPair<double, double> valuePair = qMakePair(f, r);
	 //(*it_begin).setf(f);
	 //(*it_begin).setr(r);
	 _worldModel->world()->collisionSolver()->fhash.insert(keyPair1, valuePair);
	 _worldModel->world()->collisionSolver()->fhash.insert(keyPair2, valuePair);
	 std::cout<<"second body also present"<<_b1->variablesOffset() <<", "<<_b2->variablesOffset()<<std::endl;
	 std::cout<<f<<" , "<<r<<std::endl;
	 std::cout<<(*it_begin)._frictionCoefficient<<","<<(*it_begin)._restitutionCoefficient<<std::endl;
       }
       
    }
   }
   return true;
   
   
 }
 
 
 
 
 
 
 
 