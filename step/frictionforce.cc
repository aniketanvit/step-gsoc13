/* This file is part of Step.
 * Copyright (C) 2013 Aniket Anvit <seeanvit@gmail.com>
 * 
 * Step is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * Step is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Step; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

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
 {
   double f = _createFrictionForceUi->_frictionLineEdit->text().toDouble();
   double r = _createFrictionForceUi->_restitutionLineEdit->text().toDouble();
  
   StepCore::GJKCollisionSolver* cs = dynamic_cast<StepCore::GJKCollisionSolver*>(_worldModel->world()->collisionSolver());
   
   StepCore::RigidBody* _b1 = static_cast<StepCore::RigidBody*>(_worldModel->item(_worldModel->selectionModel()->selectedIndexes()[0]));
   StepCore::RigidBody* _b2 = static_cast<StepCore::RigidBody*>(_worldModel->item(_worldModel->selectionModel()->selectedIndexes()[1]));
   
   StepCore::ContactValueList::const_iterator it_end = cs->_contacts.end();
   StepCore::ContactValueList::iterator it_begin = cs->_contacts.begin();
   
   for(it_begin; it_begin != it_end; it_begin++)
   {
     if((_b1->variablesOffset() == (*it_begin).body0->variablesOffset()) || (_b1->variablesOffset() == (*it_begin).body1->variablesOffset())){
       if((_b2->variablesOffset() == (*it_begin).body0->variablesOffset()) || (_b2->variablesOffset() == (*it_begin).body0->variablesOffset()))
       {
	 QPair<int, int> keyPair1 = qMakePair(_b1->variablesOffset(), _b2->variablesOffset());
	 QPair<int, int> keyPair2 = qMakePair(_b2->variablesOffset(), _b1->variablesOffset());
	 QPair<double, double> valuePair = qMakePair(f, r);
	 //(*it_begin).setf(f);
	 //(*it_begin).setr(r);
	 _worldModel->world()->collisionSolver()->fhash.insert(keyPair1, valuePair);
	 _worldModel->world()->collisionSolver()->fhash.insert(keyPair2, valuePair);
       }
       
    }
   }
   return true;
 }
 