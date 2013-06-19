/* This file is part of StepCore library.
   Copyright (C) 2007 Vladimir Kuznetsov <ks.vladimir@gmail.com>

   StepCore library is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   StepCore library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with StepCore; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "coulombforce.h"
#include "particle.h"
#include "rigidbody.h"

#include <cmath>
#include <QtGlobal>

namespace StepCore {

STEPCORE_META_OBJECT(CoulombForce, QT_TRANSLATE_NOOP("ObjectClass", "CoulombForce"), QT_TR_NOOP("Coulomb force"), 0,
    STEPCORE_SUPER_CLASS(Item) STEPCORE_SUPER_CLASS(Force),
    STEPCORE_PROPERTY_RW(double, coulombConst, QT_TRANSLATE_NOOP("PropertyName", "coulombConst"), STEPCORE_FROM_UTF8(QT_TRANSLATE_NOOP("Units", "N m²/C²")),
                QT_TR_NOOP("Coulomb constant"), coulombConst, setCoulombConst))

STEPCORE_META_OBJECT(CoulombForceErrors, QT_TRANSLATE_NOOP("ObjectClass", "CoulombForceErrors"), QT_TR_NOOP("Errors class for CoulombForce"), 0,
    STEPCORE_SUPER_CLASS(ObjectErrors),
    STEPCORE_PROPERTY_RW(double, coulombConstVariance, QT_TRANSLATE_NOOP("PropertyName", "coulombConstVariance"), STEPCORE_FROM_UTF8(QT_TRANSLATE_NOOP("Units", "N m²/C²")),
                QT_TR_NOOP("Coulomb constant variance"), coulombConstVariance, setCoulombConstVariance))

CoulombForce* CoulombForceErrors::coulombForce() const
{
    return static_cast<CoulombForce*>(owner());
}

CoulombForce::CoulombForce(double coulombConst)
    : _coulombConst(coulombConst)
{
    coulombForceErrors()->setCoulombConstVariance(
        square(Constants::CoulombError));
}

void CoulombForce::calcForce(bool calcVariances)
{
    const BodyList::const_iterator end = world()->bodies().end(); 
    for(BodyList::const_iterator b1 = world()->bodies().begin(); b1 != end; ++b1) {
       
        for(BodyList::const_iterator b2 = b1+1; b2 != end; ++b2) {
	  if((*b1)->metaObject()->inherits<Particle>()) {
	    if((*b2)->metaObject()->inherits<Particle>()) {
	      
	      Particle* p1 = static_cast<ChargedParticle*>(*b1);
	      Particle* p2 = static_cast<ChargedParticle*>(*b2);
	      
	      Vector2d r = p2->position() - p1->position();
	      double rnorm2 = r.squaredNorm();
	      Vector2d force = _coulombConst* p1->charge() * p2->charge() * r / (rnorm2*sqrt(rnorm2));
	      p2->applyForce(force);
	      force = -force;
	      p1->applyForce(force);
	      
	      if(calcVariances) {
		// XXX: CHECKME
		ChargedParticleErrors* pe1 = p1->chargedParticleErrors();
		ChargedParticleErrors* pe2 = p2->chargedParticleErrors();
		Vector2d rV = pe2->positionVariance() + pe1->positionVariance();
		Vector2d forceV = force.cwise().square().cwise()* (
		  Vector2d(coulombForceErrors()->_coulombConstVariance / square(_coulombConst) +
		  pe1->chargeVariance() / square(p1->charge()) +
		  pe2->chargeVariance() / square(p2->charge())) +
		  Vector2d(rV[0] * square(1/r[0] - 3*r[0]/rnorm2) + rV[1] * square(3*r[1]/rnorm2),
			   rV[1] * square(1/r[1] - 3*r[1]/rnorm2) + rV[0] * square(3*r[0]/rnorm2)));
		pe1->applyForceVariance(forceV);
		pe2->applyForceVariance(forceV);
	      }
	      
	    }
	    else if ((*b2)->metaObject()->inherits<RigidBody>()) {
	      Particle* p1 = static_cast<Particle*>(*b1);
	      RigidBody* r2 = static_cast<RigidBody*>(*b2);
	      
	      Vector2d r = r2->position() - p1->position();
	      double rnorm2 = r.squaredNorm();
	      Vector2d force = _coulombConst* p1->charge() * r2->charge() * r / (rnorm2*sqrt(rnorm2));
	      p1->applyForce(force);
	      force = -force;
	      r2->applyForce(force, r2->position);
	      
	      if(calcVariances) {
		// XXX: CHECKME
		ParticleErrors* pe1 = p1->ParticleErrors();
		RigidBodyErrors* re2 = r2->RigidBodyErrors();
		Vector2d rV = re2->positionVariance() + pe1->positionVariance();
		Vector2d forceV = force.cwise().square().cwise()* (
		  Vector2d(coulombForceErrors()->_coulombConstVariance / square(_coulombConst) +
		  pe1->chargeVariance() / square(p1->charge()) +
		  re2->chargeVariance() / square(r2->charge())) +
		  Vector2d(rV[0] * square(1/r[0] - 3*r[0]/rnorm2) + rV[1] * square(3*r[1]/rnorm2),
			   rV[1] * square(1/r[1] - 3*r[1]/rnorm2) + rV[0] * square(3*r[0]/rnorm2)));
		pe1->applyForceVariance(forceV);
		re2->applyForceVariance(force, r2->position(), forceV, re2->positionVariance());
	      }
	    
	    
	    
	    }
	 else if((*b1)->metaObject()->inherits<RigidBody>()) {
	   if((*b2)->metaObject()->inherits<Particle>()) {
	     
	     RigidBody* r1 = static_cast<RigidBody*>(*b1);
	     Particle* p2 = static_cast<Particle*>(*b2);
	     
	     Vector2d r = p2->position() - r1->position();
	     double rnorm2 = r.squaredNorm();
	     Vector2d force = _coulombConst* r1->charge() * p2->charge() * r / (rnorm2*sqrt(rnorm2));
	     r1->applyForce(force, r1->position());
	     force = -force;
	     r2->applyForce(force);
	     
	     if(calcVariances) {
	       // XXX: CHECKME
	       RigidBodyErrors* re1 = r1->ParticleErrors();
	       RigidBodyErrors* pe2 = p2->RigidBodyErrors();
	       Vector2d rV = re1->positionVariance() + pe2->positionVariance();
	       Vector2d forceV = force.cwise().square().cwise()* (
		 Vector2d(coulombForceErrors()->_coulombConstVariance / square(_coulombConst) +
		 pe1->chargeVariance() / square(p2->charge()) +
		 re2->chargeVariance() / square(r1->charge())) +
		 Vector2d(rV[0] * square(1/r[0] - 3*r[0]/rnorm2) + rV[1] * square(3*r[1]/rnorm2),
			  rV[1] * square(1/r[1] - 3*r[1]/rnorm2) + rV[0] * square(3*r[0]/rnorm2)));
	       pe1->applyForceVariance(forceV);
	       re1->applyForceVariance(-force, r1->position(), forceV, re1->positionVariance());
	     }
	      
	   }else if((*b2)->metaObject()->inherits<RigidBody>()) {
	     
	     RigidBody* r1 = static_cast<RigidBody*>(*b1);
	     RigidBody* r2 = static_cast<RigidBody*>(*b2);
	     
	     Vector2d r = p2->position() - p1->position();
	     double rnorm2 = r.squaredNorm();
	     Vector2d force = _coulombConst* p1->charge() * p2->charge() * r / (rnorm2*sqrt(rnorm2));
	     p2->applyForce(force);
	     force = -force;
	     p1->applyForce(force);
	     
	     if(calcVariances) {
	       // XXX: CHECKME
	       RigidBodyErrors* re1 = r1->RigidBodyErrors();
	       RigidBodyErrors* re2 = p2->RigidBodyErrors();
	       Vector2d rV = pe2->positionVariance() + pe1->positionVariance();
	       Vector2d forceV = force.cwise().square().cwise()* (
		 Vector2d(coulombForceErrors()->_coulombConstVariance / square(_coulombConst) +
		 re1->chargeVariance() / square(r1->charge()) +
		 re2->chargeVariance() / square(r2->charge())) +
		 Vector2d(rV[0] * square(1/r[0] - 3*r[0]/rnorm2) + rV[1] * square(3*r[1]/rnorm2),
			  rV[1] * square(1/r[1] - 3*r[1]/rnorm2) + rV[0] * square(3*r[0]/rnorm2)));
	       re1->applyForceVariance(-force, r1->position(), forceV, re1->positionVariance());
	       re2->applyForceVariance(force, r2->position(), forceV, re2->positionVariance()); 
	     
	   }
  
	    
	  }
	
    }
}

	}
    }
}
}/*
}
            if((*b2)->metaObject()->inherits<Particle>()) continue;
            ChargedParticle* p1 = static_cast<ChargedParticle*>(*b1);
            ChargedParticle* p2 = static_cast<ChargedParticle*>(*b2);

            Vector2d r = p2->position() - p1->position();
            double rnorm2 = r.squaredNorm();
            Vector2d force = _coulombConst* p1->charge() * p2->charge() * r / (rnorm2*sqrt(rnorm2));
            p2->applyForce(force);
            force = -force;
            p1->applyForce(force);

            if(calcVariances) {
                // XXX: CHECKME
                ChargedParticleErrors* pe1 = p1->chargedParticleErrors();
                ChargedParticleErrors* pe2 = p2->chargedParticleErrors();
                Vector2d rV = pe2->positionVariance() + pe1->positionVariance();
                Vector2d forceV = force.cwise().square().cwise()* (
                        Vector2d(coulombForceErrors()->_coulombConstVariance / square(_coulombConst) +
                                 pe1->chargeVariance() / square(p1->charge()) +
                                 pe2->chargeVariance() / square(p2->charge())) +
                        Vector2d(rV[0] * square(1/r[0] - 3*r[0]/rnorm2) + rV[1] * square(3*r[1]/rnorm2),
                                 rV[1] * square(1/r[1] - 3*r[1]/rnorm2) + rV[0] * square(3*r[0]/rnorm2)));
                pe1->applyForceVariance(forceV);
                pe2->applyForceVariance(forceV);
            }
        }
    }
}

} // namespace StepCore
*/
