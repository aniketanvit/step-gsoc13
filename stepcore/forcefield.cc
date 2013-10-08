/* This file is part of StepCore library.
 * Copyright (C) 2013 Aniket Anvit <seeanvit@gmail.com>
 * 
 * StepCore library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * StepCore library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with StepCore; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "forcefield.h"
#include "world.h"
#include "object.h"
#include "particle.h"

namespace StepCore {
  
STEPCORE_META_OBJECT(ForceFieldErrors, QT_TRANSLATE_NOOP("ObjectClass", ForceFieldErrors), QT_TR_NOOP("Errors Class for Force Field"), 0,
		     STEPCORE_SUPER_CLASS(ObjectErrors),)
  
STEPCORE_META_OBJECT(ForceField, QT_TRANSLATE_NOOP("ObjectClass", ForceField), QT_TR_NOOP("Force Force"), 0, STEPCORE_SUPER_CLASS(Item) 
STEPCORE_SUPER_CLASS(Force),
  STEPCORE_PROPERTY_RW(double, b, QT_TRANSLATE_NOOP("PropertyName", "b"), STEPCORE_FROM_UTF8(QT_TRANSLATE_NOOP("Units", "Wb")), QT_TR_NOOP("Constant"),
                            b, setB)
)


ForceField* ForceFieldErrors::forceField() const
{
  return static_cast<ForceField*>(owner());
}

void ForceField::calcForce(bool calcVariances)
{
  const BodyList::const_iterator end = world()->bodies().end();
  for(BodyList::const_iterator b1 = world()->bodies().begin(); b1 != end; ++b1) {
    if(!(*b1)->metaObject()->inherits<ChargedParticle>()) continue;
    else {
      ChargedParticle* p1 = static_cast<ChargedParticle*>(*b1);
      Vector2d v = p1->velocity();
      double q = p1->charge();
      p1->applyForce(Vector2d(q*_b*v[1], -q*_b*v[0]));
    
    if(calcVariances) {
      //ChargedParticleErrors* pe1 = p1->chargedParticleErrors();
      //pe1->applyForceVariance(Vector2d(pe1->positionVariance()[1], pe1->positionVariance()[1]));
    }
    }
}

}

}
