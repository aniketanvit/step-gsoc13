#include "forcefield.h"
#include "world.h"
#include "object.h"
#include "particle.h"

namespace StepCore {
  
STEPCORE_META_OBJECT(ForceFieldErrors, QT_TRANSLATE_NOOP("ObjectClass", ForceFieldErrors), QT_TR_NOOP("Errors Class for Force Field"), 0,
		     STEPCORE_SUPER_CLASS(ObjectErrors),)
  
STEPCORE_META_OBJECT(ForceField, QT_TRANSLATE_NOOP("ObjectClass", ForceField), QT_TR_NOOP("Force Force"), 0, STEPCORE_SUPER_CLASS(Item) 
STEPCORE_SUPER_CLASS(Force), )


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
      p1->applyForce(Vector2d(p1->charge()*p1->position()[1], p1->charge()*p1->position()[1]));
    
    if(calcVariances) {
      ChargedParticleErrors* pe1 = p1->chargedParticleErrors();
      pe1->applyForceVariance(Vector2d(pe1->positionVariance()[1], pe1->positionVariance()[1]));
    }
    }
}

}

}