#include "pulleycord.h"
#include "particle.h"
#include "rigidbody.h"
#include "world.h"
#include "types.h"
#include "object.h"

#include <QtGlobal>
#include <cmath>

namespace StepCore {

  STEPCORE_META_OBJECT(PulleyCord, QT_TRANSLATE_NOOP("ObjectClass", "PulleyCord"), QT_TR_NOOP("Massless Pulley with a cord which can attach to bodies"), 0, 
     STEPCORE_SUPER_CLASS(Item) STEPCORE_SUPER_CLASS(Force),
     STEPCORE_PROPERTY_RW(StepCore::Vector2d, position, QT_TRANSLATE_NOOP("PropertyName", "position"), QT_TRANSLATE_NOOP("Units", "m"), QT_TR_NOOP("Position of the Pulley-Cord System"), position, setPosition)
     STEPCORE_PROPERTY_RW(double, radius, QT_TRANSLATE_NOOP("PropertyName", "radius"), QT_TRANSLATE_NOOP("Units", "m"), QT_TR_NOOP("Radius of the Pulley"), radius, setRadius)
     STEPCORE_PROPERTY_RW(double, lengthOfCord, QT_TRANSLATE_NOOP("PropertyName", "lengthOfCord"), QT_TRANSLATE_NOOP("Units", "m"), QT_TR_NOOP("Length of Cord"), lengthOfCord, setLengthOfCord)
     STEPCORE_PROPERTY_RW(Object*, body1, QT_TRANSLATE_NOOP("PropertyName", "body1"), STEPCORE_UNITS_NULL, QT_TR_NOOP("Body1"), body1, setBody1)
     STEPCORE_PROPERTY_RW(Object*, body2, QT_TRANSLATE_NOOP("PropertyName", "body2"), STEPCORE_UNITS_NULL, QT_TR_NOOP("Body2"), body2, setBody2)
     STEPCORE_PROPERTY_RW(StepCore::Vector2d, localPosition1, QT_TRANSLATE_NOOP("PropertyName", "localPosition1"), QT_TRANSLATE_NOOP("Units", "m"),
		       QT_TR_NOOP("Local position 1 of Cord1 on body1"), localPosition1, setLocalPosition1)
     STEPCORE_PROPERTY_RW(StepCore::Vector2d, localPosition2, QT_TRANSLATE_NOOP("PropertyName", "localPosition2"), QT_TRANSLATE_NOOP("Units", "m"),
		       QT_TR_NOOP("Local position 2 of Cord2 on Body2"), localPosition2, setLocalPosition2)
     //STEPCORE_PROPERTY_R_D(double, tension, QT_TRANSLATE_NOOP("PropertyName", "tension"), QT_TRANSLATE_NOOP("Units", "N"), QT_TR_NOOP("Tension in the Cords"), tension)
  )  
  
  STEPCORE_META_OBJECT(PulleyCordErrors, QT_TRANSLATE_NOOP("ObjectClass", "PulleyCordErrors"), QT_TR_NOOP("Errors class for PulleyCord"), 0, 
     STEPCORE_SUPER_CLASS(ObjectErrors), 
     STEPCORE_PROPERTY_RW(StepCore::Vector2d, localPosition1Variance, QT_TRANSLATE_NOOP("PropertyName", "localPosition1Variance"), QT_TRANSLATE_NOOP("Units", "m"),
	               QT_TR_NOOP("Local position 1 variance"), localPosition1Variance, setLocalPosition1Variance)
     STEPCORE_PROPERTY_RW(StepCore::Vector2d, localPosition2Variance, QT_TRANSLATE_NOOP("PropertyName", "localPosition2Variance"), QT_TRANSLATE_NOOP("Units", "m"),
	               QT_TR_NOOP("Local position 2 variance"), localPosition2Variance, setLocalPosition2Variance)
     /*STEPCORE_PROPERTY_R_D(StepCore::Vector2d, position1Variance, QT_TRANSLATE_NOOP("PropertyName", "position1Variance"), QT_TRANSLATE_NOOP("Units", "m"),
			   QT_TR_NOOP("Position 2 variance"), position1Variance)
     STEPCORE_PROPERTY_R_D(StepCore::Vector2d, position2Variance, QT_TRANSLATE_NOOP("PropertyName", "position2Variance"), QT_TRANSLATE_NOOP("Units", "m"),
			   QT_TR_NOOP("Position 2 variance"), position2Variance)*/
  )
  
  
PulleyCord* PulleyCordErrors::pulleyCord()
{
  return static_cast<PulleyCord*>(owner());
}

StepCore::Vector2d PulleyCordErrors::position1Variance()
{
  if(pulleyCord()->_p1) 
    return pulleyCord()->_p1->particleErrors()->positionVariance() + _localPosition1Variance;
  else if(pulleyCord()->_r1)
    return pulleyCord()->_r1->rigidBodyErrors()->positionVariance() + _localPosition1Variance;
  
  return _localPosition1Variance;
}

StepCore::Vector2d PulleyCordErrors::position2Variance()
{
  if(pulleyCord()->_p2) 
    return pulleyCord()->_p2->particleErrors()->positionVariance() + _localPosition2Variance;
  else if(pulleyCord()->_r2)
    return pulleyCord()->_r2->rigidBodyErrors()->positionVariance() + _localPosition2Variance;
  
  return _localPosition2Variance;
}

PulleyCord::PulleyCord(Vector2d position, double radius, Item* body1, Item* body2) :
                     _position(position), _radius(radius)
{
  setBody1(body1);
  setBody1(body2);
  _localPosition1.setZero();
  _localPosition2.setZero();
  _lengthOfCord = 4;
}

void PulleyCord::setBody1(Object* body1)
{
  if(body1)
  {
    if(body1->metaObject()->inherits<Particle>()) {
      _p1 = static_cast<Particle*>(body1);
      _r1 = NULL;
      Vector2d dc1(_position[0]+_radius, _position[1]);
      dc1 = _p1->position() - dc1;
      _lengthOfCord += dc1.norm();
      return;
    }
    else if(body1->metaObject()->inherits<RigidBody>()) {
      _p1 = NULL;
      _r1 = static_cast<RigidBody*>(body1);
      Vector2d dc1(_position[0]+_radius, _position[1]);
      dc1 = _r1->position() - dc1;
      _lengthOfCord += dc1.norm(); 
      return;
    }
  }
  _p1 = NULL;
  _r1 = NULL; 
}

void PulleyCord::setBody2(Object* body2)
{
  if(body2)
  {
    if(body2->metaObject()->inherits<Particle>()) {
      _p2 = static_cast<Particle*>(body2);
      _r2 = NULL;
      Vector2d dc2(_position[0]+_radius, _position[1]);
      dc2 = _p1->position() - dc2;
      _lengthOfCord += dc2.norm();
      return;
    }
        else if(body2->metaObject()->inherits<RigidBody>()) {
	  _p2 = NULL;
	  _r2 = static_cast<RigidBody*>(body2);
	  Vector2d dc2(_position[0]-_radius, _position[1]);
	  dc2 = _r2->position() - dc2;
	  _lengthOfCord += dc2.norm(); 
	  return;
	}
  }
    _p2 = NULL;
    _r2 = NULL;  
}

void PulleyCord::calcForce(bool calcVariance)
{
  double leftForce = 0;
  double rightForce = 0;
  double systemMass = 0;
  double leftLength = 0;
  double rightLength = 0;
  Vector2d leftPoint(position()[0]-radius(), position()[1]);
  Vector2d rightPoint(position()[1]+radius(), position()[1]);
  Vector2d vLeft, vRight;
  double vLeftAlongCord=0, vRightAlongCord=0;
  double massLeft=0, massRight=0;
  
  //first check if sum of leftLength and rightLegth is greater than the totalLength i.e. if the cord is _inTension
  leftLength = Vector2d(leftPoint - position1()).norm();
  rightLength = Vector2d(rightPoint - position2()).norm();
  
  if( leftLength+rightLength - _lengthOfCord > 0 ) // XXX should we put some epsilon here in place of 0 ?
       _inTension = true;
  else _inTension = false;
  
  if(!_inTension) return;
    // now we should equate the components of velocities of both the bodies
    // attached to both the ends of the along the cord
  // kind of like momentum-conservation:
    // the body with smaller mass will get pulled towards the body with greater mass if it is moving 
    // then finally apply tension on the bodies
    // tension is like a counter force , it develops only in response to some already acting force
    
  
    if(_p1){
    vLeft = _p1->velocity();
    vLeftAlongCord = (_p1->velocity().dot(position1() - leftPoint));
    leftForce = _p1->force().dot(position1() - leftPoint);
    systemMass += _p1->mass();
    massLeft = _p1->mass();
  }
  else if(_r1){
    vLeft = _r1->velocity();
    vLeftAlongCord = (_r1->velocity().dot(position1() - leftPoint));
    leftForce = _r1->force().dot(position1() - leftPoint);
    systemMass += _r1->mass();
    massLeft = _r1->mass();
  }
  
  if(_p2){
    vRight = _p2->velocity();
    vRightAlongCord = _p2->velocity().dot(position2() - rightPoint);
    rightForce = _p2->force().dot(position1() - rightPoint);
    systemMass += _p2->mass();
    massRight = _p2->mass();
  }
  else if(_r2){
    vRight = _p2->velocity();
    vRightAlongCord = _p2->velocity().dot(position2() - rightPoint);
    rightForce = _r2->force().dot(position1() - rightPoint);
    systemMass += _r2->mass();
    massRight = _r2->mass();
  }
  
  // equate components of velocities along the cord here (but only if the bodies are ttying to move away from each other)
    double iv = (massLeft*vLeftAlongCord + massRight*vRightAlongCord)/(massLeft+massRight);
    Vector2d ld(position1() - leftPoint); 
    ld = ld/ld.norm();
    Vector2d rd(position2() - rightPoint);
    rd = rd/rd.norm();
    _tension = (leftForce-rightForce)/systemMass;
  if(vRightAlongCord > 0 && vLeftAlongCord > 0) // they are trying to move apart
  { 
      // conservation of momentum along the cord here
     if(_p1)
       _p1->setVelocity(_p1->velocity() + (-vLeftAlongCord + iv)*ld);
     else if(_r1)
       _r1->setVelocity(_r1->velocity() + (-vLeftAlongCord + iv)*ld);
    
     if(_p2)
       _p2->setVelocity(_p2->velocity() - (-vLeftAlongCord + iv)*rd);
     else if(_r2)
       _r2->setVelocity(_r2->velocity() - (-vLeftAlongCord + iv)*rd);
  }
    
  else if(vRightAlongCord > 0 && vLeftAlongCord < 0)
  {
    if(_p1)
      _p1->setVelocity(_p1->velocity() + (-vLeftAlongCord + iv)*ld);
    else if(_r1)
      _r1->setVelocity(_r1->velocity() + (-vLeftAlongCord + iv)*ld);
    
    if(_p2)
      _p2->setVelocity(_p2->velocity() - (-vLeftAlongCord + iv)*rd);
    else if(_r2)
      _r2->setVelocity(_r2->velocity() - (-vLeftAlongCord + iv)*rd);
  }
  else if(vRightAlongCord < 0 && vLeftAlongCord > 0)
  {
    if(_p1)
      _p1->setVelocity(_p1->velocity() + (-vLeftAlongCord + iv)*ld);
    else if(_r1)
      _r1->setVelocity(_r1->velocity() + (-vLeftAlongCord + iv)*ld);
    
    if(_p2)
      _p2->setVelocity(_p2->velocity() - (-vLeftAlongCord + iv)*rd);
    else if(_r2)
      _r2->setVelocity(_r2->velocity() - (-vLeftAlongCord + iv)*rd); 
  }
  
  // many more conditions are formed here
  // this is just one test-case
 //Apply proper tension force here
  
  
  if(calcVariance){
    // errors calculation to be done...
    
  }
  
}

Vector2d PulleyCord::position1() const
{
  if(_p1) return _p1->position() + _localPosition1;
  else if(_r1) return _r1->pointLocalToWorld(_localPosition1);
  else return _localPosition1;
}

Vector2d PulleyCord::position2() const
{
  if(_p2) return _p2->position() + _localPosition2;
  else if(_r2) return _r2->pointLocalToWorld(_localPosition2);
  else return _localPosition2;
}

double PulleyCordErrors::tensionVariance()
{
  return 0;
}

}




