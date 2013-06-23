#include "pulleycord.h"
#include "particle.h"
#include "rigidbody.h"
#include "world.h"

#include <cmath>

namespace StepCore {

  STEPCORE_META_OBJECT(PulleyCord, 
    
  )
  
  STEPCORE_META_OBJECT(PulleyCordErrors
    
  )
  
PulleyCord::PulleyCord(Vector2d position, double radius, Item* body1, Item* body2) :
                     _position(position), _radius(radius)
{
  setBody1(body1);
  setBody1(body2);
  _cord1 = Vector2d(-1,0);
  _cord2 = Vector2d(-1,0);
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
      _cord1 = dc1/dc1.norm();
      return;
    }
    else if(body1->metaObject()->inherits<RigidBody>()) {
      _p1 = NULL;
      _r1 = static_cast<RigidBody*>(body1);
      Vector2d dc1(_position[0]+_radius, _position[1]);
      dc1 = _r1->position() - dc1;
      _cord1 = dc1/dc1.norm(); 
      return;
    }
  }
  _p1 = NULL;
  _r1 = NULL;
  _cord1.setZero();  
}

void PulleyCord::setBody2(Object* body2)
{
  if(body2)
  {
    if(body2->metaObject()->inherits<Particle>()) {
      _p2 = static_cast<Particle*>(body1);
      _r2 = NULL;
      Vector2d dc1(_position[0]+_radius, _position[1]);
      dc2 = _p1->position() - dc1;
      _cord2 = dc2/dc2.norm();
      return;
    }
        else if(body2->metaObject()->inherits<RigidBody>()) {
	  _p2 = NULL;
	  _r2 = static_cast<RigidBody*>(body1);
	  Vector2d dc2(_position[0]-_radius, _position[1]);
	  dc2 = _r2->position() - dc2;
	  _cord2 = dc2/dc2.norm(); 
	  return;
	}
  }
    _p2 = NULL;
    _r2 = NULL;
    _cord2.setZero();  
}

void PulleyCord::calcForce(bool calcVariance)
{
  Vector2d leftForce(0,0);
  Vector2d rightForce(0,0);
  double systemMass = 0;
  
  if(_p1){
    leftForce = _p1->force().dot(_cord1);
    systemMass += _p1->mass();
  }
  else if(_r1){
    leftForce = _p2->force().dot(_cord1);
    systemMass += _r2->mass();
  }
  
  if(_p2){
    rightForce = _p2->force().dot(_cord2);
    systemMass += _p2->mass();
  }
  else if(_r2){
    rightForce = _r2->force().dot(_cord2);
    systemMass += _r2->mass();
  }
  
  _tension = leftForce-rightForce/systemMass;

  if(_p1)
    p1->applyForce(_tension*(_cord1));
  else if(_r1)
    _r1->applyForce(_tension*(_cord1), _localPosition1);
  
  if(_p2)
    _p2->applyForce(_tension*(_cord2));
  else if(_r2)
    _r2->applyForce(_tension*(_cord2), _localPosition2); 
  
}



}




