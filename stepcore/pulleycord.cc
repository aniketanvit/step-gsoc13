#include "pulleycord.h"
#include "particle.h"
#include "rigidbody.h"
#include "world.h"
#include "types.h"
#include "object.h"
#include "vector.h"
#include <QtGlobal>
#include <cmath>
//#include <float.h>
namespace StepCore {

  STEPCORE_META_OBJECT(PulleyCord, QT_TRANSLATE_NOOP("ObjectClass", "PulleyCord"), QT_TR_NOOP("Massless Pulley with a cord"), 0, 
     STEPCORE_SUPER_CLASS(Item) STEPCORE_SUPER_CLASS(Joint),
     STEPCORE_PROPERTY_RW(StepCore::Vector2d, position, QT_TRANSLATE_NOOP("PropertyName", "position"), QT_TRANSLATE_NOOP("Units", "m"), QT_TR_NOOP("Position of the Pulley-Cord System"), position, setPosition)
     STEPCORE_PROPERTY_RW(double, radius, QT_TRANSLATE_NOOP("PropertyName", "radius"), QT_TRANSLATE_NOOP("Units", "m"), QT_TR_NOOP("Radius of the Pulley"), radius, setRadius)
     STEPCORE_PROPERTY_RW(double, lengthOfCord, QT_TRANSLATE_NOOP("PropertyName", "lengthOfCord"), QT_TRANSLATE_NOOP("Units", "m"), QT_TR_NOOP("Length of Cord"), lengthOfCord, setLengthOfCord)
     STEPCORE_PROPERTY_RW(Object*, body1, QT_TRANSLATE_NOOP("PropertyName", "body1"), STEPCORE_UNITS_NULL, QT_TR_NOOP("Body1"), body1, setBody1)
     STEPCORE_PROPERTY_RW(Object*, body2, QT_TRANSLATE_NOOP("PropertyName", "body2"), STEPCORE_UNITS_NULL, QT_TR_NOOP("Body2"), body2, setBody2)
     STEPCORE_PROPERTY_RW(StepCore::Vector2d, localPosition1, QT_TRANSLATE_NOOP("PropertyName", "localPosition1"), QT_TRANSLATE_NOOP("Units", "m"),
		       QT_TR_NOOP("Local position 1"), localPosition1, setLocalPosition1)
     STEPCORE_PROPERTY_RW(StepCore::Vector2d, localPosition2, QT_TRANSLATE_NOOP("PropertyName", "localPosition2"), QT_TRANSLATE_NOOP("Units", "m"),
		       QT_TR_NOOP("Local position 2"), localPosition2, setLocalPosition2)
     STEPCORE_PROPERTY_R_D(StepCore::Vector2d, position1, QT_TRANSLATE_NOOP("PropertyName", "position1"), QT_TRANSLATE_NOOP("Units", "m"), QT_TR_NOOP("Position1"), position1)
     STEPCORE_PROPERTY_R_D(StepCore::Vector2d, position2, QT_TRANSLATE_NOOP("PropertyName", "position2"), QT_TRANSLATE_NOOP("Units", "m"), QT_TR_NOOP("Position2"), position2)
     )  
 
PulleyCord::PulleyCord(Vector2d position, double radius, Item* body1, Item* body2) :
                     _position(position), _radius(radius), _body1(body1), _body2(body2), _lengthOfCord(0),
                     _tension(0), _inTension(false), _localPosition1(0,0), _localPosition2(0,0),
                     _p1(0), _p2(0), _r1(0), _r2(0)
{
  double x1 = _radius;
  double y1 = Vector2d(_position-position1()).squaredNorm() - pow(_radius, 2);
  y1 = sqrt(y1);
  double angle1 = atan2(x1, y1);
  double x2 = _radius;
  double y2 = Vector2d(_position-position2()).squaredNorm() - pow(_radius, 2);
  y2 = sqrt(y2);
  //end1 = Vector2d(_position[0] - _radius + cos(angle1), sin(angle1));
  //end2 = Vector2d(_position[1] + _radius - cos(angle1), sin(angle1));

  end1 = Vector2d(_position[0] - _radius, _position[1]);
  end2 = Vector2d(_position[0] + _radius, _position[1]);
  
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

int PulleyCord::constraintsCount()
{
  if(_body1 && _body2) 
    return 2;
  // XXX  what value should we return here ?
  else 
    return 0;
}

void PulleyCord::getConstraintsInfo(ConstraintsInfo* info, int offset)
{
  double l1 = 0, l2 = 0, tmass = 0;
  
  Vector2d d1 = position1() - end1;
  Vector2d d2 = position2() - end2;
  d1 = d1/d1.norm();
  d2 = d2/d2.norm();
  
  l1 = d1.norm();
  l2 = d2.norm();
  Vector2d v1(0,0), v2(0,0), f1(0,0), f2(0,0);
  
  if(_r1) {
    v1 = _r1->velocity();
    f1 = _r1->force();
    tmass += _r1->mass();
  }
  else if(_p1) {
    v1 = _p1->velocity();
    f1 = _p1->force();
    tmass += _p1->mass();
  }
  
  if(_r2) {
    v2 = _r2->velocity();
    f2 = _r2->force();
    tmass += _r2->mass();
  }
  else if(_p2) {
    v2 = _p2->velocity();
    f2 = _p2->force();
    tmass += _p2->mass();
  }
  
  if(d1.dot(v1) > 0){
    if(d1.dot(v1) - d2.dot(v2) >0)
      _inTension = true;
  }
  else if (d1.dot(f1) > 0) {
    if(d1.dot(f1) - d2.dot(f2) > 0)
      _inTension = true;
  }
  
  if(_inTension) {
    
  
  double factor = fabs(f1.norm() - f2.norm())/(tmass);
    
  info->value[offset] = ( l1 + l2 - _lengthOfCord );
  info->derivative[offset] = 0 ;
  
  if(_r1){
       
    info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset)   = (d1[0]);
    info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset+1) = (d1[1]);
    info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::AngleOffset)      =  0;
    
    info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset)
    = ((1+factor)* d1[0] );
    info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset+1)
    = ((1+factor)* d1[1]);
    info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::AngleOffset)      = 0;
        
  }
  else if(_p1) {
    info->jacobian.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset)     =   (d1[0]);
    info->jacobian.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset+1)   =   (d1[1]);
    
    info->jacobianDerivative.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset)
    = ((1+factor)* d1[0]);
    info->jacobianDerivative.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset+1)
    = ((1+factor)* d1[1]);
   
  }
  
  if(_r2) {
    info->jacobian.coeffRef(offset, _r2->variablesOffset() + RigidBody::PositionOffset) = d2[0];
    info->jacobian.coeffRef(offset, _r2->variablesOffset() + RigidBody::PositionOffset+1) = d2[1];
    info->jacobian.coeffRef(offset, _r2->variablesOffset() + RigidBody::AngleOffset)      = 0;
    
    info->jacobianDerivative.coeffRef(offset, _r2->variablesOffset() + RigidBody::PositionOffset)  = ((1-factor)*d1[0]);
    info->jacobianDerivative.coeffRef(offset, _r2->variablesOffset() + RigidBody::PositionOffset+1) =((1-factor)*d1[1]);
    info->jacobianDerivative.coeffRef(offset, _r2->variablesOffset() + RigidBody::AngleOffset)     = 0 ;
  }
  else if(_p2) {
    info->jacobian.coeffRef(offset, _p2->variablesOffset() + Particle::PositionOffset)    =  d2[0];
    info->jacobian.coeffRef(offset, _p2->variablesOffset() + Particle::PositionOffset+1)  =  d2[1];
    
    info->jacobianDerivative.coeffRef(offset, _p2->variablesOffset() + Particle::PositionOffset)  = ((1-factor)*d1[0]); 
    info->jacobianDerivative.coeffRef(offset, _p2->variablesOffset() + Particle::PositionOffset+1) = ((1-factor)* d1[1]);
    
  }
  }
  
}

Vector2d PulleyCord::velocity1() const
{
  if(_p1) return _p1->velocity();
  else if(_r1) return _r1->velocityLocal(_localPosition1);
  else return Vector2d(0,0);
}

Vector2d PulleyCord::velocity2() const
{
  if(_p1) return _p1->velocity();
  else if(_r1) return _r1->velocityLocal(_localPosition1);
  else return Vector2d(0,0);
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

}




