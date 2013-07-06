#include "pulleycord.h"
#include "particle.h"
#include "rigidbody.h"
#include "world.h"
#include "types.h"
#include "object.h"
#include "vector.h"
#include <QtGlobal>
#include <cmath>

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
 
PulleyCord::PulleyCord(Vector2d position, double radius, double angle) :
                     _position(position), _radius(radius), _angle(angle), _lengthOfCord(0),
                     _tension(0), _inTension(false), _localPosition1(Vector2d::Zero()), _localPosition2(Vector2d::Zero()),
                     _body1(0), _body2(0), _p1(0), _p2(0), _r1(0), _r2(0)
{
  double x1 = _radius;
  double y1 = Vector2d(_position-position1()).squaredNorm() - pow(_radius, 2);
  y1 = sqrt(y1);
  double angle1 = atan2(y1, x1);
  double x2 = _radius;
  double y2 = Vector2d(_position-position2()).squaredNorm() - pow(_radius, 2);
  y2 = sqrt(y2);
  end1 = Vector2d(_position[0] - _radius + cos(angle1), sin(angle1));
  end2 = Vector2d(_position[1] + _radius - cos(angle1), sin(angle1));
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
  if(!_body1 && !_body2) return 0;
  // XXX  what value should we return here ?
  return 2;
}

void PulleyCord::getConstraintsInfo(ConstraintsInfo* info, int offset)
{
  if(!body || !body2) return;
  
  Vector2d p1 = position2() - end2;
  Vector2d p2 = position1() - end1;
  
  p1 = p1/p1.norm();
  p1 = p2/p2.norm();
  
  Vector2d v1 = velocity1().dot(p1);
  Vector2d v2 = velocity2().dot(p2);
  
  info->value[offset  ] = p[0];
  info->value[offset+1] = p[1];
  
  info->derivative[offset  ] = v[0];
  info->derivative[offset+1] = v[1];
  
  if(_p1) {
    info->jacobian.coeffRef(offset  , _p1->variablesOffset() + Particle::PositionOffset)   = -1;
    info->jacobian.coeffRef(offset+1, _p1->variablesOffset() + Particle::PositionOffset+1) = -1;
    
  } else if(_r1) {
    Vector2d r1 = _r1->vectorLocalToWorld(_localPosition1);
    double   av = _r1->angularVelocity();
    
    info->jacobian.coeffRef(offset  , _r1->variablesOffset() + Particle::PositionOffset) =(   -1);
    info->jacobian.coeffRef(offset+1, _r1->variablesOffset() + Particle::PositionOffset+1) =( -1);
    
    info->jacobian.coeffRef(offset  , _r1->variablesOffset()+RigidBody::AngleOffset) =( +r1[1]);
    info->jacobian.coeffRef(offset+1, _r1->variablesOffset()+RigidBody::AngleOffset) =( -r1[0]);
    info->jacobianDerivative.coeffRef(offset  , _r1->variablesOffset()+RigidBody::AngleOffset) =( +av*r1[0]);
    info->jacobianDerivative.coeffRef(offset+1, _r1->variablesOffset()+RigidBody::AngleOffset) =( +av*r1[1]);
  }
  
          if(_p2) {
	    info->jacobian.coeffRef(offset  , _p2->variablesOffset() + Particle::PositionOffset) =(   1);
	    info->jacobian.coeffRef(offset+1, _p2->variablesOffset() + Particle::PositionOffset+1) =( 1);
	    
	  } else if(_r2) {
	    Vector2d r2 = _r2->vectorLocalToWorld(_localPosition2);
	    double   av = _r2->angularVelocity();
	    
	    info->jacobian.coeffRef(offset  , _r2->variablesOffset() + Particle::PositionOffset) =(   1);
	    info->jacobian.coeffRef(offset+1, _r2->variablesOffset() + Particle::PositionOffset+1) =( 1);
	    
	    info->jacobian.coeffRef(offset  , _r2->variablesOffset()+RigidBody::AngleOffset) =( -r2[1]);
	    info->jacobian.coeffRef(offset+1, _r2->variablesOffset()+RigidBody::AngleOffset) =( +r2[0]);
	    info->jacobianDerivative.coeffRef(offset  , _r2->variablesOffset()+RigidBody::AngleOffset) =( -av*r2[0]);
	    info->jacobianDerivative.coeffRef(offset+1, _r2->variablesOffset()+RigidBody::AngleOffset) =( -av*r2[1]);
	  }
}
  
  

Vector2d PulleyCord::velocity1() const
{
  if(_p1) return _p1->velocity();
  else if(_r1) return _r1->velocityLocal(_localPosition1);
  else return Vector2d::Zero();
}

Vector2d PulleyCord::velocity2() const
{
  if(_p1) return _p1->velocity();
  else if(_r1) return _r1->velocityLocal(_localPosition1);
  else return Vector2d::Zero();
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




