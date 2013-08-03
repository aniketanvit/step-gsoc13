#include "pulleycord.h"
#include "particle.h"
#include "rigidbody.h"
#include "world.h"
#include "types.h"
#include "object.h"
#include "vector.h"
#include <QtGlobal>
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
                     _position(position), _radius(radius), _lengthOfCord(0),
                     _tension(0), _inTension(false), _localPosition1(0,0), _localPosition2(0,0),
                     _p1(0), _p2(0), _r1(0), _r2(0), _length1(0), _length2(0)
{
  setBody1(NULL);
  setBody2(NULL);
  end1 = Vector2d(_position[0] - _radius, _position[1]);
  end2 = Vector2d(_position[0] + _radius, _position[1]);
  
}


void PulleyCord::setBody1(Object* body1)
{
  if(body1)
  {
    if(body1->metaObject()->inherits<Particle>()) {
      _body1 = body1;
      _p1 = static_cast<Particle*>(body1);
      _r1 = NULL;
      Vector2d dc1 = position1() - end1;
      _lengthOfCord += dc1.norm();
      _length1 = dc1.norm();
      return;
    }
            else if(body1->metaObject()->inherits<RigidBody>()) {
	      _body1 = body1;
	      _p1 = NULL;
	      _r1 = static_cast<RigidBody*>(body1);
	      Vector2d dc1 = position1() - end1;
	      _lengthOfCord += dc1.norm();
	      _length1 = dc1.norm();
	      return;
	    }
  }
      _body1 = NULL;
      _p1 = NULL;
      _r1 = NULL; 
}

void PulleyCord::setBody2(Object* body2)
{
  if(body2)
  {
    if(body2->metaObject()->inherits<Particle>()) {
      _body2 = body2;
      _p2 = static_cast<Particle*>(body2);
      _r2 = NULL;
      Vector2d dc2 = position2() - end2;
      _lengthOfCord += dc2.norm();
      _length2 = dc2.norm();
      return;
    }
                else if(body2->metaObject()->inherits<RigidBody>()) {
		  _body2 = body2;
		  _p2 = NULL;
		  _r2 = static_cast<RigidBody*>(body2);
		  Vector2d dc2 = position2() - end2;
		  _lengthOfCord += dc2.norm(); 
		  _length2 = dc2.norm();
		  return;
		}
  }
      _body2 = NULL;
      _p2 = NULL;
      _r2 = NULL;  
}


int PulleyCord::constraintsCount()
{/*
  if(_body1 && _body2) 
  {
    Vector2d d1 = position1() - end1;
    Vector2d d2 = position2() - end2;
    
    Vector2d v1 = velocity1();
    Vector2d v2 = velocity2();
    
    if((d1.dot(v1) + d2.dot(v2)) > 10.0e-6){
      _inTension = true;
      return 1;
    }
  }
  _inTension = false;
  return 0;
  */
  if(_body1 && _body2)
  {
    if(_length1 <= _radius || _length2 <= _radius)
    {
      _inTension = true;
      return 6;
    }
    
    _inTension = true;
    return 1;
  }
  _inTension = false;
 return 0;
}

void PulleyCord::getConstraintsInfo(ConstraintsInfo* info, int offset)
{
  double l1 = 0, l2 = 0;
  Vector2d d1 = position1() - end1;
  Vector2d d2 = position2() - end2;
  
  l1 = d1.norm();
  l2 = d2.norm();
  
  d1 = d1/d1.norm();
  d2 = d2/d2.norm();
  
  Vector2d v1 = velocity1();
  Vector2d v2 = velocity2();
  
  if(_inTension) {
    if(_length1 <= _radius || _length2 <= _radius)
    {
      if(_p1) {
	info->value[offset  ] = _p1->position()[0] - _position[0];
	info->value[offset+1] = _p1->position()[1] - _position[1];
	
	info->derivative[offset  ] = _p1->velocity()[0];
	info->derivative[offset+1] = _p1->velocity()[1];
	
	info->jacobian.coeffRef(offset, _p1->variablesOffset()+Particle::PositionOffset) = 1;
	info->jacobian.coeffRef(offset+1, _p1->variablesOffset()+Particle::PositionOffset+1) = 1;
	
      } else if(_r1) {
	info->value[offset  ] = _r1->position()[0] - _position[0];
	info->value[offset+1] = _r1->position()[1] - _position[1];
	info->value[offset+2] = 0;
	
	info->derivative[offset  ] = _r1->velocity()[0];
	info->derivative[offset+1] = _r1->velocity()[1];
	info->derivative[offset+2] = _r1->angularVelocity();
	
	info->jacobian.coeffRef(offset, _r1->variablesOffset()+RigidBody::PositionOffset) = 1;
	info->jacobian.coeffRef(offset+1, _r1->variablesOffset()+RigidBody::PositionOffset+1) = 1;
	info->jacobian.coeffRef(offset+2, _r1->variablesOffset()+RigidBody::AngleOffset) = 1;
      }
      
            if(_p2) {
	      info->value[offset +3 ] = _p2->position()[0] - _position[0];
	      info->value[offset+4] = _p2->position()[1] - _position[1];
	      
	      info->derivative[offset +3 ] = _p2->velocity()[0];
	      info->derivative[offset+4] = _p2->velocity()[1];
	      
	      info->jacobian.coeffRef(offset+3, _p2->variablesOffset()+Particle::PositionOffset) = 1;
	      info->jacobian.coeffRef(offset+4, _p2->variablesOffset()+Particle::PositionOffset+1) = 1;
	      
	    } else if(_r2) {
	      info->value[offset +3 ] = _r2->position()[0] - _position[0];
	      info->value[offset+4] = _r2->position()[1] - _position[1];
	      info->value[offset+5] = 0;
	      
	      info->derivative[offset+3  ] = _r2->velocity()[0];
	      info->derivative[offset+4] = _r2->velocity()[1];
	      info->derivative[offset+5] = _r2->angularVelocity();
	      
	      info->jacobian.coeffRef(offset+3, _r2->variablesOffset()+RigidBody::PositionOffset) = 1;
	      info->jacobian.coeffRef(offset+4, _r2->variablesOffset()+RigidBody::PositionOffset+1) = 1;
	      info->jacobian.coeffRef(offset+5, _r2->variablesOffset()+RigidBody::AngleOffset) = 1;
	    }
    }
    /*
    info->value[offset] = ( l1 - _length1*_length1)*0.5;
    info->value[offset+1] = (l2 - _length2*_length2)*0.5;
    info->derivative[offset] =  (d1.dot(v1));
    info->derivative[offset+1] = (d2.dot(v2));
    
    if(_p1) {
      info->jacobian.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset) = (  -d1[0]);
      info->jacobian.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset+1) =( -d1[1]);
      
      info->jacobianDerivative.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset) = (-v1[0]);
      info->jacobianDerivative.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset+1) = ( -v1[1]);
      
    } else if(_r1) {
      //Vector2d r1 = _r1->vectorLocalToWorld(_localPosition1);
      
      info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset) =(   -d1[0]);
      info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset+1) =( -d1[1]);
      //info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::AngleOffset) =( +p[0]*r1[1] - p[1]*r1[0]);
      
      info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset) = (-v1[0]);
      info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset+1) =( -v1[1]);
      //info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::AngleOffset) =(
	//+ v[0]*r1[1] - v[1]*r1[0] + _r1->angularVelocity()*p.dot(r1));
    }
    
            if(_p2) {
	      info->jacobian.coeffRef(offset+1, _p2->variablesOffset() + Particle::PositionOffset) =(   -d2[0]);
	      info->jacobian.coeffRef(offset+1, _p2->variablesOffset() + Particle::PositionOffset+1) =( -d2[1]);
	      
	      info->jacobianDerivative.coeffRef(offset+1, _p2->variablesOffset() + Particle::PositionOffset) =(   -v2[0]);
	      info->jacobianDerivative.coeffRef(offset+1, _p2->variablesOffset() + Particle::PositionOffset+1) =( -v2[1]);
	      
	    } else if(_r2) {
	      //Vector2d r2 = _r2->vectorLocalToWorld(_localPosition2);
	      
	      info->jacobian.coeffRef(offset+1, _r2->variablesOffset() + RigidBody::PositionOffset) =(   -d2[0]);
	      info->jacobian.coeffRef(offset+1, _r2->variablesOffset() + RigidBody::PositionOffset+1) =( -d2[1]);
	     // info->jacobian.coeffRef(offset, _r2->variablesOffset() + RigidBody::AngleOffset) =( -p[0]*r2[1] + p[1]*r2[0]);
	      
	      info->jacobianDerivative.coeffRef(offset+1, _r2->variablesOffset() + RigidBody::PositionOffset) =(   -v2[0]);
	      info->jacobianDerivative.coeffRef(offset+1, _r2->variablesOffset() + RigidBody::PositionOffset+1) =( -v2[1]);
	      //info->jacobianDerivative.coeffRef(offset, _r2->variablesOffset() + RigidBody::AngleOffset) =(
		//- v[0]*r2[1] + v[1]*r2[0] - _r2->angularVelocity()*p.dot(r2));
	    }
  */ 
  info->value[offset] = ( l1 + l2 - _lengthOfCord )*0.25;
  info->derivative[offset] =  (-d1.dot(v1) - d2.dot(v2));
  
  if(_r1){
       
    Vector2d r1 = _r1->vectorLocalToWorld(_localPosition1);
    
    info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset)   = (-d1[0]);
    info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset+1) = (-d1[1]);
    //info->jacobian.coeffRef(offset, _r1->variablesOffset() + RigidBody::AngleOffset)      = 0; //(r1[0]*d1[1] - r1[1]*d1[0]);
    
    //info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset) = (v1[0]);
    //info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::PositionOffset+1) = (v1[1]);
    //info->jacobianDerivative.coeffRef(offset, _r1->variablesOffset() + RigidBody::AngleOffset)      = 0;
        
  }
  else if(_p1) {
    info->jacobian.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset)     =   (-d1[0]);
    info->jacobian.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset+1)   =   (-d1[1]);
    
    //info->jacobianDerivative.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset)  = (v1[0]);
    //info->jacobianDerivative.coeffRef(offset, _p1->variablesOffset() + Particle::PositionOffset+1) = (v1[1]);
   
  }
  
  if(_r2) {
    
    Vector2d r2 = _r2->vectorLocalToWorld(_localPosition2);
    
    info->jacobian.coeffRef(offset, _r2->variablesOffset() + RigidBody::PositionOffset) = (-d2[0]);
    info->jacobian.coeffRef(offset, _r2->variablesOffset() + RigidBody::PositionOffset+1) = (-d2[1]);
    //info->jacobian.coeffRef(offset, _r2->variablesOffset() + RigidBody::AngleOffset)      = 0;//(r2[0]*d1[1] - r2[1]*d1[0]);
    
    //info->jacobianDerivative.coeffRef(offset, _r2->variablesOffset() + RigidBody::PositionOffset)  = (v2[0]);
    //info->jacobianDerivative.coeffRef(offset, _r2->variablesOffset() + RigidBody::PositionOffset+1) = (v2[1]);
    //info->jacobianDerivative.coeffRef(offset, _r2->variablesOffset() + RigidBody::AngleOffset)     = (0) ;
  }
  else if(_p2) {
    info->jacobian.coeffRef(offset, _p2->variablesOffset() + Particle::PositionOffset)    =  (-d2[0]);
    info->jacobian.coeffRef(offset, _p2->variablesOffset() + Particle::PositionOffset+1)  =  (-d2[1]);
    
    //info->jacobianDerivative.coeffRef(offset, _p2->variablesOffset() + Particle::PositionOffset)  = (v2[0]); 
    //info->jacobianDerivative.coeffRef(offset, _p2->variablesOffset() + Particle::PositionOffset+1) = (v2[1]);
    
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




