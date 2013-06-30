#ifndef PULLEYCORD_H
#define PULLEYCORD_H

#include "object.h"
#include "world.h"
#include "types.h"
#include "vector.h"
#include "particle.h"
#include "rigidbody.h"
#include "joint.h"

namespace StepCore {
  
/*
class PulleyCord;  
class PulleyCordErrors : public ObjectErrors
{
  STEPCORE_OBJECT(PulleyCordErrors)
public:
  PulleyCordErrors(Item* owner = NULL) : ObjectErrors(owner), 
              _localPosition1Variance(0), _localPosition2Variance(0),  _tensionVariance(0){}
	    
  ~PulleyCordErrors(){}
  
  PulleyCord* pulleyCord();
  /*
  Vector2d cord1Variance() const { return _cord1Variance; }
  void setCord1Variance(const Vector2d cord1Variance) { _cord1Variance = cord1Variance; }
  
  Vector2d cord2Variance() const { return _cord2Variance; }
  void setCord2Variance(const Vector2d cord2Variance) { _cord2Variance = cord2Variance; }
  
  Vector2d localPosition1Variance() const { return _localPosition1Variance; }
  void setLocalPosition1Variance(const Vector2d var) { _localPosition1Variance = var; }
  
  Vector2d localPosition2Variance() const { return _localPosition2Variance; }
  void setLocalPosition2Variance(const Vector2d var) { _localPosition2Variance = var; } 
  
  StepCore::Vector2d position1Variance() ;
  StepCore::Vector2d position2Variance() ;
  
  double tensionVariance();
    
protected:
  
  Vector2d _localPosition1Variance;
  Vector2d _localPosition2Variance;
  
  //StepCore::Vector2d _cord1Variance;
  //StepCore::Vector2d _cord2Variance;
  
  double _tensionVariance;
  
  friend class PulleyCord;
  
};
*/
class PulleyCord : public Item, public Joint
{
  STEPCORE_OBJECT(PulleyCord)
public:
   explicit PulleyCord(Vector2d position=Vector2d(0,0), double radius=1,Item* body1= NULL, Item* body2= NULL);
                      
  ~PulleyCord() {}
  
  int constraintsCount();
  void getConstraintsInfo(ConstraintsInfo* info, int offset);

  Vector2d position() const { return _position; }
  void setPosition(const Vector2d position) { _position = position; }
  
  Vector2d position1() const;
  Vector2d position2() const;
  
  double radius() const { return _radius; }
  void setRadius(const double radius) { _radius = radius; }
  /*
  Vector2d cord1() const { return _cord1; }
  void setCord1(const Vector2d cord1) { _cord1 = cord1/cord1.norm(); }
  
  Vector2d cord2() const { return _cord2; }
  void setCord2(const Vector2d cord2) { _cord2 = cord2/cord2.norm(); }
  
  double lengthCord1() const { return _lengthCord1; }
  void setLengthCord1(const double length) { _lengthCord1 = length; }
  
  double lengthCord2() const { return _lengthCord2 ; }
  void setLengthCord2(const double length) { _lengthCord2 = length; }
  */
  
  double lengthOfCord() const { return _lengthOfCord; }
  void setLengthOfCord(const double length) { _lengthOfCord = length; }
  
  double tension() const { return _tension; }
  void setTension(const double tension) { _tension = tension; }
  
  
  Vector2d localPosition1() const { return _localPosition1; }
  void setLocalPosition1(const Vector2d localPosition1) { _localPosition1 = localPosition1; }
  
  Vector2d localPosition2() const { return _localPosition2; }
  void setLocalPosition2(const Vector2d localPosition2) { _localPosition2 = localPosition2; }
  
  void setBody1(Object* body1);
  void setBody2(Object* body2);
  
  Object* body1() const { return _body1; }
  Object* body2() const { return _body1; }
  
  Particle* particle1() const { return _p1; }
  Particle* particle2() const { return _p2; }
  
  RigidBody* rigidBody1() const { return _r1; }
  RigidBody* rigidBody2() const { return _r2; }
  
  bool inTension();
protected:
  
  Vector2d _position;
  double _radius;
  double _angle; // so that the pulley-system can be fixed at an angle  
  
  Object* _body1;
  Object* _body2;
    
/*  Vector2d _cord1;
  double _lengthCord1;
  
  Vector2d _cord2;
  double _lengthCord2;*/
  
  double _lengthOfCord;
  
  double _tension;
  bool _inTension;
  
  Vector2d _localPosition1;
  Vector2d _localPosition2;
  
  Particle*  _p1;
  Particle*  _p2;
  RigidBody* _r1;
  RigidBody* _r2;
  
  friend class PulleyCordErrors;
  
};

}
#endif


