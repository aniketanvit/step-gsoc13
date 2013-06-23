#ifndef PULLEYCORD_H
#define PULLEYCORD_H

#include "object.h"
#include "world.h"
#include "types.h"
#include "vector.h"

namespace StepCore {
  
class PulleyCordErrors : public ObjectErrors
{
  STEPCORE_OBJECT(PulleyCordErrors)
public:
  PulleyCordErrors(Item* owner = NULL) : ObjectErrors(owner), _tensionVariance(0), 
            _localPosition1Variance(0), _localPosition2Variance(0)
	    
  ~PulleyCordErrors(){}
  
  
  
  
  
protected:
  
  StepCore::Vector2d _localPosition1Variance;
  StepCore::Vector2d _localPosition2Variance;
  
  double _tensionVariance; 
  
  friend class PulleyCord;
  
};



class PulleyCord : public Item, public Force
{
  STEPCORE_OBJECT(PulleyCord)
public:
   explicit PulleyCord(Vector2d position=Vector2d(5,0), double radius=1,Item* body1=0, Item* body2=0)
  ~PulleyCord()
  
  bool calcForce(bool calcVariance);
  
  Vector2d position() const { return _position; }
  void setPosition(const Vector2d position) { _position = position; }
  
  Vector2d radius() const { return _radius; }
  void setRadius(const double radius) { _radius = radius; }
  
  Vector2d cord1() const { return _cord1; }
  void setCord1(const Vector2d cord1) { _cord1 = cord1/cord1.norm(); }
  
  Vector2d cord2() const { return _cord2; }
  void setCord2(const Vector2d cord2) { _cord2 = cord2/cord2.norm(); }
  
  double tension() const { return _tension; }
  void setTension(const double tension) { _tension = tension; }
  
  Vector2d localPosition1() const { return _localPosition1; }
  void setLocalPosition1(const Vector2d localPosition1) { _localPosition1 = localPosition1; }
  
  Vector2d localPosition2() const { return _localPosition2; }
  void setLocalPosition2(const Vector2d localPosition2) { _localPosition2 = localPosition2; }
  
  void setBody1(Object* body1);
  void setBody2(Object* body2);
  
  
  Particle* particle1() const { return _p1; }
  Particle* particle2() const { return _p2; }
  
  RigidBody* rigidBody1() const { return _r1; }
  RigidBody* rigidBody2() const { return _r2; }
  
protected:
  
  Object* _object1;
  Object* _object2;
  
  Vector2d _position;
  double _radius;
  
  Vector2d _cord1;
  double _lenghtCord1;
  
  Vector2d _cord2;
  double _lenghtCord2;
  
  double tension;
  
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


