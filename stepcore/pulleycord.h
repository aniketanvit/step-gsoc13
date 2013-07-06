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
  
class PulleyCord : public Item, public Joint
{
  STEPCORE_OBJECT(PulleyCord)
  
public:
   explicit PulleyCord(Vector2d position=Vector2d::Zero(), double radius=1, double angle = 0);
  
  int constraintsCount();
  void getConstraintsInfo(ConstraintsInfo* info, int offset);

  Vector2d position() const { return _position; }
  void setPosition(const Vector2d position) { _position = position; }
  
  Vector2d position1() const;
  Vector2d position2() const;
  
  Vector2d velocity1() const;
  Vector2d velocity2() const;
  
  double radius() const { return _radius; }
  void setRadius(const double radius) { _radius = radius; }
  
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
    
  double _lengthOfCord;
  
  double _tension;
  bool _inTension;
  
  Vector2d _localPosition1;
  Vector2d _localPosition2;
  
  Object* _body1;
  Object* _body2;
  
  Particle*  _p1;
  Particle*  _p2;
  RigidBody* _r1;
  RigidBody* _r2;
  
  Vector2d end1;
  Vector2d end2;
};

}
#endif


