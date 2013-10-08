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

/** \file pulleycord.h
 *  \brief Class definition for the pulleycord joint.
 */

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

/** \ingroup joints
 *  \brief A type of distance joint in which the sum of the distaces of the bodies from two fixed anchor points must remain constant.
 */
class PulleyCord : public Item, public Joint
{
  STEPCORE_OBJECT(PulleyCord)
  
public:
  /** Constructor for the pulleycord class */
   explicit PulleyCord(Vector2d position=Vector2d::Zero(), double radius=0.5, Item* body1 = NULL, Item* body2 = NULL);
   ~PulleyCord() {}
  /** Return the number of equations generated by a pulley joint. */
  int constraintsCount();
  /** Fill the constraint information at the correct offset in the info structure. */
  void getConstraintsInfo(ConstraintsInfo* info, int offset);
  /** Return the position of the center of the disc of the pulley. */
  Vector2d position() const { return _position; }
  /** Set the position of the center of the disc of the pulley.
   *  end1 and end2 are the points of contact of the cord with the cord.
   *  they are currently being assumed fixed ( I plan to change it later )
   */
  void setPosition(const Vector2d position) 
  {
    _position = position;
    end1 = Vector2d(_position[0] - _radius, _position[1]);
    end2 = Vector2d(_position[0] + _radius, _position[1]);
  }
  /** Get the position of the left end of the cord. */
  Vector2d position1() const;
  /** Get the position of the right end of the cord. */
  Vector2d position2() const;
  /** Get the velocity of the left end of the cord. */
  Vector2d velocity1() const;
  /** Get the velocity of the right end of the cord. */
  Vector2d velocity2() const;
  /** Get the radius of the disc of the pulley. */
  double radius() const { return _radius; }
  /** Set the radius of the disc of the pulley.
   * ON every update of the radius of the disc of the pulley, we must also 
   * update the position of end1 and end2.
   */
  void setRadius(const double radius)
  {
    _radius = radius;
    end1 = Vector2d(_position[0] - _radius, _position[1]);
    end2 = Vector2d(_position[0] + _radius, _position[1]);
  }
  /** Get the total length of the cord ( ehich is decided while fixing the bodies at the ends of the cord.) */
  double lengthOfCord() const { return _lengthOfCord; }
  /** Set the total length of the cord of the pulley. */
  void setLengthOfCord(const double length) { _lengthOfCord = length; }
  /** Return the value of the tension force developed on the cord due to forces acting on the bodies attached to th ends of the 
   * cord of the pulley. */
  double tension() const { return _tension; }
  /** Return the value of the tension force developed on the cord due to forces acting on the bodies attached to th ends of the 
   * cord of the pulley. */
  void setTension(const double tension) { _tension = tension; }
  /** Get the local-position of the cord on body1 as a 2-D vector. */
  Vector2d localPosition1() const { return _localPosition1; }
  /** Set the local-position of the cord on body1. */
  void setLocalPosition1(const Vector2d localPosition1) { _localPosition1 = localPosition1; }
  /** Get the local-position of cord on body2. */
  Vector2d localPosition2() const { return _localPosition2; }
  /** Set the local-position of the cord on body2. */
  void setLocalPosition2(const Vector2d localPosition2) { _localPosition2 = localPosition2; }
  /** Set the body attached to the left-end of the cord of the pulley. */
  void setBody1(Object* body1);
  /** Set the body attached to the right-end of the cord of the pulley. */
  void setBody2(Object* body2);
  /** Get the Body attached to the left-end of the cord ( as Object* ). */
  Object* body1() const { return _body1; }
  /** Get the Body attached to the right-end of the cord ( as Object* ). */
  Object* body2() const { return _body2; }
  
  Particle* particle1() const { return _p1; }
  Particle* particle2() const { return _p2; }
  
  RigidBody* rigidBody1() const { return _r1; }
  RigidBody* rigidBody2() const { return _r2; }
  /** Returns true if the cord of the pulley is in tension
   * due to the forces acting on the bodies attached at the two ends. */
  bool inTension();
  /** The point on the pulley where the left end of cord meets it. i.e. tangent from the cord to the pulley. */
  Vector2d end1;
  /** The point on the pulley where the left end of cord meets it. i.e. tangent from the cord to the pulley. */
  Vector2d end2;
  /** Lenght of the left part of the cord. */
  double _length1;
  /** Length of the right part of the cord. */
  double _length2;
  
protected:
  
  Vector2d _position;
  double _radius; 
  
  Object* _body1;
  Object* _body2;
  
  double _lengthOfCord;
  
  double _tension;
  //bool _inTension;
  
  Vector2d _localPosition1;
  Vector2d _localPosition2;
  
  Particle*  _p1;
  Particle*  _p2;
  RigidBody* _r1;
  RigidBody* _r2;

};

}  // namespace StepCore

#endif

