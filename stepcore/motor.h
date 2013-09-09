/* This file is part of StepCore library.
   Copyright (C) 2007 Vladimir Kuznetsov <ks.vladimir@gmail.com>

   StepCore library is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   StepCore library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with StepCore; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/** \file motor.h
 *  \brief LinearMotor class
 */

#ifndef STEPCORE_MOTOR_H
#define STEPCORE_MOTOR_H

#include "world.h"
#include "object.h"
#include "constants.h"
#include "types.h"


namespace StepCore
{

class Particle;
class RigidBody;

/** \ingroup forces
 *  \brief Linear motor: applies a force at given position on the body
 */
class LinearMotor: public Item, public Force
{
    STEPCORE_OBJECT(LinearMotor)

public:
    /** Constructs LinearMotor */
    explicit LinearMotor(Object* body = 0, const Vector2d& localPosition = Vector2d::Zero(),
                        Vector2d forceValue = Vector2d::Zero(), double expiryTime = -1);

    void calcForce(bool calcVariances);

    /** Get pointer to the body */
    Object* body() const { return _body; }
    /** Set pointer to the connected body */
    void setBody(Object* body);

    /** Local position of the motor on the body
     *  or in the world (if the motor is not connected) */
    const Vector2d& localPosition() const { return _localPosition; }
    /** Set local position of the motor on the body
     *  or in the world (if the motor is not connected) */
    void setLocalPosition(const Vector2d& localPosition) { _localPosition = localPosition; }

    /** Position of the motor */
    Vector2d position() const;

    /** Get force value */
    const Vector2d& forceValue() const { return _forceValue; }
    /** Set force value */
    void setForceValue(const Vector2d& forceValue) { _forceValue = forceValue; }
    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    
    /* Get the time after which the motor stops applying force */
    double expiryTime() const { return _expiryTime; }
    /* Set the time after which the motor stops applying force. If _expiryTime is (-)ve the motor never dies */
    void setExpiryTime(const double time) { _expiryTime = time; }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //void worldItemRemoved(Item* item);
    //void setWorld(World* world);
        
protected:
    Object*  _body;
    Vector2d _localPosition;
    Vector2d _forceValue;
    double _expiryTime;

    Particle*  _p;
    RigidBody* _r;
};

/** \ingroup forces
 *  \brief Circular motor: applies a torque to the body
 */
class CircularMotor: public Item, public Force
{
    STEPCORE_OBJECT(CircularMotor)

public:
    /** Constructs CircularMotor */
    explicit CircularMotor(Object* body = 0, const Vector2d& localPosition = Vector2d::Zero(),
                                            double torqueValue = 0, double expiryTime = -1);

    void calcForce(bool calcVariances);

    /** Get pointer to the body */
    Object* body() const { return _body; }
    /** Set pointer to the connected body */
    void setBody(Object* body);

    /** Local position of the motor on the body
     *  or in the world (if the motor is not connected) */
    Vector2d localPosition() const;
    /** Set local position of the motor on the body
     *  or in the world (if the motor is not connected) */
    void setLocalPosition(const Vector2d& localPosition) {
        _localPosition = localPosition;
    }

    /** Position of the motor */
    Vector2d position() const;

    /** Get torque value */
    double torqueValue() const { return _torqueValue; }
    /** Set torque value */
    void setTorqueValue(const double torqueValue) { _torqueValue = torqueValue; }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////    
    
    /* Get the time after which the motor stops applying force */
    double expiryTime() const { return _expiryTime; }
    /* Set the time after which the motor stops applying force. If _expiryTime is (-)ve the motor never dies */
    void setExpiryTime(const double time) { _expiryTime = time; }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //void worldItemRemoved(Item* item);
    //void setWorld(World* world);
        
protected:
    Object*  _body;
    Vector2d _localPosition;
    double   _torqueValue;
    double _expiryTime;

    RigidBody* _r;
};


} // namespace StepCore

#endif

