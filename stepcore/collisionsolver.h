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

/** \file collisionsolver.h
 *  \brief CollisionSolver interface
 */

#ifndef STEPCORE_COLLISIONSOLVER_H
#define STEPCORE_COLLISIONSOLVER_H

#include "object.h"
#include "world.h"
#include "vector.h"

namespace StepCore
{

class Polygon;
class Body;

/** \ingroup contacts
 *  \brief Description of contact between two bodies
 */
struct Contact {
    enum {
        Unknown,        /**< Contact state was not (can not) be determined
                             (if state == Unknown all other fields are not used) */
        Separated,      /**< Bodies are far away */
        Separating,     /**< Bodies are contacted but moving apart */
        Contacted,      /**< Bodies are contacted but resting */
        Colliding,      /**< Bodies are collising */
        Intersected     /**< Bodies are interpenetrating */
    };
    Body*    body0;         /**< Body0 */
    Body*    body1;         /**< Body1 */
    int      state;         /**< Contact state (maximum of pointsState if pointsCount > 0) */
    double   distance;      /**< Distance between bodies */
    Vector2d normal;        /**< Contact normal (pointing from body0 to body1) */
    int      pointsCount;   /**< Count of contact points (either one or two) */
    int      pointsState[2];/**< Contact point states */
    Vector2d points[2];     /**< Contact point coordinated */
    double   vrel[2];       /**< Relative velocities at contact points */
};

/** \ingroup contacts
 *  \brief Collision solver interface
 *
 *  Provides generic interface for collision solvers.
 */
class CollisionSolver : public Object
{
    STEPCORE_OBJECT(CollisionSolver)

public:
    CollisionSolver(): _toleranceAbs(0.001), _localError(0) {}
    virtual ~CollisionSolver() {}

    /** Get absolute allowed tolerance */
    double toleranceAbs() const { return _toleranceAbs; }
    /** Set absolute allowed tolerance */
    virtual void setToleranceAbs(double toleranceAbs) { _toleranceAbs = toleranceAbs; }
    /** Get error estimation from last step */
    double localError() const { return _localError; }

    /** Check (and update) state of the contact
     *  \param contact contact to check (only body0 and body1 fields must be set)
     *  \return state of the contact (equals to contact->state)
     */
    virtual int checkContact(Contact* contact) = 0;

    /** Check contacts between several bodies
     *  \param bodies list of bodies to check
     *  \return maximum contact state (i.e. maximum value of Contact::state)
     */
    virtual int checkContacts(World::BodyList& bodies) = 0;

    // TODO: add errors
    virtual int solveCollisions(World::BodyList& bodies) = 0;

protected:
    double _toleranceAbs;
    //double _toleranceRel;
    double _localError;
};

/** \ingroup contacts
 *  \brief Discrete collision solver using Gilbert-Johnson-Keerthi distance algorithm
 *
 *  Objects are treated as colliding if distance between them are greater then zero
 *  but smaller then certain small value. If distance is less then zero objects are
 *  always treated as interpenetrating - this signals World::doEvolve to invalidate
 *  current time step and try with smaller stepSize until objects are colliding but
 *  not interpenetrating.
 */
class GJKCollisionSolver : public CollisionSolver
{
    STEPCORE_OBJECT(GJKCollisionSolver)

public:
    /*
    enum {
        OK = 0,
        CollisionDetected = 4096,
        PenetrationDetected = 4097
    };*/

    int checkContact(Contact* contact);
    int checkContacts(World::BodyList& bodies);
    //int findClosestPoints(const Polygon* polygon1, const Polygon* polygon2);

    int solveCollisions(World::BodyList& bodies);
    int solveConstraints(World::BodyList& bodies);
};

} // namespace StepCore

#endif
