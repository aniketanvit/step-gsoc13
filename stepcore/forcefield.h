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

/** \file forcefield.h
 *  \brief Attempt to add a magnetic-field like force (which acts only on moving charged particles)
 */

#ifndef FORCEFIELD_H
#define FORCEFIELD_H

#include "object.h"
#include "world.h"

namespace StepCore {

class ForceField;

class ForceFieldErrors : public ObjectErrors
{
   STEPCORE_OBJECT(ForceFieldErrors)
public:
  ForceFieldErrors(Item* owner=NULL) : ObjectErrors(owner) 
  {}
  ForceField* forceField() const;
  friend class ForceField;
};

/** \ingroup forces
 *  \brief  Magnetic-field like force which will act only on moving charged particles. 
 */

class ForceField : public Item, public Force 
{
  STEPCORE_OBJECT(ForceField)
public:
  explicit ForceField(double b = 1): _b(b)  {}
  ~ForceField() {}
  
  double b() const { return _b ; }
  void setB(const double b) { _b = b ; }
  
  void calcForce(bool calcVariances);
  
  ForceFieldErrors* forceFieldErrors()
  {
    return static_cast<ForceFieldErrors*>(objectErrors());
  }
protected:
  ForceFieldErrors* createObjectErrors()
  {
    return new ForceFieldErrors(this);
  }
  
  double _b;
  
};

}

#endif
