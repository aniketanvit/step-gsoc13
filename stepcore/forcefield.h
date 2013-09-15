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