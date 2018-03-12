// Copyright 2018 Ishan Khatri
#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include <vector>
using namespace std;

template <class Params>
class CostFunction {
  public:
    //CostFunction(vector<Params> pa);
    virtual float getCost() = 0;
    virtual vector<Params> getGradient() = 0;
    vector<Params> getParams() {return p;}
    void setParams(vector<Params> pa) {p = pa;}
  protected:
    vector<Params> p;
};

#endif
