#ifndef PLANEFITCOST_H
#define PLANEFITCOST_H

#include <eigen3/Eigen/Dense>
#include "CostFunction.h"

using namespace std;
using Eigen::Vector3f;

// Params for plane fitting are nhat and p0
// The vector stores them in that order [nhat, p0]

class PlaneFitCost : public CostFunction<Vector3f> {
  public:
    PlaneFitCost(vector<Vector3f> pa);
    PlaneFitCost(vector<Vector3f> pa, vector<Vector3f> pointCloud);
    float getCost();
    vector<Vector3f> getGradient();
    vector<Vector3f> data;
};

PlaneFitCost::PlaneFitCost(vector<Vector3f> pa){
  p = pa;
}

PlaneFitCost::PlaneFitCost(vector<Vector3f> pa, vector<Vector3f> pointCloud){
  p = pa;
  data = pointCloud;
}

float PlaneFitCost::getCost(){
  float c = 0;
  for(int i = 0; i<data.size(); i++){
    int d =(data[i]-p[1]).dot(p[0]);
    c += d*d;
  }
  return c;
}

vector<Vector3f> PlaneFitCost::getGradient(){
  Eigen::Matrix<float, Eigen::Dynamic, 3> m(data.size(), 3);
  for(int i = 0; i<data.size(); i++){
    m(i, 0) = data[i].x() - p[1][0];
    m(i, 1) = data[i].y() - p[1][1];
    m(i, 2) = data[i].z() - p[1][2];
  }
  vector<Vector3f> grad;
  // t1 is the gradient with respect to n hat
  // 2 * d * m where d is m * nhat (M.transpose()*d)
  Vector3f t1 = 2*m.transpose()*(m*p[0]);
  cout << t1 << endl;
  // t2 is the gradient with respect to p0
  // 2 * d * nhat where d is m * nhat
  Vector3f t2 = 2*p[0]*p[0].transpose()*m;
  //Vector3f t2 = 2*(m*p[0])*p[0];
  cout << "Print some shit" << endl;
  grad.push_back(t1);
  grad.push_back(t2);
  return grad;
}

#endif