// Copyright 2018 Ishan Khatri
#include <iostream>
#include <string>
#include <stdlib.h>
#include "PlaneFitCost.h"

using namespace std;

template <class Params>
vector<Params> gradientDescent(CostFunction<Params> &cost, float stepSize, float precision){
  vector<Params> p = cost.getParams();
  float c = cost.getCost();
  int iterations = 0;
  while(1){
    iterations++;
    // Check if we're already at the minimum
    if(c <= precision)
      break;
    // If not, calculate the gradient
    vector<Params> gradient = cost.getGradient();
    // Step in the direction of the gradient vector
    for(int i = 0; i < p.size(); i++){
      p[i] = p[i] - (stepSize * gradient[i]);
      cout << p[i] << endl;
    }
    // Update the cost & return to the top of the loop
    cost.setParams(p);
    c = cost.getCost();
  }
  cout<<"Number of iterations: "<<iterations<<endl;
  return p;
}

vector<Vector3f> generateRandomPoints(int n){
  vector<Vector3f> pointCloud;
  for(int i = 0; i<n; i++){
    float r = rand()%10;
    float f = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float p = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    Vector3f t(r+f, r+p, r);
    pointCloud.push_back(t);
  }
  return pointCloud;
}

int main(){
  vector<Vector3f> pointCloud = generateRandomPoints(50);
  vector<Vector3f> params;
  Vector3f temp;
  temp << 1, 2, 3;
  temp = temp/temp.norm();
  params.push_back(temp);
  params.push_back(Vector3f::Zero());
  /*for(auto v : pointCloud)
    cout << v[0] << ", " << v[1] << ", " << v[2] << endl;*/
  PlaneFitCost::PlaneFitCost c(params, pointCloud);
  float s = 0.001;
  float p = 0.000001;
  vector<Vector3f> result = gradientDescent<Vector3f>(c, s, p);
  cout<<"n hat: "<<result[0]<<endl<<"P0: "<<result[1]<<endl;
  return 0;
}