// File:         
// Date:         
// Description:  
// Author:       
// Modifications:


#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include "Fun.hpp"

void Fun::dealvi(int n, int *V) 
{
  int r1, r2, j;
  int a;
  for (j = 1; j < n+1 ; j++) {
    r1 = rand() % n + 1;
    r2 = rand() % n + 1;
    
    if (r1 != r2) {
      a     = V[r1];  
      V[r1] = V[r2];
      V[r2] = a;
    }
  }
}

void Fun::dealvf(int n, float *V) 
{
  int r1, r2, j;
  float a;
  for (j = 1; j < n+1 ; j++) {
    r1 = rand() % n+1;
    r2 = rand() % n+1;
    
    if (r1 != r2) {
      a     = V[r1];  
      V[r1] = V[r2];
      V[r2] = a;
    }
  }
}
