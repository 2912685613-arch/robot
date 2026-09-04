#ifndef VISUALSERVOING_H
#define VISUALSERVOING_H

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>


// recieve (x,y,z,visualYaw) from vision module

// tracking with gait mode
extern void gaitTrackTarget(float x, float y, float z, float visualYaw);


// tracking with vectored underwater control
extern void trackTargetwithVectControl(float x, float y, float z, float visualYaw);





#endif //VISUALSERVOING_H
