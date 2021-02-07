#ifndef INCLUDE_AKFSFSIM_UTILS_H
#define INCLUDE_AKFSFSIM_UTILS_H

#include <vector>
#include "display.h"

double wrapAngle(double angle);
double calculateMean(const std::vector<double>& dataset);
double calculateRMSE(const std::vector<double>& dataset);

std::vector<Vector2> generateEllipse(double x, double y, double sigma_xx, double sigma_yy, double sigma_xy, int num_points = 50);
std::vector<Vector2> generateCircle(double x, double y, double radius, int num_points = 50);

#endif  // INCLUDE_AKFSFSIM_UTILS_H