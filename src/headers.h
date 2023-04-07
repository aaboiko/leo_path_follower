#pragma once

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <vector>
#include <map>

#include <Eigen/Core>

using namespace pcl;
using namespace std;
using namespace Eigen;

void passThroughFilter(PointCloud<PointXYZ>::Ptr& input, PointCloud<PointXYZ>::Ptr& output);
void evaluateCloud(PointCloud<PointXYZ>::Ptr& cloud);
void segmentate(PointCloud<PointXYZ>::Ptr& cloud);
void visualizeGrid();
void createCostmap(map<pair<int, int>, Normal>& normal_map);
void visualizeCostmap();