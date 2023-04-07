#include "headers.h"
#include "leo_path_follower/Costmap.h"
#include "leo_path_follower/CostmapRow.h"
#include "ros/ros.h"

using namespace pcl;
using namespace std;
using namespace Eigen;

const float z_min = -1.0;
const float z_max = 2.0;
const float cell_size = 1;
const float neigh_search_radius = 0.03;
float cloud_size_x, cloud_size_y, cloud_size_z;
int grid_size_x, grid_size_y;

const float k_n = 1, k_inc = 1, k_skip = 1;

string filename = "/home/anatoliy/cloud.ply";
PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr cloud_filtered(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr cloud_separated(new PointCloud<PointXYZ>);

map<pair<int, int>, PointCloud<PointXYZ>> grid;
map<pair<int, int>, PointCloud<PointXYZ>> grid_projected;
map<pair<int, int>, Normal>               normal_map;
map<pair<int, int>, float>                costmap;
map<pair<int, int>, int>                  point_number_map;

vector<vector<float>> costmap_grid;

int main(int argc, char** argv){
    ros::init(argc, argv, "costmap_publisher");
    ros::NodeHandle nh;
    ros::Publisher costmap_pub = nh.advertise<leo_path_follower::Costmap>("costmap_msg", 10);
    ros::Rate loop_rate(1);
    leo_path_follower::Costmap msg;

    evaluateCloud(cloud);
    PLYReader reader;
    reader.read(filename, *cloud);

    evaluateCloud(cloud);
    segmentate(cloud);
    createCostmap(normal_map);

    for(int i = 0; i < grid_size_x; i++){
        vector<float> row;
        for(int j = 0; j < grid_size_y; j++){
            row.push_back(costmap[make_pair(i, j)]);
        }
        costmap_grid.push_back(row);
    }

    cout << "Costmap grid" << endl;
    for(int i = 0; i < grid_size_x; i++){
        for(int j = 0; j < grid_size_y; j++){
            cout << costmap_grid[i][j] << " ";
        }
    }
    cout << endl;

    while(ros::ok()){
        for(int i = 0; i < grid_size_x; i++){
            leo_path_follower::CostmapRow row;
            for(int j = 0; j < grid_size_y; j++){
                row.arr[j] = costmap_grid[i][j];
            }
            msg.costmap[i] = row;
        }

        costmap_pub.publish(msg);
        loop_rate.sleep();
        cout << "success" << endl;
    }

    return 0;
}

void evaluateCloud(PointCloud<PointXYZ>::Ptr& cloud){
    float xmax = -100.0, xmin = 100.0, ymax = -100.0, ymin = 100.0, zmax = -100.0, zmin = 100.0;

    for(const auto& point : *cloud){
        xmax = std::max(xmax, point.x);
        xmin = std::min(xmin, point.x);
        ymax = std::max(ymax, point.y);
        ymin = std::min(ymin, point.y);
        zmax = std::max(zmax, point.z);
        zmin = std::min(zmin, point.z);
    }
    ROS_DEBUG_STREAM("Limits: x:[" << xmin << " " << xmax << 
            "] y:[" << ymin << " " << ymax << "] z:[" << zmin << 
            " " << zmax << "]" << '\n');

    cloud_size_x = xmax - xmin;
    cloud_size_y = ymax - ymin;
    cloud_size_z = zmax - zmin;

    grid_size_x = int(cloud_size_x / cell_size) + 1;
    grid_size_y = int(cloud_size_y / cell_size) + 1;

    ROS_DEBUG_STREAM("Cloud size: x = " << cloud_size_x << 
                            ", y = " << cloud_size_y << 
                            ", z = " << cloud_size_z << '\n');
    ROS_DEBUG_STREAM("Grid: " << grid_size_x << "x" << grid_size_y << '\n');
}

void passThroughFilter(PointCloud<PointXYZ>::Ptr& input, PointCloud<PointXYZ>::Ptr& output){
    PassThrough<PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud_filtered);
}

void segmentate(PointCloud<PointXYZ>::Ptr& cloud){
    for(const auto& point : *cloud){
        int x = int(point.x / cell_size);
        int y = int(point.y / cell_size);
        grid[make_pair(x, y)].push_back(point);
    }

    for(int i = 0; i < grid_size_x; ++i){
        for(int j = 0; j < grid_size_y; ++j){
            PointCloud<PointXYZ> pc = grid[make_pair(i,j)];
            PointCloud<PointXYZ>::Ptr ptr = pc.makeShared();
            
            if(pc.size() > 0){
                Matrix<float, 3, 3> A;
                Matrix<float, 3, 1> B;
                A << 0,0,0,0,0,0,0,0,0;
                B << 0,0,0;
                
                for(const auto& point : *ptr){
                    A(0, 0) += point.x * point.x;
                    A(0, 1) += point.x * point.y;
                    A(0, 2) += point.x;
                    A(1, 0) += point.x * point.y;
                    A(1, 1) += point.y * point.y;
                    A(1, 2) += point.y;
                    A(2, 0) += point.x;
                    A(2, 1) += point.y;
                    A(2, 2)++;
                    B(0) += point.x * point.z;
                    B(1) += point.y * point.z;
                    B(2) += point.z;
                }

                point_number_map[make_pair(i, j)] = A(2, 2);
                Matrix<float, 3, 1> X = A.inverse() * B;
                float norm = sqrt(X[0]*X[0]+X[1]*X[1]+1);
                float x = X[0] / norm, y = X[1] / norm, z = -1 / norm, d = X[2];
                
                ModelCoefficients::Ptr coefs(new ModelCoefficients());
                coefs->values.resize(4);
                coefs->values[0] = x;
                coefs->values[1] = y;
                coefs->values[2] = z;
                coefs->values[3] = d;

                ProjectInliers<PointXYZ> proj;
                proj.setModelType(SACMODEL_PLANE);
                proj.setInputCloud(ptr);
                proj.setModelCoefficients(coefs);
                proj.filter(*ptr);
                grid_projected[make_pair(i, j)] = *ptr;

                Normal normal(x, y, z);
                normal_map[make_pair(i, j)] = normal;
            }
        }
    }
}

void createCostmap(map<pair<int, int>, Normal>& normal_map){
    for(int i = 0; i < grid_size_x; i++){
        for(int j = 0; j < grid_size_y; j++){
            //Get number of points in cell
            int point_number = point_number_map[make_pair(i, j)];
            if(point_number < 3){
                costmap[make_pair(i, j)] = 1;
                continue;
            }

            //Get inclination angle
            Normal normal = normal_map[make_pair(i, j)];
            float x1 = normal.normal_x, y1 = normal.normal_y, z1 = normal.normal_z;
            float x2 = 0, y2 = 0, z2 = -1.0f;
            float dot = x1*x2 + y1*y2 + z1*z2;
            float sqlen1 = x1*x1 + y1*y1 + z1*z1;
            float sqlen2 = 1;
            float angle = acos(dot / sqrt(sqlen1 * sqlen2));

            //Get maximum skip between any two points in cell on Z axis
            float zmax = z_min, zmin = z_max, z_skip = 0.;
            
            PointCloud<PointXYZ> pc = grid_projected[make_pair(i,j)];

            for(const auto& point : pc){
                zmax = max(zmax, point.z);
                zmin = min(zmin, point.z);
            }
            z_skip = zmax - zmin;
        
            costmap[make_pair(i, j)] = 1 - exp(-(k_n * point_number * (k_inc * cos(angle) + k_skip * z_skip)));
        }
    }
}

void visualizeGrid(){
    visualization::CloudViewer viewer ("Point Cloud Viewer");
    PointCloud<PointXYZ>::Ptr cl(new PointCloud<PointXYZ>);

    for(int i = 0; i < grid_size_x; i++){
        for(int j = 0; j < grid_size_y; j++){
            *cl += grid_projected[make_pair(i, j)];
        }
    }
    viewer.showCloud(cl);

    while (!viewer.wasStopped ())
  {
  }
}

void visualizeCostmap(){
    
}