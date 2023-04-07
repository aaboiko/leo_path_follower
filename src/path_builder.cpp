#include "ros/ros.h"
#include "leo_path_follower/Costmap.h"
#include "leo_path_follower/CostmapRow.h"

#include <queue>
#include <tuple>

using namespace std;

const int INF = 1e9;
bool hasData = false;
int grid_size_y = 0, grid_size_x = 0;
int start_x, start_y, goal_x, goal_y;

vector<vector<float>> costmap;
vector<vector<float>> dist;
vector<vector<bool>> vis;

typedef tuple<int, int, int> pii; 
priority_queue<pii, vector<pii>, greater<pii>> pq; 
vector<pair<int, int>> path;

bool valid(int i, int j) {
    return i >= 0 && i < grid_size_x && j >= 0 && j < grid_size_y;
}

// Dijkstra's algorithm
void dijkstra(int sx, int sy, int gx, int gy) {
    pq.push(make_tuple(0, sx, sy));
    dist[sx][sy] = 0;
    
    while (!pq.empty()) {
        int d = get<0>(pq.top()), x = get<1>(pq.top()), y = get<2>(pq.top());
        pq.pop();
        
        if (x == gx && y == gy) break;
        if (vis[x][y]) continue;
        vis[x][y] = true;
        
        // Check neighbors
        int dx[] = {0, 0, -1, 1};
        int dy[] = {-1, 1, 0, 0};
        for (int k = 0; k < 4; k++) {
            int nx = x + dx[k], ny = y + dy[k];
            if (valid(nx, ny) && !vis[nx][ny]) {
                int nd = d + costmap[nx][ny];
                if (nd < dist[nx][ny]) {
                    dist[nx][ny] = nd;
                    pq.push(make_tuple(nd, nx, ny));
                }
            }
        }
    }
}

// Backtrack from goal to start to find optimal path
vector<pair<int, int>> getPath(int sx, int sy, int gx, int gy) {
    vector<pair<int, int>> path;
    
    if (dist[gx][gy] == INF) return path; // no path
    pair<int, int> cur = make_pair(gx, gy);
    path.push_back(cur);
    
    while (cur.first != sx || cur.second != sy) {
        int x = cur.first, y = cur.second;
        int dx[] = {0, 0, -1, 1};
        int dy[] = {-1, 1, 0, 0};
        
        for (int k = 0; k < 4; k++) {
            int nx = x + dx[k], ny = y + dy[k];
            if (valid(nx, ny)) {
                int nd = dist[nx][ny] + costmap[x][y];
                if (nd == dist[x][y]) {
                    cur = make_pair(nx, ny);
                    path.push_back(cur);
                    break;
                }
            }
        }
    }
    
    reverse(path.begin(), path.end());
    return path;
}

void msgCostmapCallback(const leo_path_follower::Costmap::ConstPtr& msg){
    for(leo_path_follower::CostmapRow row : msg->costmap){
        vector<float> temp;
        for(float e : row.arr){
            temp.push_back(e);
        }
        costmap.push_back(temp);
        grid_size_y = temp.size();
    }
    grid_size_x = costmap.size();

    dist = vector<vector<float>>(grid_size_x, vector<float>(grid_size_y, INF));
    vis = vector<vector<bool>>(grid_size_x, vector<bool>(grid_size_y, false));
    hasData = true;

    start_x--;
    start_y--;
    goal_x--;
    goal_y--;

    dijkstra(start_x, start_y, goal_x, goal_y);
    path = getPath(start_x, start_y, goal_x, goal_y);
}

int main(int argc, char** argv){
    start_x = atoi(argv[0]);
    start_y = atoi(argv[1]);
    goal_x = atoi(argv[2]);
    goal_y = atoi(argv[3]);
    
    ros::init(argc, argv, "path_builder");
    ros::NodeHandle nh;
    ros::Subscriber costmap_sub = nh.subscribe("costmap_publisher", 10, msgCostmapCallback);

    return 0;
}