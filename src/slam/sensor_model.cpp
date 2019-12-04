#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <lcmtypes/lidar_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}

int SensorModel::doBreshenhams(int x0, int y0, int x1, int y1, const OccupancyGrid& map)
{
    int isObstaclePresent = 0;
    int dx = abs(x0 - x1);
    int dy = abs(y0 - y1);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx-dy;
    int x = x0;
    int y = y0;
    while(x != x1 || y != y1)
    {
        CellOdds curCellOdds = map.logOdds(x, y);
        // TODO: make occupied threshold configurable via macro/private var
        if(curCellOdds > 0.0)
        {
            isObstaclePresent = 1;
        }
        int e2 = 2 * err;
        if(e2 >= -dy)
        {
            err -= dy;
            x += sx;
        }
        if(e2 <= dx)
        {
            err += dx;
            y += sy;
        }
    }
    return isObstaclePresent;
}



double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    double toReturn = 0.0;
    MovingLaserScan* adjustedScan = new MovingLaserScan(scan, sample.parent_pose, sample.pose);
    Point<int> originPoseCell = global_position_to_grid_cell(Point<double>(sample.pose.x, sample.pose.y), map);
    for(int i = 0; i < scan.num_ranges; i++)
    {
        float scanThetaGlobal = sample.pose.theta - scan.thetas[i];
        float scanRange = scan.ranges[i];
        float scanDeltaXMeters = cos(scanThetaGlobal) * scanRange;
        float scanDeltaYMeters = sin(scanThetaGlobal) * scanRange;

        Point<int> destPoseCell = 
            global_position_to_grid_cell(Point<double>(sample.pose.x + scanDeltaXMeters, sample.pose.y + scanDeltaYMeters), map);

        double castDist = 0.05; // start at 5cm cast
        float castDeltaXMeters = cos(scanThetaGlobal) * castDist;
        float castDeltaYMeters = sin(scanThetaGlobal) * castDist;
        Point<int> castCell = 
            global_position_to_grid_cell(Point<double>(sample.pose.x + castDeltaXMeters, sample.pose.y + castDeltaYMeters), map);
        double sensorMaxDist = 5.0;
        while(map.logOdds(castCell.x, castCell.y) <= 0 && castDist < sensorMaxDist)
        {
            castDist += 0.025;
            castDeltaXMeters = cos(scanThetaGlobal) * castDist;
            castDeltaYMeters = sin(scanThetaGlobal) * castDist;
            castCell = 
                global_position_to_grid_cell(Point<double>(sample.pose.x + castDeltaXMeters, sample.pose.y + castDeltaYMeters), map);
        }
        
        double distGridCoords = 
            sqrt(
                pow((double)castCell.x - (double)destPoseCell.x, 2.0) 
                + pow((double)castCell.y - (double)destPoseCell.y, 2.0));

        if(distGridCoords != 0.0)
        {
            toReturn += 1.0 / distGridCoords;
        }
        else
        {
            toReturn += 3.0;
        }
        
        /*
        int obstacleEnRoute = doBreshenhams(originPoseCell.x, originPoseCell.y, destPoseCell.x, destPoseCell.y, map);
        int obstacleAtEndPoint = 0;
        if(map.logOdds(destPoseCell.x, destPoseCell.y) > 0)
        {
            obstacleAtEndPoint = 1;
        }
        // laser overshot - return less likelyhoood
        if(obstacleEnRoute == 1)
        {
            toReturn += -12.0;
        }
        // laser undershot - return less likelyhood
        else if(obstacleAtEndPoint == 0)
        {
            toReturn += -8.0;
        }
        // if we got things right, return greater likelyhood
        else if(obstacleAtEndPoint == 1)
        {
            toReturn += 4.0;
        }
        */
    }

    return toReturn;
}
