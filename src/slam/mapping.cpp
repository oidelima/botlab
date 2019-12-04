#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/lidar_t.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}

void Mapping::doBreshenhams(int x0, int y0, int x1, int y1, OccupancyGrid& map)
{
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
        // do this to prevent integer overflow
        if(((127 - abs(curCellOdds)) > kMissOdds_) || curCellOdds >= 0)
        {
            curCellOdds -= kMissOdds_;
        }
        map.setLogOdds(x, y, curCellOdds);
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
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    Point<int> originPoseCell = global_position_to_grid_cell(Point<double>(pose.x, pose.y), map);
    for(int i = 0; i < scan.num_ranges; i++)
    {
        float scanThetaGlobal = pose.theta - scan.thetas[i];
        float scanRange = scan.ranges[i];
        float scanDeltaXMeters = cos(scanThetaGlobal) * scanRange;
        float scanDeltaYMeters = sin(scanThetaGlobal) * scanRange;
        Point<int> destPoseCell = 
            global_position_to_grid_cell(Point<double>(pose.x + scanDeltaXMeters, pose.y + scanDeltaYMeters), map);
        if(scanRange < kMaxLaserDistance_ && scanRange > 0.2)
        {
            CellOdds curCellOdds = map.logOdds(destPoseCell.x, destPoseCell.y);
            // do this to prevent integer overflow
            if(((127 - std::abs(curCellOdds)) > kHitOdds_) || curCellOdds <= 0)
            {
                curCellOdds += kHitOdds_;
            }
            map.setLogOdds(destPoseCell.x, destPoseCell.y, curCellOdds);
        }
        doBreshenhams(originPoseCell.x, originPoseCell.y, destPoseCell.x, destPoseCell.y, map);
    }
}
