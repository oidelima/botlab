#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.

    int cells_rem = map.heightInCells() * map.widthInCells();

    for (int y = 0; y < map.heightInCells(); y++)
    {
        for (int x = 0; x < map.widthInCells(); x++)
        {
            if (static_cast<int>(map(x,y)) >= 0 ){
                cells_[y*width_ + x] = 1;
                cells_rem -= 1;

            }else if (x == map.widthInCells() - 1 || y == map.heightInCells() - 1 || x == 0 || y == 0){
                cells_[y*width_ + x] = 2;
                cells_rem -= 1;
            }else {
                cells_[y*width_ + x] = 0;
            }
        }
    }

    int count = 1; //current min distance
    while(cells_rem > 0){

        for (int y = 0; y < map.heightInCells(); y++)
        {
            for (int x = 0; x < map.widthInCells(); x++)
            {

                if (cells_[y*width_ + x] == count){
                    //check north
                    if (y < map.heightInCells() - 1 && cells_[(y+1)*width_ + x] == 0){
                        cells_[(y+1)*width_ + x] = count + 1;
                        cells_rem -= 1;
                    }
                    //check south
                    if (y > 0 && cells_[(y-1)*width_ + x] == 0){
                        cells_[(y-1)*width_ + x] = count + 1;
                        cells_rem -= 1;
                    }
                    //check east
                    if (x < map.widthInCells() - 1 && cells_[(y)*width_ + x + 1] == 0){
                        cells_[(y-1)*width_ + x + 1] = count + 1;
                        cells_rem -= 1;
                    }
                    //check west
                    if (x > 0 && cells_[(y)*width_ + x - 1] == 0){
                        cells_[(y-1)*width_ + x - 1] = count + 1;
                        cells_rem -= 1;
                    }


                     //check north east
                    if (y < map.heightInCells() - 1 && x < map.widthInCells() - 1 && cells_[(y+1)*width_ + x + 1] == 0){
                        cells_[(y+1)*width_ + x + 1] = count + 1;
                        cells_rem -= 1;
                    }

                    //check north west
                    if (y < map.heightInCells() - 1 && x > 0 && cells_[(y+1)*width_ + x - 1] == 0){
                        cells_[(y+1)*width_ + x - 1] = count + 1;
                        cells_rem -= 1;
                    }
                     //check south east
                    if (y > 0 && x < map.widthInCells() - 1 && cells_[(y-1)*width_ + x + 1] == 0){
                        cells_[(y-1)*width_ + x + 1] = count + 1;
                        cells_rem -= 1;
                    }

                    //check south west
                    if (y > 0 && x > 0 && cells_[(y-1)*width_ + x - 1] == 0){
                        cells_[(y-1)*width_ + x - 1] = count + 1;
                        cells_rem -= 1;
                    }


                }

            }
        }
        count += 1;
    }


    for (int y = 0; y < map.heightInCells(); y++)
    {
        for (int x = 0; x < map.widthInCells(); x++)
        {

            cells_[y*width_ + x] = (cells_[y*width_ + x] - 1) * map.metersPerCell();
        }
    }


}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
