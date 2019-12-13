#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include<bits/stdc++.h>
#include <vector>
#include <common/point.hpp>
#include <common/grid_utils.hpp>
#include <math.h>
#include <limits.h>
#include <common/timestamp.h>

using namespace std;



// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;

// A structure to hold the neccesary parameters
struct cell
{
    // Row and Column index of its parent
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
};

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col, int rows, int cols)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < rows) &&
           (col >= 0) && (col < cols);


}

bool isSafe(int row, int col, const ObstacleDistanceGrid& distances, const SearchParams& params)
{

    // Search a circular region around (x, y). If any of the cells within the robot radius are occupied, then the
    // cell isn't safe.
    //const int kSafeCellRadius = std::lrint(std::ceil(params.minDistanceToObstacle * distances.cellsPerMeter()));

    /*for(int dy = -kSafeCellRadius; dy <= kSafeCellRadius + 1; ++dy)
    {
        for(int dx = -kSafeCellRadius; dx <= kSafeCellRadius + 1; ++dx)
        {
            // Ignore the corners of the square region, where outside the radius of the robot
            if(std::sqrt(dx*dx + dy*dy) * distances.metersPerCell() > params.minDistanceToObstacle)
            {
                continue;
            }

            // If the odds at the cells are greater than 0, then there's a collision, so the cell isn't safe
            if(distances(col + dx, row + dy) == 0)
            {
                return false;
            }
        }
    }*/

    if (distances(col, row) <= params.minDistanceToObstacle ) return false;

    // The area around the cell is free of obstacles, so all is well
    return true;
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(const ObstacleDistanceGrid& distances, int row, int col)
{
    // Returns true if the cell is not blocked else false
    if (distances(col, row) > 0)
        return (true);
    else
        return (false);


}

// A Utility Function to trace the path from the source
// to destination
//void tracePath(cell cellDetails[][COL], Pair dest)

stack<Pair> tracePath(vector<vector<cell>> cellDetails, Pair dest)
{
    int row = dest.second;
    int col = dest.first;

    stack<Pair> Path;


    while (!(cellDetails[row][col].parent_i == row
             && cellDetails[row][col].parent_j == col ))
    {
        Path.push (make_pair (row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push (make_pair (row, col));


    return Path;
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest)
{
    if (row == dest.second && col == dest.first)
        return (true);
    else
        return (false);
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest, const ObstacleDistanceGrid& distances)
{
    // Return using the distance formula and the distance to obstacle
    double obstacle_H = distances(col, row);
    double distance_H = (double)sqrt ((row-dest.second)*(row-dest.second) + (col-dest.first)*(col-dest.first));
    return distance_H + obstacle_H; // obstacle_H
}



// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
stack<Pair> aStarSearch(const ObstacleDistanceGrid& distances, Pair src, Pair dest, int rows, int cols, const SearchParams& params)
{
    // If the source is out of range
    if (isValid (src.second, src.first, rows, cols) == false)
    {
        printf ("Source is invalid\n");
        stack<Pair> Path;
        return Path;
    }

    // If the destination is out of range
    if (isValid (dest.second, dest.first, rows, cols) == false )
    {
        printf ("Destination is invalid\n");
        stack<Pair> Path;
        return Path;
    }

    // Either the source or the destination is blocked
    if (isUnBlocked(distances, src.second, src.first) == false ||
            isUnBlocked(distances, dest.second, dest.first) == false)
    {
        printf ("Source or the destination is blocked\n");
        stack<Pair> Path;
        return Path;
    }

    // If the destination cell is the same as source cell
    if (isDestination(src.second, src.first ,dest) == true)
    {
        printf ("We are already at the destination\n");
        stack<Pair> Path;
        return Path;
    }

    // Create a closed list and initialise it to false which means
    // that no cell has been included yet
    // This closed list is implemented as a boolean 2D array
    bool closedList[rows][cols];
    memset(closedList, false, sizeof (closedList));

    // Declare a 2D array of structure to hold the details
    //of that cell
    //cell cellDetails[rows][cols];
    vector<vector<cell>> cellDetails;

    cellDetails.resize(rows, std::vector<cell>(cols, {
            10000,
            10000,
            10000,
            -1,
            -1
        }));

    int i, j;

    /*for (i=0; i<rows; i++)
    {
        for (j=0; j<cols; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }*/

    // Initialising the parameters of the starting node
    i = src.second, j = src.first;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    /*
     Create an open list having information as-
     <f, <i, j>>
     where f = g + h,
     and i, j are the row and column index of that cell
     Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
     This open list is implenented as a set of pair of pair.*/
    set<pPair> openList;

    // Put the starting cell on the open list and set its
    // 'f' as 0
    openList.insert(make_pair (0.0, make_pair (i, j)));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty())
    {
        pPair p = *openList.begin();

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        i = p.second.first;
        j = p.second.second;
        closedList[i][j] = true;

       /*
        Generating all the 8 successor of this cell

            N.W   N   N.E
              \   |   /
               \  |  /
            W----Cell----E
                 / | \
               /   |  \
            S.W    S   S.E

        Cell-->Popped Cell (i, j)
        N -->  North       (i-1, j)
        S -->  South       (i+1, j)
        E -->  East        (i, j+1)
        W -->  West           (i, j-1)
        N.E--> North-East  (i-1, j+1)
        N.W--> North-West  (i-1, j-1)
        S.E--> South-East  (i+1, j+1)
        S.W--> South-West  (i+1, j-1)*/

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------

        // Only process this cell if this is a valid one
        if (isValid(i-1, j, rows, cols) == true && isSafe(i-1, j, distances, params))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j].parent_i = i;
                cellDetails[i-1][j].parent_j = j;
                printf ("The destination cell is found\n");
                stack<Pair> Path = tracePath (cellDetails, dest);
                foundDest = true;
                return Path;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i-1][j] == false &&
                     isUnBlocked(distances, i-1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i-1, j, dest, distances);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << "(cellDetails[i-1][j].f> fNew): " << (cellDetails[i-1][j].f) << " " << (fNew) << "\n";
                if (cellDetails[i-1][j].f == FLT_MAX ||
                        cellDetails[i-1][j].f > fNew)
                {
                    openList.insert( make_pair(fNew,
                                               make_pair(i-1, j)));

                    // Update the details of this cell
                    cellDetails[i-1][j].f = fNew;
                    cellDetails[i-1][j].g = gNew;
                    cellDetails[i-1][j].h = hNew;
                    cellDetails[i-1][j].parent_i = i;
                    cellDetails[i-1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------

        // Only process this cell if this is a valid one
        if (isValid(i+1, j, rows, cols) == true && isSafe(i+1, j, distances, params))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j].parent_i = i;
                cellDetails[i+1][j].parent_j = j;
                printf("The destination cell is found\n");
                stack<Pair> Path = tracePath(cellDetails, dest);
                foundDest = true;
                return Path;
            }
            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i+1][j] == false &&
                     isUnBlocked(distances, i+1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i+1, j, dest, distances);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << "(cellDetails[i+1][j].f> fNew): " << (cellDetails[i+1][j].f) << " " << (fNew) << "\n";
                if (cellDetails[i+1][j].f == FLT_MAX ||
                        cellDetails[i+1][j].f > fNew)
                {
                    openList.insert( make_pair (fNew, make_pair (i+1, j)));
                    // Update the details of this cell
                    cellDetails[i+1][j].f = fNew;
                    cellDetails[i+1][j].g = gNew;
                    cellDetails[i+1][j].h = hNew;
                    cellDetails[i+1][j].parent_i = i;
                    cellDetails[i+1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------

        // Only process this cell if this is a valid one
        if (isValid (i, j+1, rows, cols) == true && isSafe(i, j+1, distances, params))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j+1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i][j+1].parent_i = i;
                cellDetails[i][j+1].parent_j = j;
                printf("The destination cell is found\n");
                stack<Pair> Path = tracePath(cellDetails, dest);
                foundDest = true;
                return Path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            else if (closedList[i][j+1] == false &&
                     isUnBlocked (distances, i, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue (i, j+1, dest, distances);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << "(cellDetails[i][j+1].f> fNew): " << (cellDetails[i][j+1].f) << " " << (fNew) << "\n";
                if (cellDetails[i][j+1].f == FLT_MAX ||
                        cellDetails[i][j+1].f > fNew)
                {
                    openList.insert( make_pair(fNew,
                                        make_pair (i, j+1)));

                    // Update the details of this cell
                    cellDetails[i][j+1].f = fNew;
                    cellDetails[i][j+1].g = gNew;
                    cellDetails[i][j+1].h = hNew;
                    cellDetails[i][j+1].parent_i = i;
                    cellDetails[i][j+1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------

        // Only process this cell if this is a valid one
        if (isValid(i, j-1, rows, cols) == true && isSafe(i, j-1, distances, params))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i, j-1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i][j-1].parent_i = i;
                cellDetails[i][j-1].parent_j = j;
                printf("The destination cell is found\n");
                stack<Pair> Path = tracePath(cellDetails, dest);
                foundDest = true;
                return Path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            
            else if (closedList[i][j-1] == false &&
                     isUnBlocked(distances, i, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j-1, dest, distances);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << "(cellDetails[i][j-1].f> fNew): " << (cellDetails[i][j-1].f) << " " << (fNew) << "\n";
                if (cellDetails[i][j-1].f == FLT_MAX ||
                        cellDetails[i][j-1].f > fNew)
                {
                    openList.insert( make_pair (fNew,
                                          make_pair (i, j-1)));

                    // Update the details of this cell
                    cellDetails[i][j-1].f = fNew;
                    cellDetails[i][j-1].g = gNew;
                    cellDetails[i][j-1].h = hNew;
                    cellDetails[i][j-1].parent_i = i;
                    cellDetails[i][j-1].parent_j = j;
                }
            }
        }

        //----------- 5th Successor (North-East) ------------

        // Only process this cell if this is a valid one
        if (isValid(i-1, j+1, rows, cols) == true && isSafe(i-1, j+1, distances, params))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i-1, j+1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j+1].parent_i = i;
                cellDetails[i-1][j+1].parent_j = j;
                printf ("The destination cell is found\n");
                stack<Pair> Path = tracePath (cellDetails, dest);
                foundDest = true;
                return Path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            
            else if (closedList[i-1][j+1] == false &&
                     isUnBlocked(distances, i-1, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i-1, j+1, dest, distances);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << "(cellDetails[i-1][j+1].f> fNew): " << (cellDetails[i-1][j+1].f) << " " << (fNew) << "\n";
                if (cellDetails[i-1][j+1].f == FLT_MAX ||
                        cellDetails[i-1][j+1].f > fNew)
                {
                    openList.insert( make_pair (fNew,
                                    make_pair(i-1, j+1)));

                    // Update the details of this cell
                    cellDetails[i-1][j+1].f = fNew;
                    cellDetails[i-1][j+1].g = gNew;
                    cellDetails[i-1][j+1].h = hNew;
                    cellDetails[i-1][j+1].parent_i = i;
                    cellDetails[i-1][j+1].parent_j = j;
                }
            }
        }

        //----------- 6th Successor (North-West) ------------

        // Only process this cell if this is a valid one
        if (isValid (i-1, j-1, rows, cols) == true && isSafe(i-1, j-1, distances, params))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination (i-1, j-1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i-1][j-1].parent_i = i;
                cellDetails[i-1][j-1].parent_j = j;
                printf ("The destination cell is found\n");
                stack<Pair> Path = tracePath (cellDetails, dest);
                foundDest = true;
                return Path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            
            else if (closedList[i-1][j-1] == false &&
                     isUnBlocked(distances, i-1, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i-1, j-1, dest, distances);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << "(cellDetails[i-1][j-1].f> fNew): " << (cellDetails[i-1][j-1].f) << " " << (fNew) << "\n";
                if (cellDetails[i-1][j-1].f == FLT_MAX ||
                        cellDetails[i-1][j-1].f > fNew)
                {
                    openList.insert( make_pair (fNew, make_pair (i-1, j-1)));
                    // Update the details of this cell
                    cellDetails[i-1][j-1].f = fNew;
                    cellDetails[i-1][j-1].g = gNew;
                    cellDetails[i-1][j-1].h = hNew;
                    cellDetails[i-1][j-1].parent_i = i;
                    cellDetails[i-1][j-1].parent_j = j;
                }
            }
        }

        //----------- 7th Successor (South-East) ------------

        // Only process this cell if this is a valid one
        if (isValid(i+1, j+1, rows, cols) == true && isSafe(i+1, j+1, distances, params))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j+1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j+1].parent_i = i;
                cellDetails[i+1][j+1].parent_j = j;
                printf ("The destination cell is found\n");
                stack<Pair> Path = tracePath (cellDetails, dest);
                foundDest = true;
                return Path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            
            else if (closedList[i+1][j+1] == false &&
                     isUnBlocked(distances, i+1, j+1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i+1, j+1, dest, distances);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << "(cellDetails[i+1][j+1].f> fNew): " << (cellDetails[i+1][j+1].f) << " " << (fNew) << "\n";
                if (cellDetails[i+1][j+1].f == FLT_MAX ||
                        cellDetails[i+1][j+1].f > fNew)
                {
                    openList.insert(make_pair(fNew,
                                        make_pair (i+1, j+1)));

                    // Update the details of this cell
                    cellDetails[i+1][j+1].f = fNew;
                    cellDetails[i+1][j+1].g = gNew;
                    cellDetails[i+1][j+1].h = hNew;
                    cellDetails[i+1][j+1].parent_i = i;
                    cellDetails[i+1][j+1].parent_j = j;
                }
            }
        }

        //----------- 8th Successor (South-West) ------------

        // Only process this cell if this is a valid one
        if (isValid (i+1, j-1, rows, cols) == true && isSafe(i+1, j-1, distances, params))
        {
            // If the destination cell is the same as the
            // current successor
            if (isDestination(i+1, j-1, dest) == true)
            {
                // Set the Parent of the destination cell
                cellDetails[i+1][j-1].parent_i = i;
                cellDetails[i+1][j-1].parent_j = j;
                printf("The destination cell is found\n");
                stack<Pair> Path = tracePath(cellDetails, dest);
                foundDest = true;
                return Path;
            }

            // If the successor is already on the closed
            // list or if it is blocked, then ignore it.
            // Else do the following
            
            else if (closedList[i+1][j-1] == false &&
                     isUnBlocked(distances, i+1, j-1) == true)
            {
                gNew = cellDetails[i][j].g + 1.414;
                hNew = calculateHValue(i+1, j-1, dest, distances);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to
                // the open list. Make the current square
                // the parent of this square. Record the
                // f, g, and h costs of the square cell
                //                OR
                // If it is on the open list already, check
                // to see if this path to that square is better,
                // using 'f' cost as the measure.
                //cout << "(cellDetails[i+1][j-1].f> fNew): " << (cellDetails[i+1][j-1].f) <<  " " << (fNew) << "\n";
                if (cellDetails[i+1][j-1].f == FLT_MAX ||
                        cellDetails[i+1][j-1].f > fNew)
                {
                    openList.insert(make_pair(fNew,
                                        make_pair(i+1, j-1)));

                    // Update the details of this cell
                    cellDetails[i+1][j-1].f = fNew;
                    cellDetails[i+1][j-1].g = gNew;
                    cellDetails[i+1][j-1].h = hNew;
                    cellDetails[i+1][j-1].parent_i = i;
                    cellDetails[i+1][j-1].parent_j = j;
                }
            }
        }
    }

    // When the destination cell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion cell. This may happen when the
    // there is no way to destination cell (due to blockages)
    if (foundDest == false)
        printf("Failed to find the Destination Cell\n");

    stack<Pair> Path;
    return Path;
}



robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    // Creating a shortcut for int, int pair type
    typedef pair<int, int> Pair;

    // Creating a shortcut for pair<int, pair<int, int>> type
    typedef pair<double, pair<int, int>> pPair;



    // Start position
    auto srcCell = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    Pair src = make_pair(srcCell.x, srcCell.y);

    // Destination
    auto destCell = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);
    Pair dest = make_pair(destCell.x, destCell.y);

    //get number of ROWS from occupancy grid
    int rows = distances.heightInCells();

    int cols = distances.widthInCells();

    stack<Pair> Path = aStarSearch(distances, src, dest, rows, cols, params);

        //pose_xyt_t path[Path.size()];

    robot_path_t path;
    path.utime = start.utime;
    //path.path.push_back(start);
    float prev_rounded_x = start.x;
    float prev_rounded_y = start.y;
    float prev_x = start.x;
    float prev_y = start.y;
    float prev_theta = 0.0;

    while (!Path.empty())
    {

        pair<int,int> p = Path.top();
        auto stepCell = grid_position_to_global_position(Point<float>(p.second, p.first),  distances);
        float rounded_x = round( stepCell.x * 100000.0 ) / 100000.0;
        float rounded_y = round( stepCell.y * 100000.0 ) / 100000.0;
        float theta = atan2(rounded_y - prev_rounded_y, rounded_x - prev_rounded_x);

        if (theta != prev_theta){
            //push previous step
            pose_xyt_t prev_step = {NULL, prev_x, prev_y, prev_theta};
            cout << "(" << prev_step.x << "," << prev_step.y << "," << prev_theta << ") -> ";
            path.path.push_back(prev_step);

            //pushing current step
            //pose_xyt_t step = {NULL, stepCell.x, stepCell.y, theta};
            //cout << "(" << step.x << "," << step.y << "," << theta << ") -> ";
            //path.path.push_back(step);
        }
        prev_x = stepCell.x;
        prev_y = stepCell.y;
        prev_rounded_x = rounded_x;
        prev_rounded_y = rounded_y;
        prev_theta = theta;
        Path.pop();

    }
    
    //push last step
    cout << "(" << goal.x << "," << goal.y << "," << prev_theta << ") -> ";
    path.path.push_back(goal);

    path.path_length = path.path.size();

    return path;
}

