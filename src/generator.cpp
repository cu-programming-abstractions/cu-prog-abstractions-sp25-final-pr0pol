
#include "generator.h"
#include <random>
#include <algorithm>
#include <iostream>

using namespace std;

// Directions for carving: up, right, down, left (2 steps to skip over wall cells)
const int CARVE_DIRECTIONS[4][2] = {{-2, 0}, {0, 2}, {2, 0}, {0, -2}};

static mt19937 rng(random_device{}()); // Random number generator

// Check if a coordinate is inside the maze bounds
bool isInBounds(int row, int col, int rows, int cols) {
    return row >= 0 && row < rows && col >= 0 && col < cols;
}

// Check if a cell has already been carved (i.e., is a path, not wall)
bool isCarved(const vector<string>& maze, int row, int col) {
    return maze[row][col] == ' ';
}

// Carve a path between two cells, including the wall in between
void carvePassage(vector<string>& maze, int startRow, int startCol, int endRow, int endCol) {
    maze[endRow][endCol] = ' '; // Carve destination

    int wallRow = (startRow + endRow) / 2;
    int wallCol = (startCol + endCol) / 2;
    maze[wallRow][wallCol] = ' '; // Carve wall between them
}

// Find all unvisited (not yet carved) neighbors two steps away
vector<pair<int, int>> getUnvisitedNeighbors(const vector<string>& maze, int row, int col) {
    vector<pair<int, int>> neighbors;
    int rows = maze.size();
    int cols = maze[0].size();

    for (int i = 0; i < 4; i++) {
        int newRow = row + CARVE_DIRECTIONS[i][0];
        int newCol = col + CARVE_DIRECTIONS[i][1];

        if (isInBounds(newRow, newCol, rows, cols) && !isCarved(maze, newRow, newCol)) {
            neighbors.push_back({newRow, newCol});
        }
    }

    return neighbors;
}

// Randomly shuffle a vector (used for direction randomness)
template<typename T>
void shuffleVector(vector<T>& vec) {
    shuffle(vec.begin(), vec.end(), rng);
}

// Recursive backtracking algorithm to generate the maze
void recursiveBacktrack(vector<string>& maze, int row, int col) {
    maze[row][col] = ' '; // Mark current cell as carved

    vector<pair<int, int>> neighbors = getUnvisitedNeighbors(maze, row, col);
    shuffleVector(neighbors); // Randomize order to vary maze layout

    for (auto& neighbor : neighbors) {
        int newRow = neighbor.first;
        int newCol = neighbor.second;

        if (!isCarved(maze, newRow, newCol)) {
            carvePassage(maze, row, col, newRow, newCol); // Carve wall + destination
            recursiveBacktrack(maze, newRow, newCol);     // Recurse from new cell
        }
    }
}

// Optional: Add some random open spaces to make rooms
void addRandomRooms(vector<string>& maze, int roomRate) {
    int rows = maze.size();
    int cols = maze[0].size();

    int totalWalls = (rows * cols) / 4;
    int roomsToAdd = (totalWalls * roomRate) / 100;

    for (int i = 0; i < roomsToAdd; i++) {
        int row = 2 + (rng() % (rows - 4)); // Avoid edges
        int col = 2 + (rng() % (cols - 4));

        if (maze[row][col] == '#') {
            maze[row][col] = ' '; // Create open space
        }
    }
}

// Place start 'S' and exit 'E' points in two far-apart open cells
void placeStartAndExit(vector<string>& maze) {
    vector<pair<int, int>> openCells;
    int rows = maze.size();
    int cols = maze[0].size();

    // Collect all open (carved) cells
    for (int r = 1; r < rows - 1; r++) {
        for (int c = 1; c < cols - 1; c++) {
            if (maze[r][c] == ' ') {
                openCells.push_back({r, c});
            }
        }
    }

    if (openCells.size() < 2) {
        cout << "Warning: Not enough open cells for start/exit placement!" << endl;
        return;
    }

    // Set first and last open cells as start and exit
    maze[openCells[0].first][openCells[0].second] = 'S';
    maze[openCells.back().first][openCells.back().second] = 'E';
}

// Generate a new dungeon maze with given size and room openness
std::vector<std::string> generateDungeon(int rows, int cols, int roomRate) {
    // Ensure odd dimensions (required for proper maze structure)
    if (rows % 2 == 0) rows++;
    if (cols % 2 == 0) cols++;

    // Enforce minimum size
    if (rows < 5 || cols < 5) {
        rows = max(rows, 5);
        cols = max(cols, 5);
    }

    // Initialize maze filled with walls
    vector<string> maze(rows, string(cols, '#'));

    recursiveBacktrack(maze, 1, 1);     // Carve maze starting at (1,1)
    addRandomRooms(maze, roomRate);    // Optionally add rooms
    placeStartAndExit(maze);           // Place 'S' and 'E'

    return maze;
}
