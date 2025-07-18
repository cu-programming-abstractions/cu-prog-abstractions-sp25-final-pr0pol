
#include "solver.h"
#include "cell.h"
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
using namespace std;

#include <iostream>
#include <bitset>
struct KeyState {
    int row, col, keyMask;

    KeyState() : row(0), col(0), keyMask(0) {} // default constructor
    KeyState(int r, int c, int mask) : row(r), col(c), keyMask(mask) {}

    bool operator==(const KeyState& other) const {
        return row == other.row && col == other.col && keyMask == other.keyMask;
    }
};

struct KeyStateHash {
    size_t operator()(const KeyState& state) const {
        return ((state.row * 31 + state.col) * 31) + state.keyMask;
    }
};




/**
 * Helper function: Find a specific character in the dungeon
 * Returns Cell(-1, -1) if not found
 */
Cell findPosition(const vector<string>& dungeon, char target) {
    for (size_t row = 0; row < dungeon.size(); row++) {
        for (size_t col = 0; col < dungeon[row].size(); col++) {
            if (dungeon[row][col] == target) {
                return Cell(static_cast<int>(row), static_cast<int>(col));
            }
        }
    }
    return Cell(-1, -1);
}

/**
 * Helper function: Check if a position is passable for basic BFS
 * (not a wall or door, and within bounds)
 */
bool isPassable(const vector<string>& dungeon, int row, int col) {
    if (row < 0 || static_cast<size_t>(row) >= dungeon.size() ||
        col < 0 || static_cast<size_t>(col) >= dungeon[0].size()) {
        return false;
    }
    char cell = dungeon[row][col];
    return cell == ' ' || cell == 'S' || cell == 'E' || islower(cell);
}

/**
 * Helper function: Check if we can pass through a door
 * Door 'A' requires key 'a', door 'B' requires key 'b', etc.
 */
bool canPassDoor(char door, int keyMask) {
    if (door < 'A' || door > 'F') return true;
    int keyBit = door - 'A';
    return (keyMask & (1 << keyBit)) != 0;
}


/**
 * Helper function: Collect a key by setting the appropriate bit
 */
int collectKey(char key, int keyMask) {
    if (key < 'a' || key > 'f') return keyMask;

    int keyBit = key - 'a';
    return keyMask | (1 << keyBit);
}

/**
 * Helper function: Reconstruct path from parent pointers
 */
vector<Cell> reconstructPath(const unordered_map<Cell, Cell, CellHash>& parents,
                             const Cell& start, const Cell& goal) {
    vector<Cell> path;
    Cell current = goal;

    while (!(current.r == start.r && current.c == start.c)) {
        path.push_back(current);
        auto it = parents.find(current);
        if (it == parents.end()) {
            return vector<Cell>();
        }
        current = it->second;
    }
    path.push_back(start);

    reverse(path.begin(), path.end());
    return path;
}

/**
 * Helper function: Check if a position is within bounds and not a wall
 * (used by key-door BFS which handles doors separately)
 */
bool isValidPosition(const vector<string>& dungeon, int row, int col) {
    if (row < 0 || static_cast<size_t>(row) >= dungeon.size() ||
        col < 0 || static_cast<size_t>(col) >= dungeon[0].size()) {
        return false;
    }
    return dungeon[row][col] != '#';
}

/**
 * Helper function: Get all valid neighboring cells for basic BFS
 */
vector<Cell> getNeighbors(const vector<string>& dungeon, const Cell& current) {
    vector<Cell> neighbors;

    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        int newRow = current.r + DIRECTIONS[i][0];
        int newCol = current.c + DIRECTIONS[i][1];

        if (isPassable(dungeon, newRow, newCol)) {
            neighbors.push_back(Cell(newRow, newCol));
        }
    }

    return neighbors;
}

std::vector<Cell> bfsPath(const std::vector<std::string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');

    if (start.r == -1 || exit.r == -1) {
        return {};
    }

    std::queue<Cell> bfsQueue;
    std::unordered_set<Cell, CellHash> visited;
    std::unordered_map<Cell, Cell, CellHash> parents;

    bfsQueue.push(start);
    visited.insert(start);

    while (!bfsQueue.empty()) {
        Cell current = bfsQueue.front();
        bfsQueue.pop();

        if (current.r == exit.r && current.c == exit.c) {
            return reconstructPath(parents, start, exit);
        }

        std::vector<Cell> neighbors = getNeighbors(dungeon, current);
        for (const Cell& neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                bfsQueue.push(neighbor);
                visited.insert(neighbor);
                parents[neighbor] = current;
            }
        }
    }

    return {};
}


/**
 * State structure for key-door BFS that includes position and collected keys.
 *
 * CONCEPT: In basic BFS, state = (row, col). In key-door BFS, state = (row, col, keys).
 * The same position with different keys represents different states with different possibilities.
 *
 * See BITMASK_BFS_GUIDE.md for detailed explanation of state augmentation concepts.
 */


/**
 * BITMASK OVERVIEW:
 * A bitmask efficiently stores which keys we have using a single integer.
 * Each bit represents one key: bit 0 = key 'a', bit 1 = key 'b', etc.
 *
 * Examples: keyMask=0 (no keys), keyMask=1 (key 'a'), keyMask=3 (keys 'a' and 'b')
 *
 * For detailed explanation and practice exercises, see BITMASK_BFS_GUIDE.md
 */

/**
 * Helper function to convert a keyMask to a readable string for debugging.
 */
string keyMaskToString(int keyMask) {
    string result = "Keys: ";
    bool hasAny = false;
    for (int i = 0; i < 6; i++) {
        if ((keyMask >> i) & 1) {
            result += char('a' + i);
            result += " ";
            hasAny = true;
        }
    }
    if (!hasAny) result += "none";
    return result;
}

/**
 * Helper function to reconstruct path from KeyState parents.
 */
vector<Cell> reconstructKeyPath(const unordered_map<KeyState, KeyState, KeyStateHash>& parents,
                                const KeyState& start, const KeyState& goal) {
    vector<Cell> path;
    KeyState current = goal;


    while (!(current.row == start.row && current.col == start.col && current.keyMask == start.keyMask)) {
        path.push_back(Cell(current.row, current.col));
        auto it = parents.find(current);
        if (it == parents.end()) {
            return vector<Cell>();
        }
        current = it->second;
    }
    path.push_back(Cell(start.row, start.col));

    reverse(path.begin(), path.end());
    return path;
}

std::vector<Cell> bfsPathKeys(const std::vector<std::string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');
    if (start.r == -1 || exit.r == -1) return {};

    std::queue<KeyState> q;
    std::unordered_set<KeyState, KeyStateHash> visited;
    std::unordered_map<KeyState, KeyState, KeyStateHash> parent;

    KeyState startState(start.r, start.c, 0);
    q.push(startState);
    visited.insert(startState);

    while (!q.empty()) {
        KeyState curr = q.front();
        q.pop();

        std::cout << "Visiting (" << curr.row << "," << curr.col << ") with keys: "
                  << std::bitset<6>(curr.keyMask) << std::endl;

        if (curr.row == exit.r && curr.col == exit.c) {
            std::cout << "Reached EXIT at (" << curr.row << "," << curr.col
                      << ") with keys: " << std::bitset<6>(curr.keyMask) << std::endl;
            std::vector<Cell> path;
            KeyState temp = curr;
            while (!(temp.row == start.r && temp.col == start.c)) {
                path.push_back(Cell(temp.row, temp.col));
                temp = parent[temp];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            int nr = curr.row + DIRECTIONS[i][0];
            int nc = curr.col + DIRECTIONS[i][1];

            if (nr < 0 || nr >= (int)dungeon.size() || nc < 0 || nc >= (int)dungeon[0].size()) continue;

            char ch = dungeon[nr][nc];
            if (ch == '#') continue;

            int newKeyMask = curr.keyMask;

            if (ch != 'E') {
                if (ch >= 'a' && ch <= 'f') {
                    newKeyMask |= (1 << (ch - 'a'));
                }

                if (ch >= 'A' && ch <= 'F') {
                    if ((newKeyMask & (1 << (ch - 'A'))) == 0) continue;
                }
            }

            KeyState next(nr, nc, newKeyMask);
            if (visited.count(next) == 0) {
                visited.insert(next);
                parent[next] = curr;

                std::cout << "  Enqueue (" << nr << "," << nc << ") with keys: "
                          << std::bitset<6>(newKeyMask) << std::endl;

                q.push(next);
            }
        }

    }
    return {};
}

#ifdef IMPLEMENT_OPTIONAL_FUNCTIONS
/**
 * OPTIONAL INTERMEDIATE CHALLENGE:
 * Practice bitmask operations by counting reachable keys (ignoring doors).
 * This is simpler than full key-door BFS and good preparation for bitmask concepts.
 *
 * For bitmask tutorials and practice exercises, see BITMASK_BFS_GUIDE.md
 */
int countReachableKeys(const std::vector<std::string>& dungeon) {
    // OPTIONAL CHALLENGE: Practice bitmask operations in a simpler context
    // This function uses basic BFS but practices key collection with bitmasks
    // Good preparation for understanding the full key-door BFS complexity

    Cell start = findPosition(dungeon, 'S');
    if (start.r == -1) return 0;

    // TODO #1: Set up basic BFS data structures
    // HINT: You need queue<Cell>, unordered_set<Cell, CellHash>, and int keyMask = 0

    // YOUR CODE HERE: Initialize BFS structures

    // TODO #2: Implement main BFS loop to explore reachable areas
    // HINT: This is similar to basic BFS, but collect keys when you find them
    // ALGORITHM:
    // 1. While queue not empty: dequeue current
    // 2. If current cell is a key ('a' to 'f'), use collectKey() to update keyMask
    // 3. Explore neighbors using isValidPosition() (ignore doors for this version)
    // 4. Add unvisited neighbors to queue and visited set

    // YOUR CODE HERE: BFS loop with key collection

    // TODO #3: Count how many different keys were collected
    // HINT: Count the number of set bits in keyMask
    // Each bit represents one key: bit 0 = 'a', bit 1 = 'b', etc.

    // YOUR CODE HERE: Count bits in keyMask

    // DEBUGGING TIPS:
    // 1. Print keyMask value to see which bits are set
    // 2. Use keyMaskToString() helper to see collected keys
    // 3. Test on simple dungeons with 1-2 keys first
    // 4. Remember: this version ignores doors completely

    return 0;  // TODO: Return actual count
}
#else
/**
 * Stub implementation when optional functions are disabled.
 * This prevents compilation errors when the function is not implemented.
 */
#ifdef IMPLEMENT_OPTIONAL_FUNCTIONS
int countReachableKeys(const std::vector<std::string>& dungeon) {
    Cell start = findPosition(dungeon, 'S');  // Now using dungeon parameter
    if (start.r == -1) return 0;

    queue<Cell> bfsQueue;
    unordered_set<Cell, CellHash> visited;
    int keyMask = 0;

    bfsQueue.push(start)
        visited.insert(start);

    while (!bfsQueue.empty()) {
        Cell current = bfsQueue.front();
        bfsQueue.pop();

        // Collect key if present
        char cellChar = dungeon[current.r][current.c];  // Using dungeon
        if (cellChar >= 'a' && cellChar <= 'f') {
            keyMask = collectKey(cellChar, keyMask);
        }

        // Explore neighbors
        vector<Cell> neighbors = getNeighbors(dungeon, current);  // Using dungeon
        for (const Cell& neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                bfsQueue.push(neighbor);
                visited.insert(neighbor);
            }
        }
    }

    // Count how many bits are set in keyMask
    int count = 0;
    for (int i = 0; i < 6; i++) {
        if ((keyMask >> i) & 1) {
            count++;
        }
    }

    return count;
}
#endif
#endif
