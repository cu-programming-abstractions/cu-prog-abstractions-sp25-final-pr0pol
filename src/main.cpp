
#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include "generator.h"
#include "solver.h"
#include "cell.h"

using namespace std;

std::vector<Cell> bfsPathKeys(const std::vector<std::string>& dungeon);

/**
 * Prints a dungeon to the console with proper formatting.
 * Shows walls (#), open spaces ( ), start (S), exit (E), keys (a-f), and doors (A-F).
 */
void printDungeon(const vector<string>& dungeon, const string& title = "") {
    if (!title.empty()) {
        cout << title << ":" << endl;
    }

    for (const string& row : dungeon) {
        cout << row << endl;
    }
    cout << endl;
}

/**
 * Prints a dungeon with the solution path marked using '*' characters.
 * The start 'S' and exit 'E' positions are preserved.
 */
void printDungeonWithPath(vector<string> dungeon, const vector<Cell>& path, const string& title = "") {
    if (!title.empty()) {
        cout << title << ":" << endl;
    }

    for (const Cell& cell : path) {
        if (cell.r >= 0 && cell.r < (int)dungeon.size() &&
            cell.c >= 0 && cell.c < (int)dungeon[0].size()) {
            char current = dungeon[cell.r][cell.c];
            if (current != 'S' && current != 'E') {
                dungeon[cell.r][cell.c] = '*';
            }
        }
    }

    for (const string& row : dungeon) {
        cout << row << endl;
    }
    cout << endl;
}

/**
 * Validates that a path is correct by checking:
 * 1. Path starts at 'S' and ends at 'E'
 * 2. Each step moves to an adjacent cell
 * 3. No step goes through walls
 * 4. Path is non-empty if start and exit exist
 */
bool validatePath(const vector<string>& dungeon, const vector<Cell>& path) {
    if (path.empty()) {
        return false;
    }

    Cell start = findPosition(dungeon, 'S');
    Cell exit = findPosition(dungeon, 'E');

    if (start.r == -1 || exit.r == -1) {
        return false;
    }

    if (path[0] != start || path[path.size() - 1] != exit) {
        return false;
    }

    for (size_t i = 0; i < path.size(); i++) {
        const Cell& cell = path[i];

        if (cell.r < 0 || cell.r >= (int)dungeon.size() ||
            cell.c < 0 || cell.c >= (int)dungeon[0].size()) {
            return false;
        }

        char cellChar = dungeon[cell.r][cell.c];
        if (cellChar == '#') {
            return false;
        }

        if (i > 0) {
            const Cell& prev = path[i - 1];
            int dr = abs(cell.r - prev.r);
            int dc = abs(cell.c - prev.c);

            if ((dr == 1 && dc == 0) || (dr == 0 && dc == 1)) {

            } else {
                return false;
            }
        }
    }

    return true;
}

/**
 * Creates a simple test dungeon for basic pathfinding tests.
 */
vector<string> createTestDungeon1() {
    return {
        "#######",
        "#S   E#",
        "#######"
    };
}

/**
 * Creates a more complex test dungeon with corridors and turns.
 */
vector<string> createTestDungeon2() {
    return {
        "#########",
        "#S#     #",
        "# # ### #",
        "#   #  E#",
        "#########"
    };
}

/**
 * Creates a test dungeon with keys and doors for advanced pathfinding.
 */
vector<string> createTestDungeonKeys() {
    return {
        "###########",
        "#S   a    #",
        "#A#########",
        "#       b #",
        "# #B#######",
        "# #     E #",
        "###########"
    };
}

/**
 * Creates an unsolvable test dungeon (no path from S to E).
 */
vector<string> createUnsolvableDungeon() {
    return {
        "#######",
        "#S###E#",
        "#######"
    };
}

/**
 * Test basic BFS pathfinding on a simple dungeon.
 */
bool testBasicPathfinding() {
    cout << "=== Basic Pathfinding Test ===" << endl;

    vector<string> dungeon = createTestDungeon1();
    printDungeon(dungeon, "Test Dungeon");

    vector<Cell> path = bfsPath(dungeon);

    cout << "Path length: " << path.size() << endl;

    bool success = false;
    if (path.empty()) {
        cout << "[ERROR] No path found!" << endl;
    } else if (validatePath(dungeon, path)) {
        cout << "[OK] Valid path found!" << endl;
        printDungeonWithPath(dungeon, path, "Solution");
        success = true;
    } else {
        cout << "[ERROR] Invalid path!" << endl;
    }

    cout << "--------------------------------------------------" << endl << endl;
    return success;
}

/**
 * Test BFS pathfinding on a more complex dungeon with turns.
 */
bool testComplexPathfinding() {
    cout << "=== Complex Pathfinding Test ===" << endl;

    vector<string> dungeon = createTestDungeon2();
    printDungeon(dungeon, "Complex Test Dungeon");

    vector<Cell> path = bfsPath(dungeon);

    cout << "Path length: " << path.size() << endl;

    bool success = false;
    if (path.empty()) {
        cout << "[ERROR] No path found!" << endl;
    } else if (validatePath(dungeon, path)) {
        cout << "[OK] Valid path found!" << endl;
        printDungeonWithPath(dungeon, path, "Solution");
        success = true;
    } else {
        cout << "[ERROR] Invalid path!" << endl;
    }

    cout << "--------------------------------------------------" << endl << endl;
    return success;
}

/**
 * Test key-door pathfinding on a dungeon with locked doors.
 */
bool testKeyDoorPathfinding() {
    cout << "=== Key-Door Pathfinding Test ===" << endl;
    cout << "[GUIDE] For detailed explanation of bitmask BFS concepts, see: BITMASK_BFS_GUIDE.md" << endl << endl;

    vector<string> dungeon = createTestDungeonKeys();
    printDungeon(dungeon, "Key-Door Test Dungeon");

#ifdef IMPLEMENT_OPTIONAL_FUNCTIONS
    cout << "Step 1: [OPTIONAL] Counting reachable keys (ignoring doors)..." << endl;
    int reachableKeys = countReachableKeys(dungeon);
    cout << "Reachable keys without considering doors: " << reachableKeys << endl << endl;
#else
    cout << "Step 1: [OPTIONAL] Key counting function not implemented" << endl;
    cout << "To enable: uncomment #define IMPLEMENT_OPTIONAL_FUNCTIONS in solver.h" << endl << endl;
#endif

    cout << "Step 2: [REQUIRED] Testing basic BFS (should fail due to locked door)..." << endl;
    vector<Cell> basicPath = bfsPath(dungeon);
    cout << "Basic BFS result: ";
    if (basicPath.empty()) {
        cout << "[ERROR] No path found (expected - door blocks the way)" << endl;
    } else {
        cout << "[OK] Found path of length " << basicPath.size() << " (unexpected!)" << endl;
    }
    cout << endl;

    cout << "Step 3: [REQUIRED] Testing key-door BFS (should succeed by collecting key first)..." << endl;
    cout << "===========================================" << endl;
    vector<Cell> keyPath = bfsPathKeys(dungeon);
    cout << "===========================================" << endl;

    cout << "Key-Door BFS result: ";
    bool success = false;
    if (keyPath.empty()) {
        cout << "[ERROR] No path found with key system!" << endl;
    } else if (validatePath(dungeon, keyPath)) {
        cout << "[OK] Valid key-door path found! Length: " << keyPath.size() << endl;
        printDungeonWithPath(dungeon, keyPath, "Key-Door Solution");

        // Educational analysis of the solution
        cout << "Solution Analysis:" << endl;
        cout << "- The algorithm first explores reachable areas" << endl;
        cout << "- Collects key 'a' when encountered" << endl;
        cout << "- Can then pass through door 'A' to reach exit" << endl;
        cout << "- This demonstrates state augmentation in BFS!" << endl;
        success = true;
    } else {
        cout << "[ERROR] Invalid key-door path!" << endl;
    }

    cout << "--------------------------------------------------" << endl << endl;
    return success;
}

/**
 * Test pathfinding on an unsolvable dungeon.
 */
bool testUnsolvableDungeon() {
    cout << "=== Unsolvable Dungeon Test ===" << endl;

    vector<string> dungeon = createUnsolvableDungeon();
    printDungeon(dungeon, "Unsolvable Test Dungeon");

    vector<Cell> path = bfsPath(dungeon);

    bool success = false;
    if (path.empty()) {
        cout << "[OK] Correctly identified unsolvable dungeon!" << endl;
        success = true;
    } else {
        cout << "[ERROR] Found path in unsolvable dungeon!" << endl;
    }

    cout << "--------------------------------------------------" << endl << endl;
    return success;
}

/**
 * Test dungeon generation with different sizes.
 */
bool testDungeonGeneration() {
    cout << "=== Dungeon Generation Test ===" << endl;

    cout << "Generating 9x9 dungeon..." << endl;
    vector<string> smallDungeon = generateDungeon(9, 9, 10);

    bool hasStart = false, hasExit = false;
    for (const string& row : smallDungeon) {
        if (row.find('S') != string::npos) hasStart = true;
        if (row.find('E') != string::npos) hasExit = true;
    }

    if (!hasStart || !hasExit) {
        cout << "[ERROR] Dungeon generation incomplete - missing start (S) or exit (E)" << endl;
        cout << "[TIP] This indicates the recursive backtracking function needs implementation" << endl;
        printDungeon(smallDungeon, "Incomplete Generated Dungeon");
        cout << "--------------------------------------------------" << endl << endl;
        return false;
    }

    printDungeon(smallDungeon, "Generated 9x9 Dungeon");
    cout << "Testing pathfinding on generated dungeon..." << endl;
    vector<Cell> path = bfsPath(smallDungeon);

    bool success = false;
    if (!path.empty()) {
        cout << "[OK] Generated dungeon is solvable! Path length: " << path.size() << endl;
        printDungeonWithPath(smallDungeon, path, "Solved Generated Dungeon");
        success = true;
    } else {
        cout << "[ERROR] No path found - this could indicate:" << endl;
        cout << "   * Maze generation created an unsolvable maze (check implementation)" << endl;
        cout << "   * BFS pathfinding has bugs (check implementation)" << endl;
        cout << "   * Functions not yet implemented (check TODO messages above)" << endl;
    }

    cout << "--------------------------------------------------" << endl << endl;
    return success;
}

/**
 * Main function that runs all test cases.
 */
int main() {
    cout << "Testing Dungeon Pathfinder Algorithms" << endl;
    cout << "================================================" << endl;
    cout << "[TIP] If functions hang or fail, check for TODO messages!" << endl;
    cout << "[TIP] Safety mechanisms will prevent infinite loops and provide guidance." << endl << endl;

    int totalTests = 5;
    int passedTests = 0;

    cout << "Running test 1/5..." << endl;
    if (testBasicPathfinding()) passedTests++;

    cout << "Running test 2/5..." << endl;
    if (testComplexPathfinding()) passedTests++;

    cout << "Running test 3/5..." << endl;
    if (testKeyDoorPathfinding()) passedTests++;

    cout << "Running test 4/5..." << endl;
    if (testUnsolvableDungeon()) passedTests++;

    cout << "Running test 5/5..." << endl;
    if (testDungeonGeneration()) passedTests++;

    cout << "================================================" << endl;
    cout << "TEST PROGRESS SUMMARY" << endl;
    cout << "================================================" << endl;
    cout << "Tests passed: " << passedTests << "/" << totalTests;

    if (passedTests == totalTests) {
        cout << " [EXCELLENT! All tests passed!]" << endl;
    } else if (passedTests >= totalTests * 0.8) {
        cout << " [GREAT! Almost there!]" << endl;
    } else if (passedTests >= totalTests * 0.5) {
        cout << " [GOOD! Making progress!]" << endl;
    } else if (passedTests > 0) {
        cout << " [GETTING STARTED! Keep going!]" << endl;
    } else {
        cout << " [START HERE! Complete the TODOs above.]" << endl;
    }
    cout << endl;
    cout << endl;
    cout << "Implementation Requirements:" << endl;
    cout << "================================================" << endl;
    cout << "[REQUIRED] Complete the TODOs in generator.cpp for maze generation" << endl;
    cout << "[REQUIRED] Complete the basic BFS in solver.cpp for pathfinding" << endl;
    cout << "[REQUIRED] Complete the 3 TODOs in bfsPathKeys() for key-door mechanics" << endl;
    cout << endl;
    cout << "Optional Challenges:" << endl;
    cout << "================================================" << endl;
#ifdef IMPLEMENT_OPTIONAL_FUNCTIONS
    cout << "[ENABLED] countReachableKeys() function - implement for bitmask practice" << endl;
#else
    cout << "[DISABLED] countReachableKeys() function - uncomment #define in solver.h to enable" << endl;
#endif

    return 0;
}
