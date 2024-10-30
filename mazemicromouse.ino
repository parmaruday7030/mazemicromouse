#define IR_SENSOR_LEFT 2
#define IR_SENSOR_RIGHT 3
#define TRIGGER_PIN 4
#define ECHO_PIN 5

// Motor control pins
#define MOTOR_LEFT_FORWARD 6
#define MOTOR_LEFT_BACKWARD 7
#define MOTOR_RIGHT_FORWARD 8
#define MOTOR_RIGHT_BACKWARD 9

#define MAZE_SIZE 64
#define CELL_SIZE 10 // cm
#define WALL_THRESHOLD 20 // Distance in cm to detect a wall

#define GOAL_CELL 27 // Goal cell value (3,3) in a 0-indexed array

int maze[MAZE_SIZE]; // Each index corresponds to a cell in the 8x8 grid
int currentCell = 0; // Start from the initial cell
int previousCell = -1; // Track the last cell for basic backtracking

void setup() {
    Serial.begin(9600);
    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Motor control pins setup
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

    initializeMaze();
}

void loop() {
    while (currentCell != GOAL_CELL) { // Run until the goal is reached
        updateMazeWithWallInfo(currentCell); // Update maze values based on current position and sensors
        floodFill(currentCell); // Update values

        int nextCell = getNextMove(currentCell); // Get the next move

        // If stuck (no valid next move), backtrack to the previous cell
        if (nextCell == -1) {
            Serial.println("Stuck! Backtracking to the previous cell.");
            nextCell = previousCell; // Go back to the last cell
            if (nextCell == -1) {
                Serial.println("No previous cell to backtrack to. Stopping.");
                break; // Stop if there are no previous cells to explore
            }
            moveToCell(nextCell); // Move the robot to the previous cell
        } else {
            moveToCell(nextCell); // Move the robot to the next cell
        }

        previousCell = currentCell; // Update the previous cell
        currentCell = nextCell; // Update current cell
        delay(100); // Small delay for stability
    }

    Serial.println("Goal Reached!");
}

// Function to initialize the maze values
void initializeMaze() {
    for (int i = 0; i < MAZE_SIZE; i++) {
        maze[i] = 100; // High value (representing "infinity")
    }
    maze[GOAL_CELL] = 0; // Set the goal cell value to 0
}

// Function to read the distance from the sonar sensor
long readSonar() {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    return (duration / 2) * 0.0343; // Convert to cm
}

// Update the maze values based on wall detection
void updateMazeWithWallInfo(int currentCell) {
    long distance = readSonar();

    // Update based on sonar distance
    if (distance < WALL_THRESHOLD) {
        markWallInMaze(currentCell, distance);
    }

    // Update based on IR sensors
    int irLeft = digitalRead(IR_SENSOR_LEFT);
    int irRight = digitalRead(IR_SENSOR_RIGHT);
    
    if (irLeft == LOW) {
        markWallInMaze(currentCell, -1); // Assuming wall on left
    }
    if (irRight == LOW) {
        markWallInMaze(currentCell, 1); // Assuming wall on right
    }
}

// Mark walls based on current cell
void markWallInMaze(int currentCell, int direction) {
    int neighborCell;
    if (direction == -1) { // Wall on the left
        neighborCell = currentCell - 1; // Move left in 1D array
    } else if (direction == 1) { // Wall on the right
        neighborCell = currentCell + 1; // Move right in 1D array
    } else {
        return; // No wall detected
    }

    if (neighborCell >= 0 && neighborCell < MAZE_SIZE) {
        maze[neighborCell] = 0; // Mark as wall
    }
}

// Flood fill algorithm to update maze values
void floodFill(int currentCell) {
    int currentValue = maze[currentCell];
    int neighbors[4] = {currentCell - 8, currentCell + 8, currentCell - 1, currentCell + 1}; // Up, Down, Left, Right

    for (int i = 0; i < 4; i++) {
        if (neighbors[i] >= 0 && neighbors[i] < MAZE_SIZE) { // Ensure within bounds
            if (maze[neighbors[i]] > currentValue + 1) {
                maze[neighbors[i]] = currentValue + 1; // Update value
                floodFill(neighbors[i]); // Recursive call to continue filling
            }
        }
    }
}

// Get the next move based on the updated maze
int getNextMove(int currentCell) {
    int neighbors[4] = {currentCell - 8, currentCell + 8, currentCell - 1, currentCell + 1}; // Up, Down, Left, Right
    int nextCell = -1;
    int minValue = 100; // Start with a high value

    for (int i = 0; i < 4; i++) {
        if (neighbors[i] >= 0 && neighbors[i] < MAZE_SIZE && maze[neighbors[i]] < minValue) {
            minValue = maze[neighbors[i]];
            nextCell = neighbors[i];
        }
    }
    return nextCell; // Return the next move cell
}

// Function to move the robot to the next cell
void moveToCell(int nextCell) {
    if (nextCell == currentCell + 1) {
        moveForward();
    } else if (nextCell == currentCell - 1) {
        moveBackward();
    } else if (nextCell == currentCell + 8) {
        moveRight();
    } else if (nextCell == currentCell - 8) {
        moveLeft();
    }
}

// Movement functions
void moveForward() {
    Serial.println("Moving Forward");
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    delay(500); // Move forward for 500 ms (adjust based on speed)
    stopMotors();
}

void moveBackward() {
    Serial.println("Moving Backward");
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
    delay(500); // Move backward for 500 ms (adjust based on speed)
    stopMotors();
}

void moveLeft() {
    Serial.println("Turning Left");
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    delay(500); // Turn left for 500 ms (adjust based on speed)
    stopMotors();
}

void moveRight() {
    Serial.println("Turning Right");
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
    delay(500); // Turn right for 500 ms (adjust based on speed)
    stopMotors();
}

// Function to stop all motors
void stopMotors() {
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}
