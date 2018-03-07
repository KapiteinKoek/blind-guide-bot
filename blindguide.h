// Do we need debug output?
#define DEBUG 1

// Mass of the robot in kg
#define MASS 30
// Radius around the center of the robot that must stay between the borders in meter
#define RADIUS 0.8
// When the robot reaches the border within this many seconds, stop immediately
#define STOP_TIME 0.5
// When the robot reaches the border within this many seconds, start resisting
#define RESISTANCE_TIME 3.5

// THESE DO NOT NEED TO BE CHANGED
#define LEFT -1
#define RIGHT 1
#define NOTHING 0
#define RESIST 1
#define STOP 2

/* 
 * The coordinates for the initial borders (bottom_x, bottom_y, top_x, top_y)
 * Note that the border is assumed to be safe to the right of this line
 * So define the top and bottom accordingly
 */
double borderCoordinates[8] = {
    -1.5, -9.0,
    -1.5, 9.0,
    1.5, 9.0,
    1.5, -9.0
};

// A 2D coordinate structure
typedef struct Coordinate {
    double x;
    double y;
} Coordinate;

/*
 * Definition of a border line structure
 * bottom: Coordinate
 * top: Coordinate
 * length: length of border line
 * goodSide: side which is considered 'safe' (LEFT or RIGHT)
 */
typedef struct Borderline {
    struct Coordinate bottom;
    struct Coordinate top;
    double length;
    char goodSide; 
} Borderline;

/*
 * A 2D vector structure with additional length value
 * Use as unit vector recommended (hence the length)
 * Unit vector can be constructed using createVector()
 */
typedef struct Vector {
    double x;
    double y;
    double length;
} Vector;

// The number of border lines
int numBorderlines;
// Pointer to an array of border line structures
struct Borderline * borderlines;

/*
 * Creates and returns a Coordinate structure with the given x and y coordinates.
 */
Coordinate createCoordinate(double x, double y);

/*
 * Creates and returns a Borderline structure with the given bottom and top Coordinates
 * and the given good side.
 * Also computes and stores the length of this border line.
 */
Borderline createBorderline(Coordinate bottom, Coordinate top, char goodSide);

/*
 * Creates and returns a Vector structure for the given x and y values.
 * Calls createVector().
 */
Vector createVector(double x, double y);

/*
 * Fills the given Vector structure with the given x and y coordinates.
 * Will convert the vector to a unit vector.
 * Computes and stores the length of the original vector.
 */
void populateVector(double x, double y, Vector * v);

/*
 * Creates and fills the borderlines array based on the values in borderCoordinates.
 */
void initializeBorders();

/*
 * Compute the necessary resistance when the robot is at the position defined by x, y and phi
 * and the robot is pushed according to forceX and forceY, where forceX and forceY constitute the
 * force vector.
 * Note that the units of x and y are assumed to be meters, and forceX and forceY Newtons.
 */
double getResistance(double x, double y, double phi, double forceX, double forceY);

/*
 * Calculate the acceleration along the given force vector.
 * Uses the defined MASS of the robot.
 * Formula: a = F / m.
 */
double getAcceleration(Vector * force);

/*
 * Determines whether the robot at position p with the predefined RADIUS is approaching border line b
 * when it is pushed along force vector force.
 * toBorder will be populated by the vector from p to the closest point along the border line.
 * If the robot is on the 'good' side of the border, and the robot is pushed towards the border, returns RESIST.
 * If the robot is on the 'bad' side of the border, and the robot is pushed away from the border, returns STOP.
 * Else, returns NOTHING.
 */
char approachingBorder(Coordinate * p, Borderline * b, Vector * force, Vector * toBorder);

/*
 * Determines the resistance needed when the robot is pushed with the given force along the toBorder vector.
 * Uses RESISTANCE_TIME and STOP_TIME to create a linearly increasing resistance when the time to traverse
 * the toBorder vector decreases.
 * Return value is clamped between 0 and 1.
 */
double getBorderResistance(Vector * force, Vector * toBorder);

/*
 * Frees the memory allocated for the borderlines array.
 */
void cleanup();