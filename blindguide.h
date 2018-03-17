/*
 * Copyright 2018 Anne Kolmans, Dylan ter Veen, Jarno Brils, Ren??e van Hijfte, and Thomas Wiepking (TU/e Project Robots Everywhere 2017/2018 Q3 Group 12)
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Do we need debug output?
#define DEBUG 0

// Mass of the robot in kg
#define MASS 30
// Radius around the center of the robot that must stay between the borders in meter
#define RADIUS 0.3
// When the robot reaches the border within this many seconds, stop immediately
#define STOP_TIME 0.5
// When the robot reaches the border within this many seconds, start resisting
#define RESISTANCE_TIME 3.5

// THESE DO NOT NEED TO BE CHANGED
enum action {NOTHING, RESIST, STOP};
enum side {LEFT, RIGHT};

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
    enum side goodSide;
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

/*
 * Dynamic array structure that holds Borderline structures in an array.
 * size indicates the number of elements that are currently present.
 * capacity indicates the current maximum capacity of the dynamic array.
 */
typedef struct BorderlineArray {
    struct Borderline * borderlines;
    size_t size;
    size_t capacity;
} BorderlineArray;

// Structure that holds all borderlines
BorderlineArray borderlines;

/*
 * Populates the given BorderlineArray structure, such that it is an empty array of capacity size.
 */
void createBorderlineArray(BorderlineArray * ba, size_t size);

/*
 * Adds the given Borderline element to the specified BorderlineArray.
 * If necessary, increases the capacity of the BorderlineArray array, multiplying its current capacity by two.
 */
void addToBorderlineArray(BorderlineArray * ba, Borderline element);

/*
 * Frees the memory allocated to the given BorderlineArray structure.
 * Also resets the size and capacity to zero.
 */
void freeBorderlineArray(BorderlineArray * ba);

/*
 * Creates and returns a Coordinate structure with the given x and y coordinates.
 */
Coordinate createCoordinate(double x, double y);

/*
 * Creates and returns a Borderline structure with the given bottom and top Coordinates
 * and the given good side.
 * Also computes and stores the length of this border line.
 */
Borderline createBorderline(Coordinate bottom, Coordinate top, enum side goodSide);

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
 * Adds a border to the border lines array, where the border is specified by the bottom and top x and y coordinates.
 * goodSide indicates which side of the border, given the bottom and top coordinates, the robot should stay on.
 */
void addBorder(double bottomX, double bottomY, double topX, double topY, enum side goodSide);

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
 * Calculates and returns the dot product of the two given vectors.
 */
double dot(Vector * v1, Vector * v2);

/*
 * Determines whether the robot at position p with the predefined RADIUS is approaching border line b
 * when it is pushed along force vector force.
 * toBorder will be populated by the vector from p to the closest point along the border line.
 * If the robot is on the 'good' side of the border, and the robot is pushed towards the border, returns RESIST.
 * If the robot is on the 'bad' side of the border, and the robot is pushed away from the border, returns STOP.
 * Else, returns NOTHING.
 */
enum action approachingBorder(Coordinate * p, Borderline * b, Vector * force, Vector * toBorder);

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


void populateVector(double x, double y, Vector * v) {
    v->length = sqrt(x * x + y * y);
    v->x = x / v->length;
    v->y = y / v->length;
}

void createBorderlineArray(BorderlineArray * ba, size_t size) {
    ba->borderlines = (struct Borderline *) malloc(size * sizeof(struct Borderline));
    ba->size = 0;
    ba->capacity = size;
}

void addToBorderlineArray(BorderlineArray * ba, Borderline element) {
    if (ba->size >= ba->capacity) {
        ba->capacity *= 2;
        ba->borderlines = (struct Borderline *) realloc(ba->borderlines, ba->capacity * sizeof(struct Borderline));
    }
    ba->borderlines[ba->size++] = element;
}

void freeBorderlineArray(BorderlineArray * ba) {
    free(ba->borderlines);
    ba->borderlines = NULL;
    ba->size = ba->capacity = 0;
}

void initializeBorders() {
    size_t numCoords = sizeof(borderCoordinates) / sizeof(borderCoordinates[0]);
    size_t numBorderlines = numCoords / 4;
    createBorderlineArray(&borderlines, numBorderlines);
    unsigned int i = 0;
    for (i = 0; i < numBorderlines; i++) {
        int index = i * 4;
        addBorder(borderCoordinates[index], borderCoordinates[index + 1], borderCoordinates[index + 2], borderCoordinates[index + 3], RIGHT);
    }
}

void addBorder(double bottomX, double bottomY, double topX, double topY, enum side goodSide) {
    if (borderlines.borderlines == NULL || borderlines.capacity <= 0) {
        createBorderlineArray(&borderlines, 1);
    }
    struct Coordinate bottom = createCoordinate(bottomX, bottomY);
    struct Coordinate top = createCoordinate(topX, topY);
    struct Borderline bl = createBorderline(bottom, top, goodSide);
    addToBorderlineArray(&borderlines, bl);
}

double getResistance(double x, double y, double phi, double forceX, double forceY) {
    #if DEBUG
        printf("\nGetting resistance for point (%lf,%lf) against direction (%lf, %lf)\n", x, y, forceX, forceY);
    #endif
    
    double resistance = 0;
    Coordinate point = createCoordinate(x, y);
    Vector force = createVector(forceX, forceY);
    Vector toBorder;
    unsigned int i = 0;
    for (i = 0; i < borderlines.size; i++) {
        #if DEBUG
            printf("\nBorder %d of %d:\n", i+1, borderlines.size);
        #endif
        // Determine the necessary action for the current border line
        // The toBorder vector will also be populated accordingly
        enum action a = approachingBorder(&point, &(borderlines.borderlines[i]), &force, &toBorder);
        if (a == RESIST) {
            // Determine the maximum of the current resistance and the calculated resistance that is necessary for the given force and toBorder vectors
            resistance = fmax(resistance, getBorderResistance(&force, &toBorder));
        } else if (a == STOP) {
            // If STOP is required, resist fully
            return 1.0;
        }
    }
    
    return resistance;
}

double getAcceleration(Vector * force) {
    return force->length / MASS;
}

double dot(Vector * v1, Vector * v2) {
    return (v1->x * v2->x) + (v1->y * v2->y);
}

enum action approachingBorder(Coordinate * p, Borderline * b, Vector * force, Vector * toBorder) {
    Coordinate * v = &(b->bottom);
    Coordinate * w = &(b->top);
    
    // Determine the squared length of the border line
    double l2 = b->length * b->length;
    // Now determine parameter t, which indicates to what fraction of the border line, point p is closest
    double t = fmax(0.0, fmin(1.0, ((p->x - v->x) * (w->x - v->x) + (p->y - v->y) * (w->y - v->y)) / l2));
    // Use parameter t to get absolute x and y coordinates along the border line
    double x = v->x + t * (w->x - v->x);
    double y = v->y + t * (w->y - v->y);
    
    // Fill the toBorder vector using the given point and the determined border line point
    populateVector(x - p->x, y - p->y, toBorder);
    // Account for the radius of the robot (by reducing the length of the toBorder vector)
    toBorder->length -= RADIUS;
    
    // Calculate on what side of the border line p is. d < 0 means LEFT, d > means right
    double d = (p->x - v->x) * (w->y - v->y) - (p->y - v->y) * (w->x - v->x);
    
    // Get the cosine of the angle between the force and toBorder vectors
    double angle = dot(force, toBorder);
    // If this value is positive, then force is going towards the border, if it is negative, force is going away
    char goingToBorder = angle > 0 ? 1 : 0;
    
    #if DEBUG
        printf("vx: %lf, vy: %lf, wx: %lf, wy: %lf\n", v->x, v->y, w->x, w->y);
        printf("l2: %lf, t: %lf, x: %lf, y: %lf\n", l2, t, x, y);
        printf("To border: distance: %lf, x: %lf, y: %lf\n", toBorder->length, toBorder->x, toBorder->y);
        printf("Force: distance: %lf, x: %lf, y: %lf\n", force->length, force->x, force->y);
        printf("Good side: %s, Side: %s\n", b->goodSide == LEFT ? "left" : "right", d < 0 ? "left" : "right");
        printf("Angle: %lf, Going to border: %d\n", angle, goingToBorder);
    #endif
    
    if ((b->goodSide == LEFT && d < 0) || (b->goodSide == RIGHT && d > 0)) {
        // p is on the good side of the border
        if (goingToBorder) {
            // The robot is moving towards the border, so RESIST
            // Note that the amount of resistance is determined later (this can even be 0 or 1)
            return RESIST;
        }            
    } else {
        // p is on the bad side of the border
        if (!goingToBorder) {
            // The robot is moving further away from the border, so STOP
            return STOP;
        }
    }
    
    // No special circumstance, the robot is not moving towards the border, or it is going back across
    return NOTHING;
}

double getBorderResistance(Vector * force, Vector * toBorder) {
    if (toBorder->length <= 0) {
        return 1.0;
    }
    
    // Determine the angle between the force and toBorder vectors
    // Then use this to scale the distance to the border (larger angle => greater distance)
    // Then determine the acceleration and with that the time to traverse this scaled distance
    // Now linearly interpolate the resistance based on the parameters
    double cos = dot(force, toBorder);
    double dist = toBorder->length / cos;
    double a = getAcceleration(force);
    double t = sqrt((2 * dist) / a);
    double resistance = (RESISTANCE_TIME - t) / (RESISTANCE_TIME - STOP_TIME);
    #if DEBUG
        printf("length: %lf\tcos: %lf\tdist: %lf\n", toBorder->length, cos, dist);
        printf("a: %lf, t: %lf, res: %lf\n", a, t, resistance);
    #endif
    
    // Remove 0.5 from the resistance, to account for static resistance of the robot, then multiply the resistance by two, and finally clamp it between 0 and 1
    return fmin(fmax(0.0, (resistance - 0.5) * 2.0), 1.0);
}

void cleanup() {
    freeBorderlineArray(&borderlines);
}