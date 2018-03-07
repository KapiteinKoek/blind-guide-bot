#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "blindguide.h"

Coordinate createCoordinate(double x, double y) {
    Coordinate c;
    c.x = x;
    c.y = y;
    return c;
}

Borderline createBorderline(Coordinate bottom, Coordinate top, char goodSide) {
    Borderline bl;
    bl.bottom = bottom;
    bl.top = top;
    bl.length = createVector(top.x - bottom.x, top.y - bottom.y).length;
    bl.goodSide = goodSide;
    return bl;
}

Vector createVector(double x, double y) {
    Vector v;
    populateVector(x, y, &v);
    return v;
}

void populateVector(double x, double y, Vector * v) {
    v->length = sqrt(x * x + y * y);
    v->x = x / v->length;
    v->y = y / v->length;
}

void initializeBorders() {
    int numCoords = sizeof(borderCoordinates) / sizeof(borderCoordinates[0]);
    numBorderlines = numCoords / 4;
    borderlines = (struct Borderline *) malloc(numBorderlines * sizeof(struct Borderline));
    
    for (int i = 0; i < numBorderlines; i++) {
        int index = i * 4;
        borderlines[i] = createBorderline(createCoordinate(borderCoordinates[index], borderCoordinates[index + 1]), createCoordinate(borderCoordinates[index + 2], borderCoordinates[index + 3]), RIGHT);
    }
}

double getResistance(double x, double y, double phi, double forceX, double forceY) {
    #if DEBUG
        printf("\nGetting resistance for point (%lf,%lf) against direction (%lf, %lf)\n", x, y, forceX, forceY);
    #endif
    
    double resistance = 0;
    Coordinate point = createCoordinate(x, y);
    Vector force = createVector(forceX, forceY);
    Vector toBorder;
    
    for (int i = 0; i < numBorderlines; i++) {
        #if DEBUG
            printf("\nBorder %d of %d:\n", i+1, numBorderlines);
        #endif
        char approaching = approachingBorder(&point, &borderlines[i], &force, &toBorder);
        if (approaching == RESIST) {
            resistance = fmax(resistance, getBorderResistance(&force, &toBorder));
        } else if (approaching == STOP) {
            return 1.0;
        }
    }
    
    return resistance;
}

double getAcceleration(Vector * force) {
    return force->length / MASS;
}

char approachingBorder(Coordinate * p, Borderline * b, Vector * force, Vector * toBorder) {
    Coordinate * v = &(b->bottom);
    Coordinate * w = &(b->top);
    
    double l2 = b->length * b->length;
    double t = fmax(0.0, fmin(1.0, ((p->x - v->x) * (w->x - v->x) + (p->y - v->y) * (w->y - v->y)) / l2));
    double x = v->x + t * (w->x - v->x);
    double y = v->y + t * (w->y - v->y);
    
    populateVector(x - p->x, y - p->y, toBorder);
    toBorder->length -= RADIUS;
    
    double d = (p->x - v->x) * (w->y - v->y) - (p->y - v->y) * (w->x - v->x);
    
    double dot = (force->x * toBorder->x) + (force->y * toBorder->y);
    char goingToBorder = dot > 0 ? 1 : 0;
    
    #if DEBUG
        printf("vx: %lf, vy: %lf, wx: %lf, wy: %lf\n", v->x, v->y, w->x, w->y);
        printf("l2: %lf, t: %lf, x: %lf, y: %lf\n", l2, t, x, y);
        printf("To border: distance: %lf, x: %lf, y: %lf\n", toBorder->length, toBorder->x, toBorder->y);
        printf("Force: distance: %lf, x: %lf, y: %lf\n", force->length, force->x, force->y);
        printf("Good side: %s, Side: %s\n", b->goodSide == LEFT ? "left" : "right", d < 0 ? "left" : "right");
        printf("Dot: %lf, Going to border: %d\n", dot, goingToBorder);
    #endif
    
    if ((b->goodSide == LEFT && d < 0) || (b->goodSide == RIGHT && d > 0)) {
        // p is on the good side of the border
        if (goingToBorder) {
            return RESIST;
        }            
    } else {
        // p is on the bad side of the border
        if (!goingToBorder) {
            return STOP;
        }
    }
    
    return NOTHING;
}

double getBorderResistance(Vector * force, Vector * toBorder) {
    if (toBorder->length <= 0) {
        return 1.0;
    }
    
    double a = getAcceleration(force);
    double t = sqrt((2 * toBorder->length) / a);
    double resistance = (RESISTANCE_TIME - t) / (RESISTANCE_TIME - STOP_TIME);
    #if DEBUG
        printf("a: %lf, t: %lf, res: %lf\n", a, t, resistance);
    #endif
    
    return fmin(fmax(0.0, resistance), 1.0);
}

void cleanup() {
    free(borderlines);
}

/*
 * This can be used for standalone testing
 */
int main() {
    printf("Initializing borders...\n");
    initializeBorders();
    printf("\nResistance: %lf\n", getResistance(3.0, -1.0, 0.0, -0.7, 0.5));
    printf("Cleaning up...\n");
    cleanup();
}