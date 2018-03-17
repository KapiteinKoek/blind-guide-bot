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

Borderline createBorderline(Coordinate bottom, Coordinate top, enum side goodSide) {
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

int main() {
    printf("Initializing borders...\n");
    initializeBorders();
    double y = 0;
    double x;
    for (x = 1.5; x > -2; x -= 0.01) {
        printf("x: %lf, y: %lf\tresistance: %lf\n", x, y, getResistance(x, y, 0.0, -21.2, 15.0));
    }
    printf("Cleaning up...\n");
    cleanup();
}