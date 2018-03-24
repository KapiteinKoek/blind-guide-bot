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
    double x = -4;
    double phi = 0;
    double ox = -2;
    double oy = 0.6;
    for (ox = -2; ox <= 2; ox += 0.2) {
        //printf("%lf\n", phi);
        double obstacles[4] = {ox, oy, 0, 1.5};
        //printf("%lf\n", getResistance(0, 0, 0.5 * PI, 0, 20, 2, obstacles));
    }
    //for (x = 1.5; x > -2; x -= 0.01) {
        //printf("\nx: %lf, y: %lf\tresistance: %lf\n", x, y, getResistance(x, y, 0.0, -21.2, 15.0));
    //}
    double obstacles[2] = {1, -2};
        printf("%lf\n", getResistance(0, 0, 2.5 * PI, 10, -7, 1, obstacles));
        printf("%lf\n", getResistance(0, 0, 2.75 * PI, -10, -7, 1, obstacles));
    printf("\nCleaning up...\n");
    cleanup();
}