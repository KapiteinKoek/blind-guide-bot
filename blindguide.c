/*
 * Copyright 2018 Anne Kolmans, Dylan ter Veen, Jarno Brils, Renée van Hijfte, and Thomas Wiepking (TU/e Project Robots Everywhere 2017/2018 Q3 Group 12)
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
    
    for (int i = 0; i < numBorderlines; i++) {
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
    
    for (int i = 0; i < borderlines.size; i++) {
        #if DEBUG
            printf("\nBorder %d of %d:\n", i+1, borderlines.size);
        #endif
        enum action a = approachingBorder(&point, &(borderlines.borderlines[i]), &force, &toBorder);
        if (a == RESIST) {
            resistance = fmax(resistance, getBorderResistance(&force, &toBorder));
        } else if (a == STOP) {
            return 1.0;
        }
    }
    
    return resistance;
}

double getAcceleration(Vector * force) {
    return force->length / MASS;
}

enum action approachingBorder(Coordinate * p, Borderline * b, Vector * force, Vector * toBorder) {
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
    freeBorderlineArray(&borderlines);
}

/*
 * This can be used for standalone testing
 */
int main() {
    printf("Initializing borders...\n");
    initializeBorders();
    printf("\nResistance: %lf\n", getResistance(0.6, 0.0, 0.0, 0.7, 0.5));
    printf("Cleaning up...\n");
    cleanup();
}