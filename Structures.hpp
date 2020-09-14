/*
    This file is part of the Newton2D project
    -----------------------------------------
    
    What is Newton2D?
    Newton2D is a 2d physics engine designed to work without any needs for a 
    graphics API.

    Do you have a physics background?
    I never studied any sort of physics until this project, i'm sure there are
    stuff that may annoy you physicist.

    Does this library has documentation?
    This library is still in it's early stages, and i'm sure there are still a 
    lot of bugs that can be found. so no.

    Where does the project name come from?
    I have two dogs, one is named Flutter ( as the condition atrial flutter,
    because he's hyperactive ), and the other one is the oldest Newton ( after 
    the name of Isaac Newton ).

    ===============================================================================

    MIT License

    Copyright (c) 2020 Eviatar Mor

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef STRUCTURES_HPP
#define STRUCTURES_HPP
#pragma once

#include <vector>
#include <array>
#include <functional>
#include <memory>

#include "Vector.hpp"
#include "Angle.hpp"

namespace Newton2D {

    struct BaseShape 
    {
        virtual ~BaseShape() = default;

        float moment_of_inertia;
        float mass;
    }; // BaseShape

    // ----------------------------------------------------------------------- //

    struct PolygonShape : public BaseShape 
    {
        PolygonShape() = default;
        PolygonShape(const std::vector<VecF>& pts) 
            : points(pts) {}
        PolygonShape(std::initializer_list<VecF>&& pts) 
            : points(pts.begin(), pts.end()) {}

        std::vector<VecF> points;
    }; // PolygonShape

    // ----------------------------------------------------------------------- //

    struct QuadShape : public BaseShape 
    {
        QuadShape() = default;
        QuadShape(unsigned int w, unsigned h) 
            : width(w), height(h) {}

        unsigned int width, height;
    }; // QuadShape

    // ----------------------------------------------------------------------- //

    struct LineSegmentShape : public BaseShape 
    {
        LineSegmentShape() = default;
        LineSegmentShape(VecF p1, VecF p2)
            : points{p1, p2} {}
        LineSegmentShape(const std::array<VecF, 2>& pts)
            : points(pts) {}
     
        std::array<VecF, 2> points;
    }; // LineSegmentShape

    // ----------------------------------------------------------------------- //

    struct CircleShape : public BaseShape
    {
        CircleShape() = default;
        CircleShape(float r) 
            : radius(r) {}

        float radius;
    }; // CircleShape

    // ----------------------------------------------------------------------- //

    struct Particle
    {
        VecF  pos;   // position
        VecF  lvel;  // linear velocity
        Angle angle; // orientation
    }; // Particle

}

#endif // STRUCTURES_HPP
