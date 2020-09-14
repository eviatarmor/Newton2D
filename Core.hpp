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

#ifndef CORE_HPP
#define CORE_HPP
#pragma once

#include <cmath>

#include "Structures.hpp"
#include "Vector.hpp"
#include "Rigidbody.hpp"

namespace Newton2D {
    namespace impl {

        VecF transform_polygon_point(const VecF& point, const VecF& pos, Angle angle) 
        {
            VecF vec = point;
            vec *= pos;

            // https://en.wikipedia.org/wiki/Rotation_matrix
            vec.x = vec.x * cosf(angle.radians()) - vec.y * sinf(angle.radians());
            vec.y = vec.x * sinf(angle.radians()) + vec.y * cosf(angle.radians());

            return vec;
        }

        // https://imada.sdu.dk/~rolf/Edu/DM815/E10/2dcollisions.pdf
        void conservation_of_momentum(const VecF& pos1, const VecF& v1, float m1,
                                    const VecF& pos2, const VecF& v2, float m2,
                                    VecF& nv1, VecF nv2)
        {
            VecF norm = VecF::norm(VecF(pos2.x - pos1.x, pos2.y - pos1.y));
            VecF tan  = norm.tan();

            const float dot_tan1 = VecF::dot(v1, tan);
            const float dot_tan2 = VecF::dot(v2, tan);

            const float dot_norm1 = VecF::dot(v1, norm);
            const float dot_norm2 = VecF::dot(v2, norm);

            float p1 = (dot_norm1 * (m1 - m2) + 2.f * m2 * dot_norm2) / (m1 + m2);
            float p2 = (dot_norm2 * (m2 - m1) + 2.f * m1 * dot_norm1) / (m1 + m2);

            nv1.x = tan.x * dot_tan1 + norm.x * p1;
            nv1.y = tan.y * dot_tan1 + norm.y * p1;
            nv2.x = tan.x * dot_tan2 + norm.x * p2;
            nv2.y = tan.y * dot_tan2 + norm.y * p2;
        }

    } // impl
} // Newton2D

#endif // CORE_HPP
