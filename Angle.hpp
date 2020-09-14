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


#ifndef ANGLE_HPP
#define ANGLE_HPP
#pragma once

namespace Newton2D {

    enum class AngularUnit { Radians, Degrees };

    // The default angular unit of this class is radians
    class Angle
    {
    private:
        static constexpr float PI = 3.1415926535897932384626433;

    public:
        /*
         *   Expecting an angle and it's angular unit.
         */
        explicit Angle(float value, AngularUnit unit) noexcept { 
            rad = unit == AngularUnit::Radians ? rad : ((value) * PI / 180.f); 
        }

        /*
         *   Expecting an angle in radians.
         */
        Angle(float radians) { rad = radians; }

        Angle() = default;

        /*
         *   returning the angle in radians.
         */
        inline float radians() const { return rad; }

        /*
         *   returning the angle in degrees.
         */
        inline float degrees() const { return ((rad) * 180.0 / PI); }

        /*
         *   returning the angle in radians.
         *   more convenient to work with computing floats.
         */
        inline operator float&() { return rad; }

    private:
        float rad;
    }; // Angle

} // Newton2D

#endif // ANGLE_HPP
