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

#ifndef VECTOR_HPP
#define VECTOR_HPP
#pragma once

#include <type_traits>
#include <cmath>
#include <SFML/Graphics.hpp>

namespace Newton2D {
    
    // This container only except types arithmetic types.
    template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    struct Vector
    {
        T x, y;

        Vector() = default;

        Vector(T _x, T _y)
            : x(_x), y(_y) {}
        
        Vector(const Vector<T>& other)
            : x(other.x), y(other.y) {}

        Vector(Vector<T>&& other) 
            : x(other.x), y(other.y) {}

        Vector& operator=(const Vector<T>& other) {
            x = other.x;
            y = other.y;
            return *this;
        }
        
        Vector& operator=(Vector<T>&& other) {
            x = other.x;
            y = other.y;
            return *this;
        }

        /*
         *   Addition and assignment between vectors.
         */
        Vector& operator+=(const Vector<T>& other) {
            x += other.x;
            y += other.y;
            return *this;
        }
        /*
         *   Substraction and assignment between vectors.
         */

        Vector& operator-=(const Vector<T>& other) {
            x -= other.x;
            y -= other.y;
            return *this;
        }

        /*
         *   Division and assignment between vectors.
         */
        Vector& operator/=(const Vector<T>& other) {
            x /= other.x;
            y /= other.y;
            return *this;
        }

        /*
         *   Multiplying and assignment between vectors.
         */
        Vector& operator*=(const Vector<T>& other) {
            x *= other.x;
            y *= other.y;
            return *this;
        }
        
        /*
         *   Addition between vectors.
         */
        Vector operator+(const Vector<T>& other) {
            return {x + other.x, y + other.y};
        }

        /*
         *   Substraction between vectors.
         */
        Vector operator-(const Vector<T>& other) {
            return {x - other.x, y - other.y};
        }

        /*
         *   Division between vectors.
         */
        Vector operator/(const Vector<T>& other) {
            return {x / other.x, y / other.y};
        }

        /*
         *   Multiplying between vectors.
         */
        Vector operator*(const Vector<T>& other) {
            return {x * other.x, y * other.y};
        }

        /*
         *   return true if both vectors are the same.
         */
        bool operator==(const Vector<T>& other) {
            return x == other.x && y = other.y;
        }

        /*
         *   return true if both vectors are different.
         */
        bool operator!=(const Vector<T>& other) {
            return !(*this == other);
        }

        /*
         *   Magnitude of the current vector.
         */
        float mag() const {
            return sqrtf(x * x + y * y);
        }

        /*
         *   The angle of the current vector.
         */
        float ang() const {
            return atanf(y / x);
        }

        /*
         *   Converting a vector to a tangental vector.
         */
        Vector<float> tan() {
            return Vector<float>(-y, x);
        }

        /*
         *   Dot product between two vectors.
         */
        template<typename U, typename V>
        static float dot(const Vector<U>& lhs, const Vector<V>& rhs) {
            return lhs.x * rhs.x + lhs.y * rhs.y;
        }

        /*
         *   Cross product between two vectors.
         */
        template<typename U, typename V>
        static float cross(const Vector<U>& lhs, const Vector<V>& rhs) {
            return lhs.x * rhs.y - lhs.y * rhs.x;
        }

        /*
         *   Converting a vector to unit vector.
         */
        template<typename U>
        static Vector<float> norm(const Vector<U>& vec) {
            return Vector<float>(vec.x / vec.mag(), vec.y / vec.mag());
        }

    }; // Vector

    using VecI  = Vector<int>;
    using VecUI = Vector<unsigned int>;
    using VecF  = Vector<float>;
    using VecD  = Vector<double>;
}
#endif // VECTOR_HPP
