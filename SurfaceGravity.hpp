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

#ifndef SURFACE_GRAVITIES_HPP
#define SURFACE_GRAVITIES_HPP
#pragma once

namespace Newton2D {

    // Source:
    // https://en.wikipedia.org/wiki/Surface_gravity
    namespace SurfaceGravity
    { 
        namespace detail { constexpr float g = 9.80665; }
        
        constexpr float Sun       = 28.02    * detail::g;
        constexpr float Mercury   = 0.377    * detail::g;
        constexpr float Venus     = 0.905    * detail::g;
        constexpr float Earth     =            detail::g;
        constexpr float Moon      = 0.1657   * detail::g;
        constexpr float Mars      = 0.379    * detail::g;
        constexpr float Phobos    = 0.000581 * detail::g;
        constexpr float Deimos    = 0.000306 * detail::g;
        constexpr float Ceres     = 0.029    * detail::g;
        constexpr float Jupiter   = 2.528    * detail::g;
        constexpr float Io        = 0.183    * detail::g;
        constexpr float Europa    = 0.134    * detail::g;
        constexpr float Ganymede  = 0.146    * detail::g;
        constexpr float Callisto  = 0.126    * detail::g;
        constexpr float Saturn    = 1.065    * detail::g;
        constexpr float Titan     = 0.138    * detail::g;
        constexpr float Enceladus = 0.012    * detail::g;
        constexpr float Uranus    = 0.886    * detail::g;
        constexpr float Neptune   = 1.137    * detail::g;
        constexpr float Triton    = 0.08     * detail::g;
        constexpr float Pluto     = 0.063    * detail::g;
        constexpr float Eris      = 0.084    * detail::g;
    } // SurfaceGravity
    
} // Newton2D

#endif // GRAVITIES_HPP
