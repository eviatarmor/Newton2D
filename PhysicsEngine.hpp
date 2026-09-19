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

#ifndef PHYSICS_ENGINE_HPP
#define PHYSICS_ENGINE_HPP
#pragma once

#include <functional>
#include <vector>

#include "CollisionResolution.hpp"
#include "Rigidbody.hpp"
#include "SurfaceGravity.hpp"

namespace Newton2D
{
    class PhysicsEngine
    {
    public:
        enum class Accuracy { Fast, Good };

        void push_back(impl::BaseRigidbody& rb) { rigids.push_back(rb); }
        void pop_back()                         { rigids.pop_back();    }
        void clear()                            { rigids.clear();       }
        void setAccuracy(Accuracy value)        { accuracy = value;     }

        void loop(float dt = 1.f / 60.f, float gravity = SurfaceGravity::Earth)
        {
            for (auto& ref : rigids)
            {
                impl::BaseRigidbody& rb = ref.get();
                if (rb.getMass() <= 0.f)
                    continue;

                VecF v = rb.getLinearVelocity();
                v.y += gravity * dt;
                rb.setLinearVelocity(v);

                VecF p = rb.getPosition();
                p.x += v.x * dt;
                p.y += v.y * dt;
                rb.setPosition(p);
            }

            const int iterations = (accuracy == Accuracy::Good) ? 8 : 4;
            for (int k = 0; k < iterations; ++k)
            {
                for (std::size_t i = 0; i < rigids.size(); ++i)
                {
                    for (std::size_t j = i + 1; j < rigids.size(); ++j)
                        impl::CollisionResolution::resolve(rigids[i].get(), rigids[j].get());
                }
            }
        }
    
    private:
        Accuracy accuracy = Accuracy::Good;
        std::vector<std::reference_wrapper<impl::BaseRigidbody>> rigids;
    };
}

#endif // PHYSICS_ENGINE_HPP
