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

#ifndef COLLISION_RESOLUTION_HPP
#define COLLISION_RESOLUTION_HPP
#pragma once

#include <limits>
#include <algorithm>
#include <cassert>

#include "Structures.hpp"
#include "Core.hpp"
#include "Rigidbody.hpp"

// namespace Newton2D {

//     namespace impl {

    //     class CollisionResolution
    //     {
    //     private:
    //         // Circle - Circle Detection
    //         template<typename T>
    //         bool circle_circle_collision(const Rigidbody<T>& rb1, const Rigidbody<T>& rb2) 
    //         {
    //             const auto dx = rb1.particle.position.x - rb2.particle.position.x;
    //             const auto dy = rb1.particle.position.y - rb2.particle.position.y;
    //             const float dist = sqrtf(dx * dx + dy * dy);

    //             m_overlap = dist <= rb1.shape.circle.radius + rb2.shape.circle.radius;

    //             if(m_overlap)
    //             {
    //                 const float overlap = OVERLAP_FIX * (dist - rb1.shape.circle.radius - rb2.shape.circle.radius);

    //                 rb1_overlap_fix.x = -(overlap * dx / dist);
    //                 rb1_overlap_fix.y = -(overlap * dy / dist);
    //                 rb2_overlap_fix.x =  (overlap * dx / dist);
    //                 rb2_overlap_fix.y =  (overlap * dy / dist);
    //             }
    //             else
    //             {
    //                 rb1_overlap_fix.x = 0.f;
    //                 rb1_overlap_fix.y = 0.f;
    //                 rb2_overlap_fix.x = 0.f;
    //                 rb2_overlap_fix.y = 0.f;
    //             }
    //         }

    //         // Polygon - Polygon Detection
    //         // Separating Axis Theorem: 
    //         // https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169
    //         template<typename T>
    //         bool polygon_polygon_collision(const Rigidbody<T>& rb1, const Rigidbody<T>& rb2)
    //         {
    //             float overlap = std::numeric_limits<float>::infinity();

    //             for(size_t i = 0; i < rb1.shape.poly.points.size(); i++)
    //             {
    //                 auto current = transform_polygon_point(rb1.getPosition(), rb1.shape.poly.points[i], rb1.particle.angle);
    //                 auto next    = transform_polygon_point(rb1.getPosition(), rb1.shape.poly.points[(i + 1) % rb1.shape.poly.points.size()], rb1.particle.angle);
    //                 auto edge    = next - current;

    //                 auto axis = VecF::tan(edge);

    //                 auto rb1_max_proj = -std::numeric_limits<float>::infinity();
    //                 auto rb1_min_proj =  std::numeric_limits<float>::infinity();

    //                 for(size_t p = 0; p < rb1.shape.poly.points.size(); p++) 
    //                 {
    //                     auto vec = transform_polygon_point(rb1.getPosition(), rb1.shape.poly.points[p], rb1.particle.angle);
    //                     auto proj = VecF::dot(vec, axis);

    //                     rb1_max_proj = std::max(rb1_max_proj, proj);
    //                     rb1_min_proj = std::min(rb1_min_proj, proj);
    //                 }

    //                 auto rb2_max_proj = -std::numeric_limits<float>::infinity();
    //                 auto rb2_min_proj =  std::numeric_limits<float>::infinity();

    //                 for(size_t p = 0; p < rb2.shape.poly.points.size(); p++) 
    //                 {
    //                     auto vec = transform_polygon_point(rb2.getPosition(), rb2.shape.poly.points[p], rb2.particle.angle);
    //                     auto proj = VecF::dot(vec, axis);

    //                     rb2_max_proj = std::max(rb2_max_proj, proj);
    //                     rb2_min_proj = std::min(rb2_min_proj, proj);
    //                 }

    //                 overlap = std::min(std::min(rb1_max_proj, rb2_max_proj) - std::max(rb1_min_proj, rb2_min_proj), overlap);

    //                 if(rb1_max_proj < rb2_min_proj || rb1_min_proj > rb2_max_proj) 
    //                 {
    //                     rb1_overlap_fix.x = 0.f;
    //                     rb1_overlap_fix.y = 0.f;
    //                     rb2_overlap_fix.x = 0.f;
    //                     rb2_overlap_fix.y = 0.f;

    //                     return false;
    //                 }
    //             }

    //             const auto dx = rb1.particle.position.x - rb2.particle.position.x;
    //             const auto dy = rb1.particle.position.y - rb2.particle.position.y;
    //             const float dist = sqrtf(dx * dx + dy * dy);

    //             rb1_overlap_fix.x = -(OVERLAP_FIX * overlap * dx / dist);
    //             rb1_overlap_fix.y = -(OVERLAP_FIX * overlap * dy / dist);
    //             rb2_overlap_fix.x =  (OVERLAP_FIX * overlap * dx / dist);
    //             rb2_overlap_fix.y =  (OVERLAP_FIX * overlap * dy / dist);

    //             return true;
    //         }

    //     public:
    //         CollisionResolution(const Rigidbody& rb1, const Rigidbody& rb2)
    //         {
    //             if(rb1.type == Rigidbody::ShapeType::Circle && rb2.type == Rigidbody::ShapeType::Circle)
    //                 circle_circle_collision(rb1, rb2);
    //             else
    //                 polygon_polygon_collision(rb1, rb2);
    //         }

    //         VecF rb1_overlap_fix, rb2_overlap_fix;
    //         bool m_overlap;
        
    //     private:
    //         static constexpr float OVERLAP_FIX = 0.1f;
    //     };
    // } // impl


// } // Newton2D

#endif // COLLISION_RESOLUTION_HPP
