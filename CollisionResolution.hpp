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

#include <algorithm>
#include <cmath>

#include "Core.hpp"
#include "Rigidbody.hpp"
#include "Structures.hpp"

namespace Newton2D {
    namespace impl {

        class CollisionResolution
        {
        public:
            static constexpr float restitution        = 0.4f;
            static constexpr float correction_percent = 0.8f;
            static constexpr float slop               = 0.05f;
            static constexpr float rest_threshold     = 25.f;

            static void resolve(BaseRigidbody& a, BaseRigidbody& b)
            {
                struct AgainstCircle : ShapeVisitor
                {
                    AgainstCircle(BaseRigidbody& a_rb, CircleShape& ca, BaseRigidbody& b_rb)
                        : a_rb(a_rb), ca(ca), b_rb(b_rb) {}

                    BaseRigidbody& a_rb;
                    CircleShape& ca;
                    BaseRigidbody& b_rb;

                    void visit(CircleShape& cb) override
                    {
                        circle_circle(a_rb, ca, b_rb, cb);
                    }

                    void visit(QuadShape& qb) override
                    {
                        circle_quad(a_rb, ca, b_rb, qb);
                    }

                    void visit(PolygonShape&) override {}
                    void visit(LineSegmentShape&) override {}
                };

                struct AgainstQuad : ShapeVisitor
                {
                    AgainstQuad(BaseRigidbody& a_rb, QuadShape& qa, BaseRigidbody& b_rb)
                        : a_rb(a_rb), qa(qa), b_rb(b_rb) {}

                    BaseRigidbody& a_rb;
                    QuadShape& qa;
                    BaseRigidbody& b_rb;

                    void visit(CircleShape& cb) override
                    {
                        circle_quad(b_rb, cb, a_rb, qa);
                    }

                    void visit(QuadShape&) override {}
                    void visit(PolygonShape&) override {}
                    void visit(LineSegmentShape&) override {}
                };

                struct First : ShapeVisitor
                {
                    First(BaseRigidbody& a_rb, BaseRigidbody& b_rb)
                        : a_rb(a_rb), b_rb(b_rb) {}

                    BaseRigidbody& a_rb;
                    BaseRigidbody& b_rb;

                    void visit(CircleShape& ca) override
                    {
                        AgainstCircle second(a_rb, ca, b_rb);
                        b_rb.accept(second);
                    }

                    void visit(QuadShape& qa) override
                    {
                        AgainstQuad second(a_rb, qa, b_rb);
                        b_rb.accept(second);
                    }

                    void visit(PolygonShape&) override {}
                    void visit(LineSegmentShape&) override {}
                } first(a, b);

                a.accept(first);
            }

        private:
            static float inv_mass(const BaseRigidbody& rb)
            {
                const float m = rb.getMass();
                return m > 0.f ? 1.f / m : 0.f;
            }

            static void apply_positional_correction(BaseRigidbody& a, BaseRigidbody& b,
                                                    const VecF& n, float penetration)
            {
                const float ima = inv_mass(a);
                const float imb = inv_mass(b);
                const float sum = ima + imb;
                if (sum <= 0.f)
                    return;

                const float mag = std::max(penetration - slop, 0.f) / sum * correction_percent;
                VecF pa = a.getPosition();
                VecF pb = b.getPosition();
                pa.x -= n.x * mag * ima;
                pa.y -= n.y * mag * ima;
                pb.x += n.x * mag * imb;
                pb.y += n.y * mag * imb;
                a.setPosition(pa);
                b.setPosition(pb);
            }

            static void apply_velocity_response(BaseRigidbody& a, BaseRigidbody& b, const VecF& n)
            {
                const float ima = inv_mass(a);
                const float imb = inv_mass(b);
                const float sum = ima + imb;
                if (sum <= 0.f)
                    return;

                VecF va = a.getLinearVelocity();
                VecF vb = b.getLinearVelocity();

                const float rvn = (vb.x - va.x) * n.x + (vb.y - va.y) * n.y;
                if (rvn > 0.f)
                    return;

                float e = restitution;
                if (std::fabs(rvn) < rest_threshold)
                    e = 0.f;

                const float j = -(1.f + e) * rvn / sum;
                va.x -= ima * j * n.x;
                va.y -= ima * j * n.y;
                vb.x += imb * j * n.x;
                vb.y += imb * j * n.y;
                a.setLinearVelocity(va);
                b.setLinearVelocity(vb);
            }

            static void circle_circle(BaseRigidbody& a, const CircleShape& sa,
                                      BaseRigidbody& b, const CircleShape& sb)
            {
                const VecF pa = a.getPosition();
                const VecF pb = b.getPosition();
                const float dx = pb.x - pa.x;
                const float dy = pb.y - pa.y;
                const float dist = sqrtf(dx * dx + dy * dy);
                const float min_dist = sa.radius + sb.radius;

                if (dist >= min_dist)
                    return;

                VecF n;
                if (dist > 1e-6f)
                {
                    n.x = dx / dist;
                    n.y = dy / dist;
                }
                else
                {
                    n.x = 0.f;
                    n.y = -1.f;
                }

                apply_positional_correction(a, b, n, min_dist - dist);
                apply_velocity_response(a, b, n);
            }

            static void circle_quad(BaseRigidbody& circle_rb, const CircleShape& circle,
                                    BaseRigidbody& quad_rb, const QuadShape& quad)
            {
                const VecF c = circle_rb.getPosition();
                const VecF q = quad_rb.getPosition();
                const float hw = static_cast<float>(quad.width)  * 0.5f;
                const float hh = static_cast<float>(quad.height) * 0.5f;
                const float left   = q.x - hw;
                const float right  = q.x + hw;
                const float top    = q.y - hh;
                const float bottom = q.y + hh;

                const bool inside = c.x > left && c.x < right && c.y > top && c.y < bottom;

                VecF n;
                float penetration = 0.f;

                if (inside)
                {
                    const float dl = c.x - left;
                    const float dr = right - c.x;
                    const float dt = c.y - top;
                    const float db = bottom - c.y;
                    const float m = std::min(std::min(dl, dr), std::min(dt, db));

                    if (m == dl)      { n = VecF(-1.f,  0.f); penetration = circle.radius + dl; }
                    else if (m == dr) { n = VecF( 1.f,  0.f); penetration = circle.radius + dr; }
                    else if (m == dt) { n = VecF( 0.f, -1.f); penetration = circle.radius + dt; }
                    else              { n = VecF( 0.f,  1.f); penetration = circle.radius + db; }
                }
                else
                {
                    const float closest_x = std::clamp(c.x, left, right);
                    const float closest_y = std::clamp(c.y, top, bottom);
                    const float dx = c.x - closest_x;
                    const float dy = c.y - closest_y;
                    const float dist = sqrtf(dx * dx + dy * dy);

                    if (dist >= circle.radius)
                        return;

                    if (dist > 1e-6f)
                    {
                        n.x = dx / dist;
                        n.y = dy / dist;
                    }
                    else
                    {
                        n.x = 0.f;
                        n.y = -1.f;
                    }
                    penetration = circle.radius - dist;
                }

                apply_positional_correction(quad_rb, circle_rb, n, penetration);
                apply_velocity_response(quad_rb, circle_rb, n);
            }
        };

    } // impl
} // Newton2D

#endif // COLLISION_RESOLUTION_HPP
