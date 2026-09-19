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
#include <limits>
#include <vector>

#include "Core.hpp"
#include "Rigidbody.hpp"
#include "Structures.hpp"

namespace Newton2D {
    namespace impl {

        class CollisionResolution
        {
        public:
            static constexpr float restitution        = 0.35f;
            static constexpr float correction_percent = 0.8f;
            static constexpr float slop               = 0.05f;
            static constexpr float rest_threshold     = 25.f;
            static constexpr float friction           = 0.55f;
            static constexpr float line_thickness     = 8.f;

            struct Manifold
            {
                bool  hit = false;
                VecF  n{};
                float pen = 0.f;
                VecF  contact{};
            };

            static Manifold compute(BaseRigidbody& a, BaseRigidbody& b)
            {
                Manifold m;

                struct AgainstCircle : ShapeVisitor
                {
                    AgainstCircle(BaseRigidbody& a_rb, CircleShape& ca,
                                  BaseRigidbody& b_rb, Manifold& out)
                        : a_rb(a_rb), ca(ca), b_rb(b_rb), out(out) {}

                    BaseRigidbody& a_rb;
                    CircleShape& ca;
                    BaseRigidbody& b_rb;
                    Manifold& out;

                    void visit(CircleShape& cb) override
                    {
                        out = circle_circle(a_rb, ca, b_rb, cb);
                    }

                    void visit(QuadShape&) override { vs_poly(); }
                    void visit(PolygonShape&) override { vs_poly(); }
                    void visit(LineSegmentShape&) override { vs_poly(); }

                    void vs_poly()
                    {
                        out = circle_poly(a_rb, ca, b_rb);
                        out.n.x = -out.n.x;
                        out.n.y = -out.n.y;
                    }
                };

                struct AgainstPoly : ShapeVisitor
                {
                    AgainstPoly(BaseRigidbody& a_rb, BaseRigidbody& b_rb, Manifold& out)
                        : a_rb(a_rb), b_rb(b_rb), out(out) {}

                    BaseRigidbody& a_rb;
                    BaseRigidbody& b_rb;
                    Manifold& out;

                    void visit(CircleShape& cb) override
                    {
                        out = circle_poly(b_rb, cb, a_rb);
                    }

                    void visit(QuadShape&) override { out = poly_poly(a_rb, b_rb); }
                    void visit(PolygonShape&) override { out = poly_poly(a_rb, b_rb); }
                    void visit(LineSegmentShape&) override { out = poly_poly(a_rb, b_rb); }
                };

                struct First : ShapeVisitor
                {
                    First(BaseRigidbody& a_rb, BaseRigidbody& b_rb, Manifold& out)
                        : a_rb(a_rb), b_rb(b_rb), out(out) {}

                    BaseRigidbody& a_rb;
                    BaseRigidbody& b_rb;
                    Manifold& out;

                    void visit(CircleShape& ca) override
                    {
                        AgainstCircle second(a_rb, ca, b_rb, out);
                        b_rb.accept(second);
                    }

                    void visit(QuadShape&) override { vs_poly(); }
                    void visit(PolygonShape&) override { vs_poly(); }
                    void visit(LineSegmentShape&) override { vs_poly(); }

                    void vs_poly()
                    {
                        AgainstPoly second(a_rb, b_rb, out);
                        b_rb.accept(second);
                    }
                } first(a, b, m);

                a.accept(first);
                return m;
            }

            static void separate(BaseRigidbody& a, BaseRigidbody& b)
            {
                const Manifold m = compute(a, b);
                if (m.hit)
                    apply_positional_correction(a, b, m.n, m.pen);
            }

            static void bounce(BaseRigidbody& a, BaseRigidbody& b)
            {
                const Manifold m = compute(a, b);
                if (m.hit)
                    apply_velocity_response(a, b, m.n, m.contact);
            }

            static void resolve(BaseRigidbody& a, BaseRigidbody& b)
            {
                separate(a, b);
                bounce(a, b);
            }

        private:
            static float inv_mass(const BaseRigidbody& rb)
            {
                const float m = rb.getMass();
                return m > 0.f ? 1.f / m : 0.f;
            }

            static float inv_inertia(const BaseRigidbody& rb)
            {
                if (rb.getMass() <= 0.f)
                    return 0.f;
                const float I = rb.getMomentOfInertia();
                return I > 1e-8f ? 1.f / I : 0.f;
            }

            static float cross(const VecF& a, const VecF& b)
            {
                return a.x * b.y - a.y * b.x;
            }

            static VecF cross(float s, const VecF& v)
            {
                return VecF(-s * v.y, s * v.x);
            }

            static VecF support(const std::vector<VecF>& verts, const VecF& dir)
            {
                std::size_t best = 0;
                float best_d = verts[0].x * dir.x + verts[0].y * dir.y;
                for (std::size_t i = 1; i < verts.size(); ++i)
                {
                    const float d = verts[i].x * dir.x + verts[i].y * dir.y;
                    if (d > best_d)
                    {
                        best_d = d;
                        best = i;
                    }
                }
                return verts[best];
            }

            static VecF rotate_local(const VecF& local, const VecF& pos, float c, float s)
            {
                return VecF(pos.x + local.x * c - local.y * s,
                            pos.y + local.x * s + local.y * c);
            }

            static std::vector<VecF> world_vertices(const BaseRigidbody& rb)
            {
                const VecF pos = rb.getPosition();
                const float rad = rb.getAngle().radians();
                const float c = cosf(rad);
                const float s = sinf(rad);
                std::vector<VecF> verts;

                struct Vertices : ConstShapeVisitor
                {
                    Vertices(const VecF& pos, float c, float s, std::vector<VecF>& verts)
                        : pos(pos), c(c), s(s), verts(verts) {}

                    const VecF& pos;
                    float c;
                    float s;
                    std::vector<VecF>& verts;

                    void visit(const CircleShape&) override {}

                    void visit(const QuadShape& q) override
                    {
                        const float hw = static_cast<float>(q.width)  * 0.5f;
                        const float hh = static_cast<float>(q.height) * 0.5f;
                        verts = {
                            rotate_local(VecF(-hw, -hh), pos, c, s),
                            rotate_local(VecF( hw, -hh), pos, c, s),
                            rotate_local(VecF( hw,  hh), pos, c, s),
                            rotate_local(VecF(-hw,  hh), pos, c, s)
                        };
                    }

                    void visit(const PolygonShape& p) override
                    {
                        verts.reserve(p.points.size());
                        for (const auto& pt : p.points)
                            verts.push_back(rotate_local(pt, pos, c, s));
                    }

                    void visit(const LineSegmentShape& l) override
                    {
                        const VecF a = rotate_local(l.points[0], pos, c, s);
                        const VecF b = rotate_local(l.points[1], pos, c, s);
                        const float dx = b.x - a.x;
                        const float dy = b.y - a.y;
                        const float mag = sqrtf(dx * dx + dy * dy);
                        VecF n = mag > 1e-6f ? VecF(-dy / mag, dx / mag) : VecF(0.f, -1.f);
                        n.x *= line_thickness * 0.5f;
                        n.y *= line_thickness * 0.5f;
                        verts = {
                            VecF(a.x + n.x, a.y + n.y),
                            VecF(b.x + n.x, b.y + n.y),
                            VecF(b.x - n.x, b.y - n.y),
                            VecF(a.x - n.x, a.y - n.y)
                        };
                    }
                } visitor(pos, c, s, verts);

                rb.accept(visitor);
                return verts;
            }

            static VecF centroid(const std::vector<VecF>& verts)
            {
                VecF c{};
                if (verts.empty())
                    return c;
                for (const auto& v : verts)
                {
                    c.x += v.x;
                    c.y += v.y;
                }
                const float inv = 1.f / static_cast<float>(verts.size());
                c.x *= inv;
                c.y *= inv;
                return c;
            }

            static void project(const std::vector<VecF>& verts, const VecF& axis, float& mn, float& mx)
            {
                mn = mx = verts[0].x * axis.x + verts[0].y * axis.y;
                for (std::size_t i = 1; i < verts.size(); ++i)
                {
                    const float p = verts[i].x * axis.x + verts[i].y * axis.y;
                    mn = std::min(mn, p);
                    mx = std::max(mx, p);
                }
            }

            static bool overlap_axis(const std::vector<VecF>& a, const std::vector<VecF>& b,
                                     VecF axis, float& min_overlap, VecF& best)
            {
                const float mag = sqrtf(axis.x * axis.x + axis.y * axis.y);
                if (mag < 1e-8f)
                    return true;
                axis.x /= mag;
                axis.y /= mag;

                float amin, amax, bmin, bmax;
                project(a, axis, amin, amax);
                project(b, axis, bmin, bmax);
                const float o = std::min(amax, bmax) - std::max(amin, bmin);
                if (o <= 0.f)
                    return false;
                if (o < min_overlap)
                {
                    min_overlap = o;
                    best = axis;
                }
                return true;
            }

            static bool sat(const std::vector<VecF>& a, const std::vector<VecF>& b, VecF& n, float& pen)
            {
                if (a.size() < 2 || b.size() < 2)
                    return false;

                float min_overlap = std::numeric_limits<float>::infinity();
                VecF best{};

                auto test_edges = [&](const std::vector<VecF>& verts) {
                    for (std::size_t i = 0; i < verts.size(); ++i)
                    {
                        const VecF& p = verts[i];
                        const VecF& q = verts[(i + 1) % verts.size()];
                        if (!overlap_axis(a, b, VecF(-(q.y - p.y), q.x - p.x), min_overlap, best))
                            return false;
                    }
                    return true;
                };

                if (!test_edges(a) || !test_edges(b))
                    return false;

                const VecF ca = centroid(a);
                const VecF cb = centroid(b);
                if ((cb.x - ca.x) * best.x + (cb.y - ca.y) * best.y < 0.f)
                {
                    best.x = -best.x;
                    best.y = -best.y;
                }

                n = best;
                pen = min_overlap;
                return true;
            }

            static bool circle_vs_poly(const VecF& center, float radius,
                                       const std::vector<VecF>& poly, VecF& n, float& pen)
            {
                if (poly.size() < 2)
                    return false;

                float min_overlap = std::numeric_limits<float>::infinity();
                VecF best{};

                auto project_circle = [&](const VecF& axis, float& mn, float& mx) {
                    const float mid = center.x * axis.x + center.y * axis.y;
                    mn = mid - radius;
                    mx = mid + radius;
                };

                auto overlap = [&](VecF axis) {
                    const float mag = sqrtf(axis.x * axis.x + axis.y * axis.y);
                    if (mag < 1e-8f)
                        return true;
                    axis.x /= mag;
                    axis.y /= mag;

                    float pmin, pmax, cmin, cmax;
                    project(poly, axis, pmin, pmax);
                    project_circle(axis, cmin, cmax);
                    const float o = std::min(pmax, cmax) - std::max(pmin, cmin);
                    if (o <= 0.f)
                        return false;
                    if (o < min_overlap)
                    {
                        min_overlap = o;
                        best = axis;
                    }
                    return true;
                };

                for (std::size_t i = 0; i < poly.size(); ++i)
                {
                    const VecF& p = poly[i];
                    const VecF& q = poly[(i + 1) % poly.size()];
                    if (!overlap(VecF(-(q.y - p.y), q.x - p.x)))
                        return false;
                }

                std::size_t closest = 0;
                float best_d2 = std::numeric_limits<float>::infinity();
                for (std::size_t i = 0; i < poly.size(); ++i)
                {
                    const float dx = center.x - poly[i].x;
                    const float dy = center.y - poly[i].y;
                    const float d2 = dx * dx + dy * dy;
                    if (d2 < best_d2)
                    {
                        best_d2 = d2;
                        closest = i;
                    }
                }
                if (!overlap(VecF(center.x - poly[closest].x, center.y - poly[closest].y)))
                    return false;

                const VecF pc = centroid(poly);
                if ((center.x - pc.x) * best.x + (center.y - pc.y) * best.y < 0.f)
                {
                    best.x = -best.x;
                    best.y = -best.y;
                }

                n = best;
                pen = min_overlap;
                return true;
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

            static void apply_velocity_response(BaseRigidbody& a, BaseRigidbody& b,
                                                const VecF& n, const VecF&)
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

            static Manifold circle_circle(BaseRigidbody& a, const CircleShape& sa,
                                          BaseRigidbody& b, const CircleShape& sb)
            {
                Manifold m;
                const VecF pa = a.getPosition();
                const VecF pb = b.getPosition();
                const float dx = pb.x - pa.x;
                const float dy = pb.y - pa.y;
                const float dist = sqrtf(dx * dx + dy * dy);
                const float min_dist = sa.radius + sb.radius;

                if (dist >= min_dist)
                    return m;

                if (dist > 1e-6f)
                {
                    m.n.x = dx / dist;
                    m.n.y = dy / dist;
                }
                else
                {
                    m.n.x = 0.f;
                    m.n.y = -1.f;
                }

                m.hit = true;
                m.pen = min_dist - dist;
                m.contact = VecF(pa.x + m.n.x * sa.radius, pa.y + m.n.y * sa.radius);
                return m;
            }

            static Manifold circle_poly(BaseRigidbody& circle_rb, const CircleShape& circle,
                                        BaseRigidbody& poly_rb)
            {
                Manifold m;
                const auto verts = world_vertices(poly_rb);
                if (!circle_vs_poly(circle_rb.getPosition(), circle.radius, verts, m.n, m.pen))
                    return m;

                const VecF c = circle_rb.getPosition();
                m.hit = true;
                m.contact = VecF(c.x - m.n.x * circle.radius, c.y - m.n.y * circle.radius);
                return m;
            }

            static Manifold poly_poly(BaseRigidbody& a, BaseRigidbody& b)
            {
                Manifold m;
                const auto va = world_vertices(a);
                const auto vb = world_vertices(b);
                if (!sat(va, vb, m.n, m.pen))
                    return m;

                const VecF sa = support(va, m.n);
                const VecF sb = support(vb, VecF(-m.n.x, -m.n.y));
                m.hit = true;
                if (inv_mass(b) == 0.f && inv_mass(a) > 0.f)
                    m.contact = sa;
                else if (inv_mass(a) == 0.f && inv_mass(b) > 0.f)
                    m.contact = sb;
                else
                    m.contact = VecF(0.5f * (sa.x + sb.x), 0.5f * (sa.y + sb.y));
                return m;
            }
        };

    } // impl
} // Newton2D

#endif // COLLISION_RESOLUTION_HPP
