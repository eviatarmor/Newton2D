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

#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP
#pragma once

#include <memory>

#include "Structures.hpp"
#include "Vector.hpp"

namespace Newton2D {
    namespace impl {

        // Forward Declaration
        class CollisionResolution; 

        // ----------------------------------------------------------------------- //

        // This is the base class of the rigidbody.
        // it encapsulate the particle and the shape.
        class BaseRigidbody 
        {
        public:
            virtual ~BaseRigidbody() = default;

            void setPosition(VecF position)       { particle.pos   = position; }
            void setAngle(Angle angle)            { particle.angle = angle;    }
            void setLinearVelocity(VecF velocity) { particle.lvel  = velocity; }
            void setAngularVelocity(float omega)  { particle.avel  = omega;    }
            void setMass(float mass)
            {
                shape->mass = mass;
                updateInertia();
            }

            VecF  getPosition()         const { return particle.pos;   }
            Angle getAngle()            const { return particle.angle; }
            VecF  getLinearVelocity()   const { return particle.lvel;  }
            float getAngularVelocity()  const { return particle.avel;  }
            float getMass()             const { return shape->mass;    }
            ShapeKind shapeKind()       const { return shape->kind();  }
            BaseShape* getShape()             { return shape.get();    }
            const BaseShape* getShape() const { return shape.get();    }

            void accept(ShapeVisitor& visitor) { shape->accept(visitor); }
            void accept(ConstShapeVisitor& visitor) const { shape->accept(visitor); }

            float getMomentOfInertia() const { return shape->moment_of_inertia; }

        protected:
            void updateInertia()
            {
                if (!shape)
                    return;

                const float m = shape->mass;
                if (m <= 0.f)
                {
                    shape->moment_of_inertia = 0.f;
                    return;
                }

                struct Inertia : ConstShapeVisitor
                {
                    Inertia(float m, float& I) : m(m), I(I) {}

                    float m;
                    float& I;

                    void visit(const CircleShape& c) override
                    {
                        I = 0.5f * m * c.radius * c.radius;
                    }

                    void visit(const QuadShape& q) override
                    {
                        const float w = static_cast<float>(q.width);
                        const float h = static_cast<float>(q.height);
                        I = m * (w * w + h * h) / 12.f;
                    }

                    void visit(const PolygonShape& p) override
                    {
                        float acc = 0.f;
                        for (const auto& pt : p.points)
                            acc += pt.x * pt.x + pt.y * pt.y;
                        const float n = p.points.empty() ? 1.f : static_cast<float>(p.points.size());
                        I = m * acc / n;
                    }

                    void visit(const LineSegmentShape& l) override
                    {
                        const float dx = l.points[1].x - l.points[0].x;
                        const float dy = l.points[1].y - l.points[0].y;
                        I = m * (dx * dx + dy * dy) / 12.f;
                    }
                } visitor(m, shape->moment_of_inertia);

                shape->accept(visitor);
            }

            std::unique_ptr<BaseShape> shape;
            Particle particle;

            friend class CollisionResolution;
        }; // BaseRigidbody

    } // impl
    
    // ----------------------------------------------------------------------- //

    // Template Partial Specialization
    template<typename>
    class Rigidbody;

    // ----------------------------------------------------------------------- //

    template<>
    class Rigidbody<CircleShape> : public impl::BaseRigidbody
    {
    private:
        using Type = CircleShape;
        
        Type& getDerivedShape()             { return static_cast<Type&>(*shape); }
        const Type& getDerivedShape() const { return static_cast<const Type&>(*shape); }

    public:
        Rigidbody() { shape = std::make_unique<Type>(); }

        Rigidbody(float radius, float mass)
        {
            shape = std::make_unique<Type>(radius);

            setMass(mass);
        }

        Rigidbody(VecF position, float radius, float mass)
        {
            shape = std::make_unique<Type>(radius);

            setPosition(position);
            setMass(mass);
        }

        Rigidbody(VecF position, float radius, VecF velocity, float mass) 
        {
            shape = std::make_unique<Type>(radius);

            setPosition(position);
            setLinearVelocity(velocity);
            setMass(mass);
        }

        void  setRadius(float radius) { getDerivedShape().radius = radius; updateInertia(); }
        float getRadius() const       { return getDerivedShape().radius;   }

    }; // Rigidbody<Circle>  

    // ----------------------------------------------------------------------- //

    template<>
    class Rigidbody<QuadShape> : public impl::BaseRigidbody
    {
    private:
        using Type = QuadShape;
        
        Type& getDerivedShape()             { return static_cast<Type&>(*shape); }
        const Type& getDerivedShape() const { return static_cast<const Type&>(*shape); }
    
    public: 
        Rigidbody() { shape = std::make_unique<Type>(); }

        Rigidbody(unsigned int width, unsigned int height, float mass) 
        { 
            shape = std::make_unique<Type>(width, height); 

            setMass(mass);
        }

        Rigidbody(VecF position, unsigned int width, unsigned int height, float mass) 
        {
            shape = std::make_unique<Type>(width, height); 

            setPosition(position);
            setMass(mass);
        }

        Rigidbody(VecF position, unsigned int width, unsigned int height, VecF velocity, float mass) 
        {
            shape = std::make_unique<Type>(width, height); 

            setPosition(position);
            setLinearVelocity(velocity);
            setMass(mass);
        }
        
        Rigidbody(VecF position, unsigned int width, unsigned int height, Angle angle, VecF velocity, float mass) 
        {
            shape = std::make_unique<Type>(width, height); 

            setPosition(position);
            setLinearVelocity(velocity);
            setAngle(angle);
            setMass(mass);
        }

        unsigned int getWidth()  const { return getDerivedShape().width;  }
        unsigned int getHeight() const { return getDerivedShape().height; }

        void setWidth(unsigned int width)   { getDerivedShape().width = width;   updateInertia(); }
        void setHeight(unsigned int height) { getDerivedShape().height = height; updateInertia(); }
    }; // Rigidbody<Quad>

    // ----------------------------------------------------------------------- //

    template<>
    class Rigidbody<PolygonShape> : public impl::BaseRigidbody
    {
    private:
        using Type = PolygonShape;
        
        Type& getDerivedShape()             { return static_cast<Type&>(*shape); }
        const Type& getDerivedShape() const { return static_cast<const Type&>(*shape); }

    public:
        Rigidbody() { shape = std::make_unique<Type>(); }

        Rigidbody(const std::vector<VecF>& points, float mass) 
        { 
            shape = std::make_unique<Type>(points);

            setMass(mass);
        }

        Rigidbody(std::initializer_list<VecF>&& points, float mass) 
        { 
            shape = std::make_unique<Type>(std::move(points));

            setMass(mass);
        }

        Rigidbody(VecF position, const std::vector<VecF>& points, float mass) 
        { 
            shape = std::make_unique<Type>(points);

            setMass(mass);
            setPosition(position);
        }

        Rigidbody(VecF position, std::initializer_list<VecF>&& points, float mass) 
        { 
            shape = std::make_unique<Type>(std::move(points));

            setMass(mass);
            setPosition(position);
        }

        Rigidbody(VecF position, const std::vector<VecF>& points, VecF velocity, float mass) 
        { 
            shape = std::make_unique<Type>(points);

            setMass(mass);
            setPosition(position);
            setLinearVelocity(velocity);
        }

        Rigidbody(VecF position, std::initializer_list<VecF>&& points, VecF velocity, float mass) 
        { 
            shape = std::make_unique<Type>(std::move(points));

            setMass(mass);
            setPosition(position);
            setLinearVelocity(velocity);
        }

        Rigidbody(VecF position, const std::vector<VecF>& points, VecF velocity, Angle angle, float mass) 
        { 
            shape = std::make_unique<Type>(points);

            setMass(mass);
            setPosition(position);
            setLinearVelocity(velocity);
            setAngle(angle);
        }

        Rigidbody(VecF position, std::initializer_list<VecF>&& points, VecF velocity, Angle angle, float mass) 
        { 
            shape = std::make_unique<Type>(std::move(points));

            setMass(mass);
            setPosition(position);
            setLinearVelocity(velocity);
            setAngle(angle);
        }

        void setPoints(const std::vector<VecF>& points) { 
            getDerivedShape().points = points; 
        }
        void setPoints(std::initializer_list<VecF>&& points) { 
            getDerivedShape().points.assign(points.begin(), points.end()); 
        }

        const std::vector<VecF>& getPoints() const { return getDerivedShape().points; }
    }; // Rigidbody<Polygon>

    // ----------------------------------------------------------------------- //

    template<>
    class Rigidbody<LineSegmentShape> : public impl::BaseRigidbody
    {

    private:
        using Type = LineSegmentShape;
        
        Type& getDerivedShape()             { return static_cast<Type&>(*shape); }
        const Type& getDerivedShape() const { return static_cast<const Type&>(*shape); }

    public:
        Rigidbody() { shape = std::make_unique<Type>(); }

        Rigidbody(VecF point1, VecF point2, float mass) 
        { 
            shape = std::make_unique<Type>(point1, point2);

            setMass(mass);
        }

        Rigidbody(const std::array<VecF, 2>& points, float mass) 
        { 
            shape = std::make_unique<Type>(points);

            setMass(mass);
        }

        Rigidbody(VecF position, VecF point1, VecF point2, float mass) 
        { 
            shape = std::make_unique<Type>(point1, point2);

            setMass(mass);
            setPosition(position);
        }

        Rigidbody(VecF position, const std::array<VecF, 2>& points, float mass) 
        { 
            shape = std::make_unique<Type>(points);

            setMass(mass);
            setPosition(position);
        }

        Rigidbody(VecF position, VecF point1, VecF point2, VecF velocity, float mass) 
        { 
            shape = std::make_unique<Type>(point1, point2);

            setMass(mass);
            setPosition(position);
            setLinearVelocity(velocity);
        }

        Rigidbody(VecF position, const std::array<VecF, 2>& points, VecF velocity, float mass) 
        { 
            shape = std::make_unique<Type>(points);

            setMass(mass);
            setPosition(position);
            setLinearVelocity(velocity);
        }

        Rigidbody(VecF position, VecF point1, VecF point2, VecF velocity, Angle angle, float mass) 
        { 
            shape = std::make_unique<Type>(point1, point2);

            setMass(mass);
            setPosition(position);
            setLinearVelocity(velocity);
            setAngle(angle);
        }

        Rigidbody(VecF position, const std::array<VecF, 2>& points, VecF velocity, Angle angle, float mass) 
        { 
            shape = std::make_unique<Type>(points);

            setMass(mass);
            setPosition(position);
            setLinearVelocity(velocity);
            setAngle(angle);
        }
        
        void setPoints(VecF point1, VecF point2) { 
            getDerivedShape().points[0] = point1; 
            getDerivedShape().points[1] = point2;
        }

        void setPoints(const std::array<VecF, 2>& points) { 
            getDerivedShape().points = points;
        }

        const std::array<VecF, 2>& getPoints() const { 
            return getDerivedShape().points; 
        }
    }; // Rigidbody<LineSegment>

} // Newton2D

#endif // RIGIDBODY_HPP
