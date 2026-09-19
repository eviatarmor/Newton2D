#include "PhysicsEngine.hpp"
#include "Rigidbody.hpp"

#include <cmath>
#include <iostream>

static int fails = 0;

static void expect(bool cond, const char* msg)
{
    if (!cond)
    {
        std::cerr << "FAIL: " << msg << std::endl;
        ++fails;
    }
    else
    {
        std::cout << "ok: " << msg << std::endl;
    }
}

int main()
{
    Newton2D::Rigidbody<Newton2D::CircleShape> a(Newton2D::VecF(0.f, 0.f), 10.f, 2.5f);
    expect(std::fabs(a.getMass() - 2.5f) < 1e-6f, "circle mass is stored");
    expect(std::fabs(a.getRadius() - 10.f) < 1e-6f, "circle radius is stored");

    Newton2D::Rigidbody<Newton2D::QuadShape> q(Newton2D::VecF(0.f, 100.f), 40u, 20u, 3.f);
    expect(std::fabs(q.getMass() - 3.f) < 1e-6f, "quad mass is stored");

    Newton2D::Rigidbody<Newton2D::CircleShape> c1(Newton2D::VecF(0.f, 0.f), 10.f, 1.f);
    Newton2D::Rigidbody<Newton2D::CircleShape> c2(Newton2D::VecF(5.f, 0.f), 10.f, 1.f);
    Newton2D::PhysicsEngine overlap;
    overlap.push_back(c1);
    overlap.push_back(c2);
    overlap.loop(0.f, 0.f);
    const float dx = c2.getPosition().x - c1.getPosition().x;
    const float dy = c2.getPosition().y - c1.getPosition().y;
    const float dist = std::sqrt(dx * dx + dy * dy);
    expect(dist + Newton2D::impl::CollisionResolution::slop + 1e-3f >= 20.f,
           "overlapping circles are separated");

    Newton2D::Rigidbody<Newton2D::CircleShape> falling(Newton2D::VecF(0.f, 0.f), 10.f, 1.f);
    Newton2D::PhysicsEngine grav;
    grav.push_back(falling);
    grav.loop(0.1f, 10.f);
    expect(falling.getLinearVelocity().y > 0.f, "gravity increases downward velocity");
    expect(falling.getPosition().y > 0.f, "gravity moves body downward");

    Newton2D::Rigidbody<Newton2D::CircleShape> ball(Newton2D::VecF(0.f, 0.f), 10.f, 1.f);
    Newton2D::Rigidbody<Newton2D::QuadShape> ground(Newton2D::VecF(0.f, 40.f), 200u, 20u, 0.f);
    Newton2D::PhysicsEngine land;
    land.push_back(ball);
    land.push_back(ground);
    for (int i = 0; i < 180; ++i)
        land.loop(1.f / 60.f, 500.f);
    const float gy = ground.getPosition().y - 10.f;
    expect(ball.getPosition().y + ball.getRadius() <= gy + 2.f, "circle rests on static ground");
    expect(std::fabs(ground.getPosition().y - 40.f) < 1e-4f, "static ground does not move");

    Newton2D::Rigidbody<Newton2D::CircleShape> top(Newton2D::VecF(0.f, -80.f), 12.f, 1.f);
    Newton2D::Rigidbody<Newton2D::CircleShape> bot(Newton2D::VecF(0.f, -20.f), 12.f, 1.f);
    Newton2D::Rigidbody<Newton2D::QuadShape> floor(Newton2D::VecF(0.f, 30.f), 400u, 20u, 0.f);
    Newton2D::PhysicsEngine stack;
    stack.push_back(top);
    stack.push_back(bot);
    stack.push_back(floor);
    for (int i = 0; i < 300; ++i)
        stack.loop(1.f / 60.f, 400.f);
    expect(top.getPosition().y < bot.getPosition().y, "upper circle stays above lower circle");
    const float gap = bot.getPosition().y - top.getPosition().y;
    expect(gap + 0.5f >= top.getRadius() + bot.getRadius(), "stacked circles do not sink through each other");

    if (fails)
    {
        std::cerr << fails << " failed\n";
        return 1;
    }
    std::cout << "all checks passed\n";
    return 0;
}
