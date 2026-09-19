#include <algorithm>
#include <cmath>
#include <SFML/Graphics.hpp>

#include "CollisionResolution.hpp"
#include "PhysicsEngine.hpp"
#include "Rigidbody.hpp"
#include "SurfaceGravity.hpp"

namespace {

sf::Vector2f world_point(Newton2D::VecF local, Newton2D::VecF pos, Newton2D::Angle angle)
{
    const float c = std::cos(angle.radians());
    const float s = std::sin(angle.radians());
    return sf::Vector2f(pos.x + local.x * c - local.y * s,
                        pos.y + local.x * s + local.y * c);
}

void style_shape(sf::Shape& shape, sf::Color fill, sf::Color outline)
{
    shape.setFillColor(fill);
    shape.setOutlineColor(outline);
    shape.setOutlineThickness(2.f);
}

} // namespace

int main()
{
    constexpr unsigned int width  = 900;
    constexpr unsigned int height = 700;
    constexpr float pixels_per_meter = 80.f;

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(width, height), "Newton2D", sf::Style::Default, settings);
    window.setFramerateLimit(60);
    window.setSize(sf::Vector2u(width, height));
    window.setView(sf::View(sf::FloatRect(0.f, 0.f, static_cast<float>(width), static_cast<float>(height))));

    Newton2D::Rigidbody<Newton2D::CircleShape> circle(
        Newton2D::VecF(240.f, 50.f), 34.f, 1.f);
    Newton2D::Rigidbody<Newton2D::CircleShape> pebble(
        Newton2D::VecF(500.f, 30.f), 22.f, 0.6f);
    Newton2D::Rigidbody<Newton2D::QuadShape> box(
        Newton2D::VecF(400.f, 70.f), 70u, 48u,
        Newton2D::Angle(0.35f), Newton2D::VecF(0.f, 0.f), 2.f);
    Newton2D::Rigidbody<Newton2D::PolygonShape> triangle(
        Newton2D::VecF(620.f, 60.f),
        { Newton2D::VecF(0.f, -42.f), Newton2D::VecF(40.f, 30.f), Newton2D::VecF(-40.f, 30.f) },
        1.4f);
    Newton2D::Rigidbody<Newton2D::LineSegmentShape> ramp(
        Newton2D::VecF(200.f, 430.f),
        Newton2D::VecF(-150.f, 70.f),
        Newton2D::VecF(150.f, -70.f),
        0.f);
    Newton2D::Rigidbody<Newton2D::QuadShape> ground(
        Newton2D::VecF(width * 0.5f, height - 28.f), width, 56u, 0.f);

    Newton2D::PhysicsEngine engine;
    engine.push_back(circle);
    engine.push_back(pebble);
    engine.push_back(box);
    engine.push_back(triangle);
    engine.push_back(ramp);
    engine.push_back(ground);

    sf::CircleShape circle_draw;
    style_shape(circle_draw, sf::Color(232, 126, 74), sf::Color(40, 24, 16));
    sf::CircleShape pebble_draw;
    style_shape(pebble_draw, sf::Color(236, 201, 75), sf::Color(50, 40, 10));
    sf::RectangleShape box_draw;
    style_shape(box_draw, sf::Color(72, 175, 184), sf::Color(20, 40, 48));
    sf::ConvexShape triangle_draw;
    triangle_draw.setPointCount(3);
    style_shape(triangle_draw, sf::Color(168, 108, 203), sf::Color(40, 20, 50));
    sf::ConvexShape ramp_draw;
    ramp_draw.setPointCount(4);
    style_shape(ramp_draw, sf::Color(196, 164, 108), sf::Color(50, 36, 20));
    sf::RectangleShape ground_draw;
    style_shape(ground_draw, sf::Color(70, 78, 90), sf::Color(30, 34, 40));

    sf::Clock clock;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape)
                window.close();
            if (event.type == sf::Event::Resized)
            {
                window.setView(sf::View(sf::FloatRect(
                    0.f, 0.f, static_cast<float>(width), static_cast<float>(height))));
            }
        }

        float dt = clock.restart().asSeconds();
        dt = std::min(dt, 1.f / 30.f);
        engine.loop(dt, Newton2D::SurfaceGravity::Earth * pixels_per_meter);

        const float cr = circle.getRadius();
        circle_draw.setRadius(cr);
        circle_draw.setOrigin(cr, cr);
        circle_draw.setPosition(circle.getPosition().x, circle.getPosition().y);

        const float pr = pebble.getRadius();
        pebble_draw.setRadius(pr);
        pebble_draw.setOrigin(pr, pr);
        pebble_draw.setPosition(pebble.getPosition().x, pebble.getPosition().y);

        const float bw = static_cast<float>(box.getWidth());
        const float bh = static_cast<float>(box.getHeight());
        box_draw.setSize(sf::Vector2f(bw, bh));
        box_draw.setOrigin(bw * 0.5f, bh * 0.5f);
        box_draw.setPosition(box.getPosition().x, box.getPosition().y);
        box_draw.setRotation(box.getAngle().degrees());

        const auto& tpts = triangle.getPoints();
        for (std::size_t i = 0; i < tpts.size(); ++i)
            triangle_draw.setPoint(i, world_point(tpts[i], triangle.getPosition(), triangle.getAngle()));

        const auto& rpts = ramp.getPoints();
        const auto ra = world_point(rpts[0], ramp.getPosition(), ramp.getAngle());
        const auto rb = world_point(rpts[1], ramp.getPosition(), ramp.getAngle());
        const float rdx = rb.x - ra.x;
        const float rdy = rb.y - ra.y;
        const float rmag = std::sqrt(rdx * rdx + rdy * rdy);
        sf::Vector2f rn = rmag > 1e-6f
            ? sf::Vector2f(-rdy / rmag, rdx / rmag)
            : sf::Vector2f(0.f, -1.f);
        const float half = Newton2D::impl::CollisionResolution::line_thickness * 0.5f;
        rn.x *= half;
        rn.y *= half;
        ramp_draw.setPoint(0, ra + rn);
        ramp_draw.setPoint(1, rb + rn);
        ramp_draw.setPoint(2, rb - rn);
        ramp_draw.setPoint(3, ra - rn);

        const float gw = static_cast<float>(ground.getWidth());
        const float gh = static_cast<float>(ground.getHeight());
        ground_draw.setSize(sf::Vector2f(gw, gh));
        ground_draw.setOrigin(gw * 0.5f, gh * 0.5f);
        ground_draw.setPosition(ground.getPosition().x, ground.getPosition().y);

        window.clear(sf::Color(18, 20, 26));
        window.draw(ground_draw);
        window.draw(ramp_draw);
        window.draw(box_draw);
        window.draw(triangle_draw);
        window.draw(pebble_draw);
        window.draw(circle_draw);
        window.display();
    }
}
