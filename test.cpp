#include <algorithm>
#include <SFML/Graphics.hpp>

#include "PhysicsEngine.hpp"
#include "Rigidbody.hpp"
#include "SurfaceGravity.hpp"

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

    Newton2D::Rigidbody<Newton2D::CircleShape> upper(
        Newton2D::VecF(430.f, 60.f), 36.f, 1.f);
    Newton2D::Rigidbody<Newton2D::CircleShape> lower(
        Newton2D::VecF(470.f, 240.f), 48.f, 2.f);
    Newton2D::Rigidbody<Newton2D::QuadShape> ground(
        Newton2D::VecF(width * 0.5f, height - 30.f), width, 60u, 0.f);

    Newton2D::PhysicsEngine engine;
    engine.push_back(upper);
    engine.push_back(lower);
    engine.push_back(ground);

    sf::CircleShape upper_draw;
    upper_draw.setFillColor(sf::Color(232, 126, 74));
    upper_draw.setOutlineColor(sf::Color(40, 24, 16));
    upper_draw.setOutlineThickness(2.f);

    sf::CircleShape lower_draw;
    lower_draw.setFillColor(sf::Color(72, 175, 184));
    lower_draw.setOutlineColor(sf::Color(20, 40, 48));
    lower_draw.setOutlineThickness(2.f);

    sf::RectangleShape ground_draw;
    ground_draw.setFillColor(sf::Color(70, 78, 90));
    ground_draw.setOutlineColor(sf::Color(30, 34, 40));
    ground_draw.setOutlineThickness(2.f);

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
        }

        float dt = clock.restart().asSeconds();
        dt = std::min(dt, 1.f / 30.f);

        engine.loop(dt, Newton2D::SurfaceGravity::Earth * pixels_per_meter);

        const float ur = upper.getRadius();
        upper_draw.setRadius(ur);
        upper_draw.setOrigin(ur, ur);
        upper_draw.setPosition(upper.getPosition().x, upper.getPosition().y);

        const float lr = lower.getRadius();
        lower_draw.setRadius(lr);
        lower_draw.setOrigin(lr, lr);
        lower_draw.setPosition(lower.getPosition().x, lower.getPosition().y);

        const float gw = static_cast<float>(ground.getWidth());
        const float gh = static_cast<float>(ground.getHeight());
        ground_draw.setSize(sf::Vector2f(gw, gh));
        ground_draw.setOrigin(gw * 0.5f, gh * 0.5f);
        ground_draw.setPosition(ground.getPosition().x, ground.getPosition().y);

        window.clear(sf::Color(18, 20, 26));
        window.draw(ground_draw);
        window.draw(lower_draw);
        window.draw(upper_draw);
        window.display();
    }
}
