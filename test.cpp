#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <SFML/Graphics.hpp>

#include "CollisionResolution.hpp"
#include "PhysicsEngine.hpp"
#include "Rigidbody.hpp"
#include "SurfaceGravity.hpp"

namespace {

constexpr float kSidebar   = 220.f;
constexpr float kWorldW    = 900.f;
constexpr float kWorldH    = 700.f;
constexpr float kWindowW   = kSidebar + kWorldW;
constexpr float kWindowH   = kWorldH;
constexpr float kPixelsPerMeter = 80.f;

enum class Tool { Circle, Box, Triangle, Line };

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

sf::Color with_alpha(sf::Color c, sf::Uint8 a)
{
    c.a = a;
    return c;
}

void draw_line_body(sf::RenderWindow& window,
                    const Newton2D::impl::BaseRigidbody& rb,
                    const Newton2D::LineSegmentShape& line,
                    sf::Color fill, sf::Color outline)
{
    const auto& pts = line.points;
    const auto a = world_point(pts[0], rb.getPosition(), rb.getAngle());
    const auto b = world_point(pts[1], rb.getPosition(), rb.getAngle());
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float mag = std::sqrt(dx * dx + dy * dy);
    sf::Vector2f n = mag > 1e-6f
        ? sf::Vector2f(-dy / mag, dx / mag)
        : sf::Vector2f(0.f, -1.f);
    const float half = Newton2D::impl::CollisionResolution::line_thickness * 0.5f;
    n *= half;

    sf::ConvexShape draw;
    draw.setPointCount(4);
    style_shape(draw, fill, outline);
    draw.setPoint(0, a + n);
    draw.setPoint(1, b + n);
    draw.setPoint(2, b - n);
    draw.setPoint(3, a - n);
    window.draw(draw);
}

void draw_body(sf::RenderWindow& window, Newton2D::impl::BaseRigidbody& rb, bool ghost)
{
    const sf::Uint8 alpha = ghost ? 90 : 255;
    const auto pos = rb.getPosition();

    if (auto* circle = Newton2D::as_shape<Newton2D::CircleShape>(rb.getShape()))
    {
        const float r = circle->radius;
        sf::CircleShape draw;
        style_shape(draw,
                    with_alpha(sf::Color(232, 126, 74), alpha),
                    with_alpha(sf::Color(40, 24, 16), alpha));
        draw.setRadius(r);
        draw.setOrigin(r, r);
        draw.setPosition(pos.x, pos.y);
        window.draw(draw);
        return;
    }

    if (auto* box = Newton2D::as_shape<Newton2D::QuadShape>(rb.getShape()))
    {
        const float w = static_cast<float>(box->width);
        const float h = static_cast<float>(box->height);
        sf::RectangleShape draw;
        style_shape(draw,
                    with_alpha(sf::Color(72, 175, 184), alpha),
                    with_alpha(sf::Color(20, 40, 48), alpha));
        draw.setSize(sf::Vector2f(w, h));
        draw.setOrigin(w * 0.5f, h * 0.5f);
        draw.setPosition(pos.x, pos.y);
        draw.setRotation(rb.getAngle().degrees());
        window.draw(draw);
        return;
    }

    if (auto* tri = Newton2D::as_shape<Newton2D::PolygonShape>(rb.getShape()))
    {
        const auto& pts = tri->points;
        sf::ConvexShape draw;
        draw.setPointCount(static_cast<unsigned int>(pts.size()));
        style_shape(draw,
                    with_alpha(sf::Color(168, 108, 203), alpha),
                    with_alpha(sf::Color(40, 20, 50), alpha));
        for (std::size_t i = 0; i < pts.size(); ++i)
            draw.setPoint(i, world_point(pts[i], pos, rb.getAngle()));
        window.draw(draw);
        return;
    }

    if (auto* line = Newton2D::as_shape<Newton2D::LineSegmentShape>(rb.getShape()))
    {
        draw_line_body(window, rb, *line,
                       with_alpha(sf::Color(196, 164, 108), alpha),
                       with_alpha(sf::Color(50, 36, 20), alpha));
    }
}

std::unique_ptr<Newton2D::impl::BaseRigidbody> make_shape(Tool tool, Newton2D::VecF pos)
{
    using namespace Newton2D;
    switch (tool)
    {
    case Tool::Circle:
        return std::make_unique<Rigidbody<CircleShape>>(pos, 28.f, 1.f);
    case Tool::Box:
        return std::make_unique<Rigidbody<QuadShape>>(pos, 64u, 44u, 2.f);
    case Tool::Triangle:
        return std::make_unique<Rigidbody<PolygonShape>>(
            pos,
            std::initializer_list<VecF>{ VecF(0.f, -36.f), VecF(34.f, 26.f), VecF(-34.f, 26.f) },
            1.4f);
    case Tool::Line:
        return std::make_unique<Rigidbody<LineSegmentShape>>(
            pos, VecF(-55.f, 0.f), VecF(55.f, 0.f), 1.f);
    }
    return nullptr;
}

struct Button
{
    sf::FloatRect rect;
    Tool tool;
    const char* label;
};

} // namespace

int main()
{
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(
        sf::VideoMode(static_cast<unsigned int>(kWindowW), static_cast<unsigned int>(kWindowH)),
        "Newton2D", sf::Style::Default, settings);
    window.setFramerateLimit(60);
    window.setSize(sf::Vector2u(static_cast<unsigned int>(kWindowW),
                                static_cast<unsigned int>(kWindowH)));
    window.setView(sf::View(sf::FloatRect(0.f, 0.f, kWindowW, kWindowH)));

    sf::Font font;
    const bool have_font =
        font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
        || font.loadFromFile("/mnt/c/Windows/Fonts/segoeui.ttf")
        || font.loadFromFile("/mnt/c/Windows/Fonts/arial.ttf");

    Newton2D::Rigidbody<Newton2D::QuadShape> ground(
        Newton2D::VecF(kSidebar + kWorldW * 0.5f, kWindowH - 28.f),
        static_cast<unsigned int>(kWorldW), 56u, 0.f);

    std::vector<std::unique_ptr<Newton2D::impl::BaseRigidbody>> dropped;
    Newton2D::PhysicsEngine engine;
    engine.push_back(ground);

    auto rebuild = [&] {
        engine.clear();
        engine.push_back(ground);
        for (auto& body : dropped)
            engine.push_back(*body);
    };

    const Button buttons[] = {
        { sf::FloatRect(16.f,  86.f, 188.f, 70.f), Tool::Circle,   "Circle"   },
        { sf::FloatRect(16.f, 166.f, 188.f, 70.f), Tool::Box,      "Box"      },
        { sf::FloatRect(16.f, 246.f, 188.f, 70.f), Tool::Triangle, "Triangle" },
        { sf::FloatRect(16.f, 326.f, 188.f, 70.f), Tool::Line,     "Line"     },
    };
    const sf::FloatRect clear_rect(16.f, 430.f, 188.f, 48.f);

    Tool selected = Tool::Circle;

    auto label = [&](const std::string& str, unsigned size, sf::Color color) {
        sf::Text text;
        if (have_font)
        {
            text.setFont(font);
            text.setString(str);
            text.setCharacterSize(size);
            text.setFillColor(color);
        }
        return text;
    };

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
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R)
            {
                dropped.clear();
                rebuild();
            }
            if (event.type == sf::Event::Resized)
            {
                window.setView(sf::View(sf::FloatRect(0.f, 0.f, kWindowW, kWindowH)));
            }
            if (event.type == sf::Event::MouseButtonPressed
                && event.mouseButton.button == sf::Mouse::Left)
            {
                const sf::Vector2f mouse = window.mapPixelToCoords(
                    sf::Vector2i(event.mouseButton.x, event.mouseButton.y));

                bool hit_ui = false;
                for (const auto& button : buttons)
                {
                    if (button.rect.contains(mouse))
                    {
                        selected = button.tool;
                        hit_ui = true;
                        break;
                    }
                }
                if (!hit_ui && clear_rect.contains(mouse))
                {
                    dropped.clear();
                    rebuild();
                    hit_ui = true;
                }
                if (!hit_ui && mouse.x >= kSidebar && dropped.size() < 80)
                {
                    auto body = make_shape(selected, Newton2D::VecF(mouse.x, mouse.y));
                    engine.push_back(*body);
                    dropped.push_back(std::move(body));
                }
            }
        }

        float dt = clock.restart().asSeconds();
        dt = std::min(dt, 1.f / 30.f);
        engine.loop(dt, Newton2D::SurfaceGravity::Earth * kPixelsPerMeter);

        window.clear(sf::Color(18, 20, 26));

        sf::RectangleShape sidebar;
        sidebar.setSize(sf::Vector2f(kSidebar, kWindowH));
        sidebar.setFillColor(sf::Color(28, 31, 40));
        window.draw(sidebar);

        sf::RectangleShape divider;
        divider.setSize(sf::Vector2f(2.f, kWindowH));
        divider.setPosition(kSidebar - 2.f, 0.f);
        divider.setFillColor(sf::Color(48, 54, 68));
        window.draw(divider);

        if (have_font)
        {
            sf::Text title = label("Newton2D", 22, sf::Color(230, 232, 240));
            title.setPosition(16.f, 18.f);
            window.draw(title);

            sf::Text hint = label("Choose a shape,\nthen click to drop.", 14, sf::Color(160, 168, 180));
            hint.setPosition(16.f, 50.f);
            window.draw(hint);
        }

        for (const auto& button : buttons)
        {
            sf::RectangleShape bg;
            bg.setPosition(button.rect.left, button.rect.top);
            bg.setSize(sf::Vector2f(button.rect.width, button.rect.height));
            const bool on = button.tool == selected;
            bg.setFillColor(on ? sf::Color(48, 58, 78) : sf::Color(36, 40, 52));
            bg.setOutlineThickness(2.f);
            bg.setOutlineColor(on ? sf::Color(232, 196, 96) : sf::Color(58, 64, 80));
            window.draw(bg);

            const sf::Vector2f icon_at(button.rect.left + 36.f,
                                       button.rect.top + button.rect.height * 0.5f);
            if (button.tool == Tool::Circle)
            {
                sf::CircleShape icon(14.f);
                style_shape(icon, sf::Color(232, 126, 74), sf::Color(40, 24, 16));
                icon.setOrigin(14.f, 14.f);
                icon.setPosition(icon_at);
                window.draw(icon);
            }
            else if (button.tool == Tool::Box)
            {
                sf::RectangleShape icon(sf::Vector2f(28.f, 20.f));
                style_shape(icon, sf::Color(72, 175, 184), sf::Color(20, 40, 48));
                icon.setOrigin(14.f, 10.f);
                icon.setPosition(icon_at);
                window.draw(icon);
            }
            else if (button.tool == Tool::Triangle)
            {
                sf::ConvexShape icon;
                icon.setPointCount(3);
                style_shape(icon, sf::Color(168, 108, 203), sf::Color(40, 20, 50));
                icon.setPoint(0, icon_at + sf::Vector2f(0.f, -16.f));
                icon.setPoint(1, icon_at + sf::Vector2f(16.f, 12.f));
                icon.setPoint(2, icon_at + sf::Vector2f(-16.f, 12.f));
                window.draw(icon);
            }
            else
            {
                sf::RectangleShape icon(sf::Vector2f(32.f, 6.f));
                style_shape(icon, sf::Color(196, 164, 108), sf::Color(50, 36, 20));
                icon.setOrigin(16.f, 3.f);
                icon.setPosition(icon_at);
                window.draw(icon);
            }

            if (have_font)
            {
                sf::Text text = label(button.label, 16, sf::Color(220, 224, 232));
                text.setPosition(button.rect.left + 72.f, button.rect.top + 24.f);
                window.draw(text);
            }
        }

        sf::RectangleShape clear_bg;
        clear_bg.setPosition(clear_rect.left, clear_rect.top);
        clear_bg.setSize(sf::Vector2f(clear_rect.width, clear_rect.height));
        clear_bg.setFillColor(sf::Color(70, 42, 48));
        clear_bg.setOutlineThickness(2.f);
        clear_bg.setOutlineColor(sf::Color(120, 70, 76));
        window.draw(clear_bg);
        if (have_font)
        {
            sf::Text clear = label("Clear  (R)", 16, sf::Color(236, 210, 210));
            clear.setPosition(clear_rect.left + 48.f, clear_rect.top + 12.f);
            window.draw(clear);

            sf::Text help = label("Esc  quit", 13, sf::Color(120, 128, 140));
            help.setPosition(16.f, kWindowH - 36.f);
            window.draw(help);
        }

        sf::RectangleShape ground_draw;
        const float gw = static_cast<float>(ground.getWidth());
        const float gh = static_cast<float>(ground.getHeight());
        style_shape(ground_draw, sf::Color(70, 78, 90), sf::Color(30, 34, 40));
        ground_draw.setSize(sf::Vector2f(gw, gh));
        ground_draw.setOrigin(gw * 0.5f, gh * 0.5f);
        ground_draw.setPosition(ground.getPosition().x, ground.getPosition().y);
        window.draw(ground_draw);

        for (auto& body : dropped)
            draw_body(window, *body, false);

        const sf::Vector2f mouse = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        if (mouse.x >= kSidebar && mouse.x <= kWindowW && mouse.y >= 0.f && mouse.y <= kWindowH)
        {
            auto ghost = make_shape(selected, Newton2D::VecF(mouse.x, mouse.y));
            draw_body(window, *ghost, true);
        }

        window.display();
    }
}
