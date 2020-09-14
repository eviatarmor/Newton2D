#include "PhysicsEngine.hpp"
#include "Rigidbody.hpp"

int main() 
{
    Newton2D::Rigidbody<Newton2D::CircleShape> circle;

    Newton2D::PhysicsEngine engine;
    engine.push_back(circle);
}