# Newton2D

<img src="newton.jpg" alt="Newton" width="200">

This is a 2D physics engine. It doesn't need a graphics library to work — you can hook it up to whatever you draw with. I used SFML for a small sandbox so you can drop shapes and watch them fall on each other.

I never studied physics until this project. If you're a physicist, you'll probably find things that annoy you. That's fine.

## Why it's called Newton2D

I had two dogs. Flutter (as in atrial flutter — he's hyperactive) and Newton, the older one, named after Isaac Newton because of how smart he is.

Newton died in 2025. I finished this in his honour.

## The sandbox

There's a menu on the left. Pick a circle, box, triangle, or line, then click in the world to drop it. `R` or **Clear** wipes the dropped bodies. `Esc` quits.

```bash
g++ -std=c++17 -o newton2d_test test.cpp -lsfml-graphics -lsfml-window -lsfml-system
./newton2d_test
```

## Using the engine

It's headers. Include `PhysicsEngine.hpp` and `Rigidbody.hpp`, create some rigidbodies, `push_back` them into an engine, and call `loop(dt, gravity)` each frame. Mass `0` means the body is static (the ground).
