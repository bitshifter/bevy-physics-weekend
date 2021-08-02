# Game Physics in a Weekend (in Rust)

This project is an implementation of the [Game Physics in a Weekend] book using
the [Rust] programming language and the [Bevy] game engine.

This has been a learning excercise for me to get a better understanding of
implementing a physics engine, using Bevy and also making use of my math library
[glam].

Note that the code from the book and this code is for learning purposes, it's
not very optimized and not intended for use in production. For real world use
check out [Rapier] or [physx-rs].

I'm using a fork of Bevy 0.5.0 that is using the latest version of glam from
github.

## Simulation controls

* T - toggles pausing the simulation
* Y - step the simulation while paused
* R - reset the simulation

## Camera controls

The [Bevy Flycam] plugin is used for camera movement.

* WASD - move backwards, forwards, left and right
* Space - move up
* Left shift - move down
* Escape - toggle mouse cursor lock

## Differences from the book

### Coordinate system

The book uses Z up and Bevy uses Y up. I've used the Bevy Y up convention here.

### Matrices

The book uses row-major matrices whereas `glam` uses column vectors and column
major matrices.

### Architecture

I've mostly tried to follow the code structure in the book, however some things
don't translate so well to Rust so some organisation of code has changed.

* `BodyHandle` is used instead of `Body*` pointers. The handle is currently
  just wrapping an array index.
* `Body` and `Constraint` structs are owned by `BodyArena` and
  `ConstraintArena` instead of being owned by `PhysicsScene`. This makes
  working with the borrow checker easier.

## License

[Creative Commons Zero v1.0 Universal]

[Game Physics in a Weekend]: https://gamephysicsweekend.github.io
[Rust]: https://www.rust-lang.org
[Bevy]: https://bevyengine.org
[glam]: https://github.com/bitshifter/glam
[Bevy Flycam]: https://github.com/sburris0/bevy_flycam
[Rapier]: https://rapier.rs
[physx-rs]: https://github.com/EmbarkStudios/physx-rs
[Creative Commons Zero v1.0 Universal]: LICENSE
