# Game Physics in a Weekend (in Rust)

This project is an implementation of the [Game Physics in a Weekend] book using
the [Rust] programming language and the [Bevy] game engine.

This has been a learning excercise for me to get a better understanding of
implementing a physics engine, using Bevy and also making use of my math library
[glam].

So far only the first book has been implemented.

I'm using a fork of Bevy 0.4.0 that is using the latest version of glam from
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

## License

* [Creative Commons Zero v1.0 Universal]

[Game Physics in a Weekend]: https://gamephysicsweekend.github.io
[Rust]: https://www.rust-lang.org
[Bevy]: https://bevyengine.org
[glam]: https://github.com/bitshifter/glam
[Bevy Flycam]: https://github.com/sburris0/bevy_flycam
[Creative Commons Zero v1.0 Universal]: LICENSE
