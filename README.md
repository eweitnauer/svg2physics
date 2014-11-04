# svg2physics

Construct Box2D physics scenes from SVG images.

To run the example, you will most likely have to start a local server in the examples directory. On many systems, `python -m SimpleHTTPServer` will work. Another option is the `static` node package which you can install using `npm`.

### How to create SVG images

In principle, any drawing application capable of working with SVG images should be fine to create the source images for the physical scenes. So far, the library was only tested with images created with [Inkscape](https://inkscape.org/en/), though.

There are a couple of things you need to pay attention to so that the resulting files can be parsed by this library:

1. The biggest rectangle in the scene will be interpreted as the viewport of the physics scene. Its lower left corner will be positioned at (0, 0) and all other objects will be positioned relative to it.
2. Use black stroke color for all static objects.
3. Use any stroke color except black for dynamic, movable objects.
