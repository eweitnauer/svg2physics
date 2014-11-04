JS_COMPILER = ./node_modules/.bin/uglifyjs

all: svg2physics.min.js svg2physics.zip

.INTERMEDIATE svg2physics.js: \
	src/start.js \
	src/box2d-adapter.js \
	src/box2d-extensions.js \
	src/box2d-simulator.js \
	src/physics-scene.js \
	src/svg-scene-parser.js \
	src/physics-oracle.js \
	src/end.js

svg2physics.min.js: svg2physics.js Makefile
	@rm -f $@
	$(JS_COMPILER) -m --preamble '// Copyright Erik Weitnauer 2014.' < $< > $@
	@chmod a-w $@

svg2physics.js: Makefile
	@rm -f $@
	cat $(filter %.js,$^) > $@
	@chmod a-w $@

svg2physics.zip: svg2physics.min.js
	zip svg2physics.zip \
	  svg2physics.min.js svg2physics.js LICENCE \
	  libs/box2dweb/Box2D.min.js libs/box2dweb/Box2D.js

clean:
	rm -f svg2physics*.js

.PHONY: all clean
