JS_COMPILER = ./node_modules/.bin/uglifyjs

all: svg2physics.min.js

.INTERMEDIATE svg2physics.js: \
	src/start.js \
	src/box2d-adapter.js \
	src/box2d-extensions.js \
	src/box2d-simulator.js \
	src/physics-scene.js \
	src/svg-scene-parser.js \
	src/end.js

svg2physics.min.js: svg2physics.js Makefile
	@rm -f $@
	$(JS_COMPILER) -m --preamble '// Copyright Erik Weitnauer 2014.' < $< > $@
	@chmod a-w $@

svg2physics.js: Makefile
	@rm -f $@
	cat $(filter %.js,$^) > $@
	@chmod a-w $@

clean:
	rm -f svg2physics*.js

.PHONY: all clean