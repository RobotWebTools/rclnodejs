.PHONY: docs

docs:
	npx jsdoc --package ../../package.json ../../index.js ../../lib/*.js ../../lib/action/*.js -t . -d ../../docs
	node ./build-index.js
