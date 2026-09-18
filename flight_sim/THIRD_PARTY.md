# Third-party components

- **Godot 4.4.1**, MIT license: https://godotengine.org/license/ . The locally downloaded engine is in `.tools/Godot.app`. Copyright and license text are supplied with the engine.
- **Betaflight 4.5.2**, GNU GPL version 3 or later: https://github.com/betaflight/betaflight/tree/4.5.2 . Complete downloaded source, license, and the compiled native simulator are in `vendor/betaflight-4.5.2`. The archive is retained in `.tools/betaflight.tar.gz`. Local changes are reproducible using `tools/patch_betaflight.py`; build flags are in `tools/setup.sh`. Preserve the applicable source and license obligations if redistributing a binary bundle.

The flight scene is generated from primitive geometry; no external artwork, terrain, fonts, or audio were downloaded.
