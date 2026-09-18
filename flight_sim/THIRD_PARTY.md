# Third-party components

- **Godot 4.5.2**, MIT license: https://godotengine.org/license/ . The locally downloaded engine is in `.tools/Godot.app`. Copyright and license text, including SDL controller support, are supplied with the engine. Official release: https://github.com/godotengine/godot-builds/releases/tag/4.5.2-stable . SHA-256 for the macOS universal archive: `2a3f35cf5813b0d26e3f4c15dabc5e7c58407fceec7bae5291740772f72d141a`.
- **Betaflight 4.5.2**, GNU GPL version 3 or later: https://github.com/betaflight/betaflight/tree/4.5.2 . Complete downloaded source, license, and the compiled native simulator are in `vendor/betaflight-4.5.2`. The archive is retained in `.tools/betaflight.tar.gz`. Local changes are reproducible using `tools/patch_betaflight.py`; build flags are in `tools/build_betaflight.py`, called by setup and the launcher. Preserve the applicable source and license obligations if redistributing a binary bundle.

The flight scene is generated from primitive geometry; no external artwork, terrain, fonts, or audio were downloaded.
