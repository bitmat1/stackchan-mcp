# TinySoundFont (vendored)

`tsf.h` (v0.9) and `tml.h` from
[schellingb/TinySoundFont](https://github.com/schellingb/TinySoundFont)
at commit `853a0a1`, MIT licensed (see the header of each file).
Used by `main/boards/stackchan/midi_player.cc`.

## Local patch

`tsf.h` gains an optional `TSF_SHORT_SAMPLES` mode (search for
`TSF_SHORT_SAMPLES` and `fontSamplesBorrowed`). Upstream converts every
sample to `float`, which would need about 5.5 MB of RAM for the SoundFont
below. With the option, samples stay 16-bit, and when the font is loaded with
`tsf_load_memory` from 2-byte aligned data they are used in place, so a font
in memory-mapped flash needs no RAM for its samples. Rendered output matches
the float build to within 1 LSB.

## SoundFont

`main/boards/stackchan/assets_extra/soundfont.sf2` is **microGM** by
michael (normal version, 2.7 MB, sha256 `6b1bceb9…28e8ba`), a complete
General MIDI bank with GS drum kits made for tiny devices, licensed
[WTFPL](http://www.wtfpl.net/):
<https://musical-artifacts.com/artifacts/2529>. It is packed into the assets
partition (`generated_assets.bin`) at build time.
