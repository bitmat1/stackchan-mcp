"""Tests for the advertised avatar face list."""

from __future__ import annotations

from stackchan_mcp.faces import BASE_FACES, EXTENDED_FACES, available_faces


def test_extended_by_default(monkeypatch):
    monkeypatch.delenv("STACKCHAN_EXTENDED_FACES", raising=False)
    faces = available_faces()
    assert faces[:6] == BASE_FACES
    assert len(faces) == 27
    assert len(set(faces)) == len(faces)


def test_stock_only(monkeypatch):
    monkeypatch.setenv("STACKCHAN_EXTENDED_FACES", "0")
    assert available_faces() == BASE_FACES


def test_matches_firmware_names():
    # Keep in sync with kSpecs in firmware/components/procedural_face.
    from pathlib import Path

    src = (
        Path(__file__).resolve().parents[2]
        / "firmware/components/procedural_face/procedural_face.cc"
    ).read_text()
    for name in BASE_FACES + EXTENDED_FACES:
        assert f'{{"{name}",' in src, name
