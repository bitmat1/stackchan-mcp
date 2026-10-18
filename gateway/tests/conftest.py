"""Pytest configuration."""

import os

import pytest


# Use asyncio mode for all async tests
def pytest_collection_modifyitems(config, items):
    """Auto-mark all async tests."""
    for item in items:
        if item.get_closest_marker("asyncio") is None:
            if asyncio_test(item):
                item.add_marker(pytest.mark.asyncio)


def asyncio_test(item):
    """Check if test is async."""
    return hasattr(item, "function") and hasattr(item.function, "__wrapped__")


@pytest.fixture(autouse=True)
def _clear_gateway_env(monkeypatch):
    """Drop gateway settings inherited from the shell.

    A session started by the Stack-chan app inherits the gateway's .env
    (TTS engine, pitch limits, voice channel...), which would override
    the defaults the tests expect. Defined first so the fixtures below
    set their variables after the clear.
    """
    for name in list(os.environ):
        if name.startswith(("STACKCHAN_", "VISION_")):
            monkeypatch.delenv(name)


@pytest.fixture(autouse=True)
def _tts_wifi_awake_off(monkeypatch):
    """Keep TTS tests' recorded device traffic to the audio path itself.

    say() normally wraps playback in a Wi-Fi power-save lease (two extra
    self.wifi.set_power_save calls); tests that cover the lease opt back
    in by setting STACKCHAN_TTS_WIFI_AWAKE=1.
    """
    monkeypatch.setenv("STACKCHAN_TTS_WIFI_AWAKE", "0")


@pytest.fixture(autouse=True)
def _subtitles_off(monkeypatch):
    """say() subtitles add set_subtitle device calls; tests opt in."""
    monkeypatch.setenv("STACKCHAN_SUBTITLES", "0")
    from stackchan_mcp import subtitles

    monkeypatch.setattr(subtitles, "_override", None)
