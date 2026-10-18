/*
 * SPDX-License-Identifier: MIT
 *
 * General MIDI synthesiser for Stack-chan (self.audio.midi_* tools).
 *
 * TinySoundFont plays the SoundFont "soundfont.sf2" from the assets
 * partition. Its 16-bit samples are read in place from memory-mapped flash
 * (TSF_SHORT_SAMPLES patch), so only the preset tables (a few hundred KB)
 * live in PSRAM. One task renders mono PCM at the codec rate and queues it
 * on the AudioService playback queue, the same path as decoded speech.
 *
 * Two inputs: a whole MIDI file (parsed by TinyMidiLoader, played in time)
 * and raw MIDI messages sent live (note on/off, program, control, pitch
 * bend). Playback pauses while the robot speaks and stops when it starts
 * listening, so the music neither talks over replies nor reaches the mic.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct tsf;
struct tml_message;

class MidiPlayer {
public:
    static MidiPlayer& GetInstance();

    // Starts playing a Standard MIDI File. `data` is copied by the parser
    // and can be freed afterwards. loops = extra repeats (0 plays once).
    // volume is 1..100. Returns "" or an error string.
    std::string PlayFile(const uint8_t* data, size_t size, int loops, int volume);

    // Applies raw MIDI channel messages straight away (running status is
    // allowed). Returns "" or an error string.
    std::string Send(const uint8_t* data, size_t size, int volume);

    // Stops the file and silences every note.
    void Stop();

    // {"loaded", "playing", "position_ms", "length_ms", "voices", ...}
    cJSON* Status();

private:
    MidiPlayer() = default;
    MidiPlayer(const MidiPlayer&) = delete;
    MidiPlayer& operator=(const MidiPlayer&) = delete;

    std::string EnsureLoadedLocked();
    void ResetChannelsLocked();
    void ApplyMessageLocked(const tml_message* m);
    void SetVolumeLocked(int volume);
    void FreeFileLocked();
    void Wake();
    static void TaskTrampoline(void* arg);
    void TaskMain();

    std::mutex mutex_;
    tsf* synth_ = nullptr;
    int output_rate_ = 24000;
    TaskHandle_t task_ = nullptr;

    // Protected by mutex_.
    tml_message* file_ = nullptr;      // first message, owns the list
    const tml_message* next_ = nullptr;
    unsigned int file_length_ms_ = 0;
    double position_ms_ = 0;
    int loops_left_ = 0;
    bool file_playing_ = false;
    std::string last_stop_reason_;
    uint8_t running_status_ = 0;
};
