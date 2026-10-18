/*
 * SPDX-License-Identifier: MIT
 *
 * See midi_player.h.
 */
#include "midi_player.h"

#include <algorithm>
#include <vector>

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/idf_additions.h>

#include "application.h"
#include "assets.h"
#include "board.h"

// Every TinySoundFont / TinyMidiLoader allocation goes to PSRAM: internal
// RAM is short, and the preset tables alone are a few hundred KB.
#define TSF_MALLOC(size) heap_caps_malloc((size), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define TSF_REALLOC(ptr, size) heap_caps_realloc((ptr), (size), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define TSF_FREE(ptr) heap_caps_free(ptr)
#define TML_MALLOC TSF_MALLOC
#define TML_REALLOC TSF_REALLOC
#define TML_FREE TSF_FREE
#define TSF_NO_STDIO
#define TML_NO_STDIO
#define TSF_SHORT_SAMPLES
#define TSF_IMPLEMENTATION
#include "tsf.h"
#define TML_IMPLEMENTATION
#include "tml.h"

#define TAG "MidiPlayer"

namespace {
constexpr const char* kSoundFontAsset = "soundfont.sf2";
constexpr int kMaxVoices = 24;
// Block pushed to the playback queue, matching the Opus frame length of
// speech. At 24 kHz it is 2880 bytes, so it lands in PSRAM.
constexpr int kBlockMs = 60;
// Synth gain at volume 100. Leaves headroom for chords; the speaker
// volume still applies on top.
constexpr float kMaxGain = 0.5f;
constexpr size_t kMaxSendBytes = 4096;
}  // namespace

MidiPlayer& MidiPlayer::GetInstance() {
    static MidiPlayer instance;
    return instance;
}

std::string MidiPlayer::EnsureLoadedLocked() {
    if (synth_ != nullptr) return "";
    void* ptr = nullptr;
    size_t size = 0;
    if (!Assets::GetInstance().GetAssetData(kSoundFontAsset, ptr, size)) {
        return "soundfont.sf2 not in the assets partition (flash generated_assets.bin)";
    }
    synth_ = tsf_load_memory(ptr, static_cast<int>(size));
    if (synth_ == nullptr) return "could not load soundfont.sf2";
    if (!synth_->fontSamplesBorrowed) {
        ESP_LOGW(TAG, "SoundFont samples are not aligned, copied to PSRAM");
    }
    auto codec = Board::GetInstance().GetAudioCodec();
    output_rate_ = codec != nullptr && codec->output_sample_rate() > 0 ? codec->output_sample_rate() : 24000;
    tsf_set_output(synth_, TSF_MONO, output_rate_, 0.0f);
    tsf_set_max_voices(synth_, kMaxVoices);
    ResetChannelsLocked();
    ESP_LOGI(TAG, "Loaded %s: %u KB, %d presets, %d Hz, free PSRAM %u KB", kSoundFontAsset,
             static_cast<unsigned>(size / 1024), tsf_get_presetcount(synth_), output_rate_,
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));

    if (task_ == nullptr &&
        xTaskCreatePinnedToCoreWithCaps(TaskTrampoline, "midi_synth", 8192, this, 2, &task_, 1,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        task_ = nullptr;
        return "task_create_failed";
    }
    return "";
}

// General MIDI defaults: piano everywhere, the drum kit on channel 10.
void MidiPlayer::ResetChannelsLocked() {
    tsf_reset(synth_);
    for (int ch = 0; ch < 16; ch++) {
        tsf_channel_set_presetnumber(synth_, ch, 0, ch == 9);
    }
    running_status_ = 0;
}

void MidiPlayer::SetVolumeLocked(int volume) {
    volume = std::clamp(volume, 1, 100);
    tsf_set_volume(synth_, kMaxGain * static_cast<float>(volume) / 100.0f);
}

void MidiPlayer::FreeFileLocked() {
    if (file_ != nullptr) tml_free(file_);
    file_ = nullptr;
    next_ = nullptr;
    file_playing_ = false;
}

void MidiPlayer::ApplyMessageLocked(const tml_message* m) {
    switch (m->type) {
        case TML_NOTE_ON:
            if (m->velocity > 0) {
                tsf_channel_note_on(synth_, m->channel, m->key, m->velocity / 127.0f);
            } else {
                tsf_channel_note_off(synth_, m->channel, m->key);
            }
            break;
        case TML_NOTE_OFF:
            tsf_channel_note_off(synth_, m->channel, m->key);
            break;
        case TML_PROGRAM_CHANGE:
            tsf_channel_set_presetnumber(synth_, m->channel, m->program, m->channel == 9);
            break;
        case TML_PITCH_BEND:
            tsf_channel_set_pitchwheel(synth_, m->channel, m->pitch_bend);
            break;
        case TML_CONTROL_CHANGE:
            tsf_channel_midi_control(synth_, m->channel, m->control, m->control_value);
            break;
        default:
            break;
    }
}

std::string MidiPlayer::PlayFile(const uint8_t* data, size_t size, int loops, int volume) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string error = EnsureLoadedLocked();
    if (!error.empty()) return error;
    tml_message* file = tml_load_memory(data, static_cast<int>(size));
    if (file == nullptr) return "not a valid MIDI file";
    FreeFileLocked();
    ResetChannelsLocked();
    SetVolumeLocked(volume);
    file_ = file;
    next_ = file;
    tml_get_info(file, nullptr, nullptr, nullptr, nullptr, &file_length_ms_);
    position_ms_ = 0;
    loops_left_ = std::max(0, loops);
    file_playing_ = true;
    last_stop_reason_.clear();
    ESP_LOGI(TAG, "Playing MIDI file: %u bytes, %u ms, loops=%d", static_cast<unsigned>(size),
             file_length_ms_, loops_left_);
    Wake();
    return "";
}

std::string MidiPlayer::Send(const uint8_t* data, size_t size, int volume) {
    if (size > kMaxSendBytes) return "too many bytes in one send";
    std::lock_guard<std::mutex> lock(mutex_);
    std::string error = EnsureLoadedLocked();
    if (!error.empty()) return error;
    if (!file_playing_) SetVolumeLocked(volume);
    uint8_t status = running_status_;
    size_t i = 0;
    while (i < size) {
        uint8_t b = data[i];
        if (b == 0xF0) {  // SysEx: skip to its end
            while (i < size && data[i] != 0xF7) i++;
            i++;
            continue;
        }
        if (b >= 0xF0) {  // other system messages carry no channel data we use
            i++;
            continue;
        }
        if (b & 0x80) {
            status = b;
            i++;
        }
        if (status == 0) return "data byte without a status byte";
        int type = status & 0xF0;
        int ch = status & 0x0F;
        size_t need = (type == 0xC0 || type == 0xD0) ? 1 : 2;
        if (i + need > size) return "truncated MIDI message";
        uint8_t d1 = data[i] & 0x7F;
        uint8_t d2 = need == 2 ? (data[i + 1] & 0x7F) : 0;
        i += need;
        switch (type) {
            case 0x80:
                tsf_channel_note_off(synth_, ch, d1);
                break;
            case 0x90:
                if (d2 > 0) {
                    tsf_channel_note_on(synth_, ch, d1, d2 / 127.0f);
                } else {
                    tsf_channel_note_off(synth_, ch, d1);
                }
                break;
            case 0xB0:
                tsf_channel_midi_control(synth_, ch, d1, d2);
                break;
            case 0xC0:
                tsf_channel_set_presetnumber(synth_, ch, d1, ch == 9);
                break;
            case 0xE0:
                tsf_channel_set_pitchwheel(synth_, ch, d1 | (d2 << 7));
                break;
            default:  // key / channel pressure: not supported by TinySoundFont
                break;
        }
    }
    running_status_ = status;
    Wake();
    return "";
}

void MidiPlayer::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (synth_ == nullptr) return;
    if (file_playing_) last_stop_reason_ = "stopped";
    FreeFileLocked();
    tsf_note_off_all(synth_);
}

cJSON* MidiPlayer::Status() {
    std::lock_guard<std::mutex> lock(mutex_);
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "loaded", synth_ != nullptr);
    cJSON_AddBoolToObject(root, "playing", file_playing_);
    if (synth_ != nullptr) {
        cJSON_AddNumberToObject(root, "presets", tsf_get_presetcount(synth_));
        cJSON_AddNumberToObject(root, "voices", tsf_active_voice_count(synth_));
        cJSON_AddNumberToObject(root, "sample_rate", output_rate_);
    }
    if (file_playing_) {
        cJSON_AddNumberToObject(root, "position_ms", static_cast<int>(position_ms_));
        cJSON_AddNumberToObject(root, "length_ms", file_length_ms_);
        cJSON_AddNumberToObject(root, "loops_left", loops_left_);
    }
    if (!last_stop_reason_.empty()) {
        cJSON_AddStringToObject(root, "last_stop", last_stop_reason_.c_str());
    }
    return root;
}

void MidiPlayer::Wake() {
    if (task_ != nullptr) xTaskNotifyGive(task_);
}

void MidiPlayer::TaskTrampoline(void* arg) {
    static_cast<MidiPlayer*>(arg)->TaskMain();
}

// Sleeps until there is something to play, then renders blocks while a
// file plays or notes are still sounding. The playback queue holds only a
// couple of blocks, so pushing blocks here paces the loop in real time.
void MidiPlayer::TaskMain() {
    auto& app = Application::GetInstance();
    auto& audio = app.GetAudioService();
    const int block = output_rate_ * kBlockMs / 1000;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (true) {
            DeviceState state = app.GetDeviceState();
            if (state == kDeviceStateListening) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (file_playing_) last_stop_reason_ = "listening";
                FreeFileLocked();
                tsf_reset(synth_);
                break;
            }
            if (state == kDeviceStateSpeaking) {
                // Speech owns the speaker; carry on where we were afterwards.
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            std::vector<int16_t> pcm(block);
            bool more;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                int done = 0;
                while (done < block) {
                    int chunk = block - done;
                    if (file_playing_) {
                        while (next_ != nullptr && next_->time <= position_ms_) {
                            ApplyMessageLocked(next_);
                            next_ = next_->next;
                        }
                        if (next_ == nullptr) {
                            tsf_note_off_all(synth_);
                            if (loops_left_ > 0) {
                                loops_left_--;
                                next_ = file_;
                                position_ms_ = 0;
                                continue;
                            }
                            FreeFileLocked();
                            last_stop_reason_ = "finished";
                        } else {
                            // Render up to the next event so it lands on time.
                            double until = (next_->time - position_ms_) * output_rate_ / 1000.0;
                            chunk = std::min(chunk, std::max(1, static_cast<int>(until)));
                        }
                    }
                    tsf_render_short(synth_, pcm.data() + done, chunk, 0);
                    done += chunk;
                    if (file_playing_) position_ms_ += chunk * 1000.0 / output_rate_;
                }
                more = file_playing_ || tsf_active_voice_count(synth_) > 0;
            }
            if (!audio.PushPcmToPlaybackQueue(std::move(pcm))) break;
            if (!more) break;
        }
    }
}
