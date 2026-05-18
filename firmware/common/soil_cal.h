#pragma once

// Soil moisture calibration via USB-CDC.
//
// Lets the web UI capture dry / wet anchor voltages over USB serial and
// persist them to NVS so the soil_moisture lambda uses them on subsequent
// boots — without reflashing. Falls back to the YAML substitution defaults
// (${soil_dry_value} / ${soil_wet_value}) when NVS is unset.
//
// Protocol (line-based ASCII, \n-terminated, "@CAL " sentinel both ways):
//
//   Host -> Device
//     @CAL PING
//     @CAL GET
//     @CAL SET DRY <float>
//     @CAL SET WET <float>
//     @CAL RESET
//     @CAL STREAM ON | @CAL STREAM OFF
//
//   Device -> Host
//     @CAL OK <verb> [k=v ...]
//     @CAL ERR <verb> <reason>
//     @CAL RAW <float>
//     @CAL READY

#include <string>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"

#include "nvs_helper.h"

namespace planthub {

  // Live calibration state. Read by the soil_moisture lambda every poll;
  // updated at boot from NVS-or-YAML, and on each SET command.
  inline float soil_cal_dry = 0.0f;
  inline float soil_cal_wet = 0.0f;
  inline float soil_cal_raw_v = 0.0f;   // latest raw ADC voltage (lambda writes, streamer reads)
  inline bool  soil_cal_streaming = false;

  namespace _soil_cal {
    inline float    yaml_default_dry = 0.0f;
    inline float    yaml_default_wet = 0.0f;
    inline bool     inited = false;
    inline bool     driver_ready = false;
    inline bool     ready_emitted = false;
    inline char     line_buf[128];
    inline size_t   line_len = 0;
    inline uint32_t last_stream_ms = 0;

    inline uint32_t now_ms() {
      return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    }

    // All output goes through printf so it shares newlib's stdio mutex with
    // ESPHome's logger — no torn lines.
    inline void writeln(const char* s) {
      printf("%s\n", s);
      fflush(stdout);
    }

    inline bool current_src_is_nvs() {
      return nvs_has_key("sc_dry") || nvs_has_key("sc_wet");
    }

    inline void emit_state(const char* verb) {
      char buf[128];
      snprintf(buf, sizeof(buf),
               "@CAL OK %s dry=%.3f wet=%.3f raw=%.3f src=%s",
               verb, soil_cal_dry, soil_cal_wet, soil_cal_raw_v,
               current_src_is_nvs() ? "nvs" : "yaml");
      writeln(buf);
    }

    inline void emit_err(const char* verb, const char* reason) {
      char buf[96];
      snprintf(buf, sizeof(buf), "@CAL ERR %s %s", verb, reason);
      writeln(buf);
    }

    inline bool parse_float_strict(const char* s, float& out) {
      if (s == nullptr || *s == '\0') return false;
      char* end = nullptr;
      float v = std::strtof(s, &end);
      if (end == s) return false;
      while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') end++;
      if (*end != '\0') return false;
      if (std::isnan(v) || std::isinf(v)) return false;
      out = v;
      return true;
    }

    inline void handle_set(const char* which, const char* value_str) {
      float v;
      if (!parse_float_strict(value_str, v)) {
        emit_err("SET", "parse_failed");
        return;
      }
      // ADC on ESP32-C3 with 12 dB attenuation tops near 3.3 V; gate to a
      // sane range so a typo can't poison the curve.
      if (!(v > 0.0f && v < 4.0f)) {
        emit_err("SET", "out_of_range");
        return;
      }
      const char* key;
      if (std::strcmp(which, "DRY") == 0) {
        soil_cal_dry = v;
        key = "sc_dry";
      } else if (std::strcmp(which, "WET") == 0) {
        soil_cal_wet = v;
        key = "sc_wet";
      } else {
        emit_err("SET", "unknown_field");
        return;
      }
      char vs[16];
      snprintf(vs, sizeof(vs), "%.3f", v);
      if (!nvs_write(key, std::string(vs))) {
        emit_err("SET", "nvs_write_failed");
        return;
      }
      nvs_save();
      char buf[128];
      snprintf(buf, sizeof(buf),
               "@CAL OK SET %s value=%.3f src=nvs", which, v);
      writeln(buf);
    }

    inline void handle_reset() {
      nvs_delete("sc_dry");
      nvs_delete("sc_wet");
      nvs_save();
      soil_cal_dry = yaml_default_dry;
      soil_cal_wet = yaml_default_wet;
      emit_state("RESET");
    }

    inline void handle_line(char* line) {
      size_t n = std::strlen(line);
      while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == ' ' || line[n - 1] == '\t')) {
        line[--n] = '\0';
      }
      if (n == 0) return;
      if (std::strncmp(line, "@CAL ", 5) != 0) return;
      char* rest = line + 5;
      while (*rest == ' ') rest++;

      if (std::strcmp(rest, "PING") == 0) {
        writeln("@CAL OK PING");
      } else if (std::strcmp(rest, "GET") == 0) {
        emit_state("GET");
      } else if (std::strcmp(rest, "RESET") == 0) {
        handle_reset();
      } else if (std::strcmp(rest, "STREAM ON") == 0) {
        soil_cal_streaming = true;
        last_stream_ms = now_ms();
        writeln("@CAL OK STREAM state=on");
      } else if (std::strcmp(rest, "STREAM OFF") == 0) {
        soil_cal_streaming = false;
        writeln("@CAL OK STREAM state=off");
      } else if (std::strncmp(rest, "SET DRY ", 8) == 0) {
        handle_set("DRY", rest + 8);
      } else if (std::strncmp(rest, "SET WET ", 8) == 0) {
        handle_set("WET", rest + 8);
      } else {
        emit_err("UNK", "unknown_command");
      }
    }
  } // namespace _soil_cal

  // Called once from the soil_moisture sensor lambda the first time it runs.
  // The lazy-init avoids touching base.yaml's consolidated on_boot list
  // (see the comment at base.yaml:34-38 about ESPHome package merge).
  inline void soil_cal_init(float yaml_dry, float yaml_wet) {
    using namespace _soil_cal;
    if (inited) return;
    inited = true;
    yaml_default_dry = yaml_dry;
    yaml_default_wet = yaml_wet;

    soil_cal_dry = yaml_dry;
    soil_cal_wet = yaml_wet;

    if (nvs_initialized) {
      std::string s_dry = nvs_read("sc_dry");
      if (!s_dry.empty()) {
        float v;
        if (parse_float_strict(s_dry.c_str(), v) && v > 0.0f && v < 4.0f) {
          soil_cal_dry = v;
        }
      }
      std::string s_wet = nvs_read("sc_wet");
      if (!s_wet.empty()) {
        float v;
        if (parse_float_strict(s_wet.c_str(), v) && v > 0.0f && v < 4.0f) {
          soil_cal_wet = v;
        }
      }
    }

    // ESPHome's USB-CDC logger may have already installed the driver.
    // Treat ESP_ERR_INVALID_STATE as success — it just means the driver
    // is up and we can read from it.
    usb_serial_jtag_driver_config_t cfg = {
      .tx_buffer_size = 256,
      .rx_buffer_size = 256,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    driver_ready = (err == ESP_OK || err == ESP_ERR_INVALID_STATE);
  }

  // Called from a 50 ms interval block in soil_moisture_adc.yaml.
  // Non-blocking; safe to call before init (no-ops).
  inline void soil_cal_poll() {
    using namespace _soil_cal;
    if (!inited) return;

    if (!ready_emitted) {
      writeln("@CAL READY");
      ready_emitted = true;
    }

    if (driver_ready) {
      uint8_t buf[64];
      int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
      for (int i = 0; i < n; i++) {
        char c = static_cast<char>(buf[i]);
        if (c == '\n') {
          line_buf[line_len] = '\0';
          handle_line(line_buf);
          line_len = 0;
        } else if (line_len + 1 < sizeof(line_buf)) {
          line_buf[line_len++] = c;
        } else {
          // Line overflow — discard and resync at next \n.
          line_len = 0;
        }
      }
    }

    if (soil_cal_streaming) {
      uint32_t t = now_ms();
      if (t - last_stream_ms >= 500) {
        last_stream_ms = t;
        char buf[48];
        snprintf(buf, sizeof(buf), "@CAL RAW %.3f", soil_cal_raw_v);
        writeln(buf);
      }
    }
  }

} // namespace planthub
