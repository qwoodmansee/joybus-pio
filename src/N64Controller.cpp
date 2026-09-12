#include "N64Controller.hpp"

#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/stdlib.h>
#include <pico/time.h>

N64Controller::N64Controller(uint pin, uint polling_rate, PIO pio, int sm, int offset) {
    joybus_port_init(&_port, pin, pio, sm, offset);
    _polling_period_us = 1'000'000 / polling_rate;
    _next_poll = get_absolute_time();
}

N64Controller::~N64Controller() {
    joybus_port_terminate(&_port);
}

void __no_inline_not_in_flash_func(N64Controller::_wait_poll_cooldown)() {
    // Wait for start of next polling period.
    while (!time_reached(_next_poll)) {
        tight_loop_contents();
    }

    // Reset poll cooldown.
    _next_poll = make_timeout_time_us(_polling_period_us);
}

bool __no_inline_not_in_flash_func(N64Controller::_init)() {
    // Send probe command.
    uint8_t probe_cmd[] = { (uint8_t)N64Command::PROBE };
    joybus_send_bytes(&_port, probe_cmd, sizeof(probe_cmd));

    // Read and validate probe response.
    uint received_len = joybus_receive_bytes(
        &_port,
        (uint8_t *)&_status,
        sizeof(n64_status_t),
        receive_timeout_us,
        true
    );

    // If response is invalid, return false.
    if (received_len != sizeof(n64_status_t) || _status.device == 0) {
        return false;
    }

    // Wait until start of next polling period before sending origin.
    _wait_poll_cooldown();

    // Send origin command.
    uint8_t origin_cmd[] = { (uint8_t)N64Command::POLL };
    joybus_send_bytes(&_port, origin_cmd, sizeof(origin_cmd));

    // Read and validate poll response.
    n64_report_t report;
    received_len = joybus_receive_bytes(
        &_port,
        (uint8_t *)&report,
        sizeof(n64_report_t),
        receive_timeout_us,
        true
    );

    // If response is invalid, return false.
    if (received_len != sizeof(n64_report_t)) {
        return false;
    }

    return true;
}

bool __no_inline_not_in_flash_func(N64Controller::Poll)(n64_report_t *report, bool rumble) {
    // If controller is uninitialized, do probe/origin sequence.
    if (!_initialized) {
        _wait_poll_cooldown();

        _initialized = _init();
        if (!_initialized) {
            return false;
        }
    }

    _wait_poll_cooldown();

    // Send poll command. N64 POLL is the single byte 0x01 — the 3-byte
    // `01 03 rumble` form is the GameCube poll's shape and confuses the
    // controller (rumble on N64 goes through pak writes, not the poll).
    // `_init` already polls with the single byte and gets a clean report.
    (void)rumble;
    uint8_t poll_cmd[] = { (uint8_t)N64Command::POLL };
    joybus_send_bytes(&_port, poll_cmd, sizeof(poll_cmd));

    // Read and validate report.
    uint8_t received_len = joybus_receive_bytes(
        &_port,
        (uint8_t *)report,
        sizeof(n64_report_t),
        receive_timeout_us,
        true
    );

    // If response is invalid, restart the initialization process.
    if (received_len != sizeof(n64_report_t)) {
        _initialized = false;
        return false;
    }

    return true;
}

int N64Controller::GetOffset() {
    return _port.offset;
}
