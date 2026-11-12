#include "mcu_latency_hooks.h"
#include <string.h>

typedef struct {
  latency_tag_t tag;
  uint8_t status;
  uint32_t adopted_us, cycle, period_us, cycle_tick_us;
} record_t;

static record_t records[LATENCY_RING_CAPACITY];
static uint32_t head, count, dropped, invalid;
static latency_enter_fn enter_critical;
static latency_leave_fn leave_critical;

static uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xffffu;
  size_t i;
  unsigned int bit;
  for (i = 0; i < length; ++i) {
    crc ^= data[i];
    for (bit = 0; bit < 8; ++bit) {
      crc = (uint16_t)((crc >> 1) ^ ((crc & 1u) ? 0x8408u : 0u));
    }
  }
  return crc;
}

static uint32_t get32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void put32(uint8_t *p, uint32_t value) {
  unsigned int i;
  for (i = 0; i < 4; ++i) p[i] = (uint8_t)(value >> (8 * i));
}

static void put64(uint8_t *p, uint64_t value) {
  put32(p, (uint32_t)value);
  put32(p + 4, (uint32_t)(value >> 32));
}

int latency_init(latency_enter_fn enter, latency_leave_fn leave) {
  if (enter == NULL || leave == NULL) return 0;
  enter_critical = enter;
  leave_critical = leave;
  head = count = dropped = invalid = 0;
  return 1;
}

int latency_decode_command(const uint8_t *wire, size_t length, uint32_t rx_complete_us,
                           latency_tag_t *tag, uint8_t legacy[11]) {
  uint16_t expected;
  uint32_t saved;
  if (enter_critical == NULL || wire == NULL || tag == NULL || legacy == NULL) return 0;
  expected = (length == LATENCY_COMMAND_SIZE)
                 ? (uint16_t)(wire[29] | ((uint16_t)wire[30] << 8)) : 0u;
  if (length != LATENCY_COMMAND_SIZE || wire[0] != 0xd3 || wire[1] != 0x91 ||
      wire[2] != 1 || wire[3] != 1 || wire[4] != 23 || wire[5] != 0 ||
      crc16(wire, 29) != expected || wire[18] != 0xa5 || wire[27] != 0x5a ||
      wire[28] != 0x5b) {
    saved = enter_critical();
    ++invalid;
    leave_critical(saved);
    return 0;
  }
  tag->session = (uint64_t)get32(wire + 6) | ((uint64_t)get32(wire + 10) << 32);
  tag->sequence = get32(wire + 14);
  tag->rx_us = rx_complete_us;
  tag->parsed_us = 0;
  memcpy(legacy, wire + 18, 11);
  return 1;
}

void latency_parsed(latency_tag_t *tag, uint32_t parsed_us) { tag->parsed_us = parsed_us; }

void latency_record(const latency_tag_t *tag, uint8_t status, uint32_t adopted_us,
                    uint32_t cycle, uint32_t previous_period_us, uint32_t cycle_tick_us) {
  uint32_t saved;
  record_t *record;
  if (enter_critical == NULL || tag == NULL) return;
  saved = enter_critical();
  if (count == LATENCY_RING_CAPACITY) {
    ++dropped; /* Drop newest record; never alter target/control behavior. */
    leave_critical(saved);
    return;
  }
  record = &records[(head + count) % LATENCY_RING_CAPACITY];
  record->tag = *tag;
  record->status = status;
  record->adopted_us = status == LATENCY_APPLIED ? adopted_us : 0u;
  record->cycle = cycle;
  record->period_us = previous_period_us;
  record->cycle_tick_us = cycle_tick_us;
  ++count;
  leave_critical(saved);
}

int latency_peek_ack(uint8_t wire[LATENCY_ACK_SIZE]) {
  record_t record;
  uint16_t crc;
  uint32_t saved;
  if (enter_critical == NULL || wire == NULL) return 0;
  saved = enter_critical();
  if (count == 0) {
    leave_critical(saved);
    return 0;
  }
  record = records[head];
  leave_critical(saved);
  wire[0] = 0xd3; wire[1] = 0x91; wire[2] = 1; wire[3] = 2;
  wire[4] = 37; wire[5] = 0;
  put64(wire + 6, record.tag.session);
  put32(wire + 14, record.tag.sequence);
  wire[18] = record.status;
  put32(wire + 19, record.tag.rx_us);
  put32(wire + 23, record.tag.parsed_us);
  put32(wire + 27, record.adopted_us);
  put32(wire + 31, record.cycle);
  put32(wire + 35, record.period_us);
  put32(wire + 39, record.cycle_tick_us);
  crc = crc16(wire, 43);
  wire[43] = (uint8_t)crc; wire[44] = (uint8_t)(crc >> 8);
  return 1;
}

void latency_pop_ack(void) {
  uint32_t saved;
  if (enter_critical == NULL) return;
  saved = enter_critical();
  if (count != 0) { head = (head + 1) % LATENCY_RING_CAPACITY; --count; }
  leave_critical(saved);
}

uint32_t latency_dropped_records(void) {
  uint32_t saved, value;
  if (enter_critical == NULL) return 0;
  saved = enter_critical(); value = dropped; leave_critical(saved);
  return value;
}

uint32_t latency_invalid_commands(void) {
  uint32_t saved, value;
  if (enter_critical == NULL) return 0;
  saved = enter_critical(); value = invalid; leave_critical(saved);
  return value;
}
