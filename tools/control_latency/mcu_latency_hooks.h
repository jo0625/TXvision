#ifndef MCU_LATENCY_HOOKS_H
#define MCU_LATENCY_HOOKS_H

/* Portable integration component, NOT wired into the unidentified flashed MCU.
 * Supply a free-running 1 MHz uint32 timer; do not reset an existing timer.
 * enter/leave must save/restore interrupt state, never block or print.
 */
#include <stddef.h>
#include <stdint.h>

#define LATENCY_APPLIED 1u
#define LATENCY_SUPERSEDED 2u
#define LATENCY_REJECTED 3u
#define LATENCY_NOT_ADOPTED 4u
#define LATENCY_ACK_SIZE 45u
#define LATENCY_COMMAND_SIZE 31u
#define LATENCY_RING_CAPACITY 64u

typedef struct {
  uint64_t session;
  uint32_t sequence;
  uint32_t rx_us;
  uint32_t parsed_us;
} latency_tag_t;

typedef uint32_t (*latency_enter_fn)(void);
typedef void (*latency_leave_fn)(uint32_t saved_state);

/* Requires valid callbacks; all shared ring operations use them. */
int latency_init(latency_enter_fn enter, latency_leave_fn leave);
/* Call AFTER recording last-byte arrival and BEFORE validating/decoding.
 * This validates the separate V1 envelope, not legacy commands or target limits.
 * Copy bytes through the existing legacy parser; use the same limits/semantics.
 */
int latency_decode_command(const uint8_t *wire, size_t length, uint32_t rx_complete_us,
                           latency_tag_t *tag, uint8_t legacy[11]);
void latency_parsed(latency_tag_t *tag, uint32_t parsed_us);
/* APPLIED: call where the controller actually reads/uses the tagged target.
 * NOT at USB callback, parse completion, or target mailbox assignment.
 * Call once per NEW tag; unchanged target reuse must not create repeated ACKs.
 * For other statuses adopted_us is zero; zero is a valid APPLIED timer value.
 */
void latency_record(const latency_tag_t *tag, uint8_t status, uint32_t adopted_us,
                    uint32_t cycle, uint32_t previous_period_us, uint32_t cycle_tick_us);
/* One existing telemetry task consumes the ring. No new timer/ISR transmission.
 * Keep the encoded TX buffer alive until USB Tx-complete. On BUSY do not pop.
 */
int latency_peek_ack(uint8_t wire[LATENCY_ACK_SIZE]);
void latency_pop_ack(void);
uint32_t latency_dropped_records(void);
uint32_t latency_invalid_commands(void);

#endif
