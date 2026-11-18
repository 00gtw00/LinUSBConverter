# Bounded RAW extension

This fork adds an experimental, bounded RAW transaction mode to firmware 3.10.
It is intended for long request/response records used by the Tesla 16 V BMS
bootloader, which do not fit the normal 8-byte LIN frame commands.

The extension preserves the stock 3.8 command set. It does not implement a BMS
flasher, EEPROM editor, or MOSFET control policy; the host remains responsible
for the target protocol and all safety gates.

## Identification

Commands use ASCII and end with CR (`0x0D`):

| Command | Reply | Meaning |
|---|---|---|
| `v` | `v030A` | Fork firmware version 3.10 |
| `q` | `qRAW02` | Bounded RAW plus adapter-timed probe is present |

## RAW transaction

Send `xNN<HEX>\r`, where `NN` is a two-digit hex byte count from `01` through
`4C` (1-76 bytes), followed by exactly `2*NN` hex digits. The adapter transmits
those bytes through the LIN UART without adding BREAK, SYNC, PID, payload
length, or checksum.

On accepted transmit the adapter replies `Z\r`. Received UART bytes are emitted
after more than 3 ms of bus idle as one or more records:

```text
xNN<HEX>\r
```

Each record contains at most 28 bytes. Reception starts with the physical echo
of the transmitted bytes; host software must compare and remove that echo
before parsing the target response. `E01\r` means the 160-byte receive buffer
overflowed, so the whole transaction must be rejected.

Malformed commands, zero length, lengths above 76, trailing characters, or
invalid hex return BEL (`0x07`) and do not transmit.

## Adapter-timed probe

`XNN<HEX>\r` validates and stores the same 1-76 byte RAW payload, but does not
transmit it immediately. It is accepted only while monitor mode is open and
replies `Z\r`. The next successfully transmitted monitor-mode LIN frame starts
the following bounded sequence inside the adapter:

- switch to RAW receive mode after the LIN frame has completed;
- send the stored bytes first after 1 ms, then every 20 ms;
- emit received bytes using the same `xNN<HEX>\r` records;
- stop automatically after 25 seconds.

`C\r` cancels an armed or active probe. An immediate lowercase `x` transaction
also cancels it. This removes host USB/OS scheduling from a short target reset
window; it does not interpret the stored bytes or add any target write service.

## Build

The RAW extension enables LTO because STM32F042G4 has 16 KiB flash:

```bash
cd embedded
make clean
make
```

The reference build uses GNU Tools for STM32 14.3.rel1. Hardware validation is
still required for the new `X` handoff, echo timing, maximum response length,
and recovery after overflow or a disconnected target.
