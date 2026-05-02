# t4-i2s Manual Test Notes

This document records the manual serial-monitor tests performed during
development. The tests were run on the LilyGo T4 AMOLED ESP32-S3 with a FAT32 SD
card and the I2S microphone connected.

## Test: SD Recording Start/Stop

Purpose:

Verify that button-controlled recording creates WAV files, writes data, and
stops cleanly.

Procedure:

1. Boot the device.
2. Press the top button to start recording.
3. Press the top button again to stop recording.
4. Repeat to create more than one file.
5. Reboot and inspect the boot-time SD file list.

Observed result:

```text
recording started: /3.wav
record stop requested
record start requested
recording started: /4.wav
record stop requested

SD file 1.wav size=44
SD file 2.wav size=100396
SD file 3.wav size=43052
SD file 4.wav size=34860
```

Outcome:

Pass. New WAV files were created with increasing numeric names. The earlier
`1.wav` was a 44-byte header-only file from a previous failed header-write test.
Later recordings produced non-empty WAV files.

## Test: Long Recording Around 9 MB

Purpose:

Verify that asynchronous SD writing keeps up during a longer recording.

Procedure:

1. Start recording.
2. Let recording run until the WAV data reaches about 9 MB.
3. Stop recording.
4. Watch SD queue depth, queue drops, SD write timings, and final byte count.

Key observed serial values:

```text
sd status: recording ready=1 file=/1.wav q=1 free=255 chunks=4539 bytes=9295872 drops=0 total_drops=0 q_high=12 errors=0 write_avg=7781us write_max=569547us
recording stopped: /1.wav bytes=9578496 chunks=4677 errors=0
sd status: idle ready=1 file=/1.wav q=0 free=256 chunks=4677 bytes=9578496 drops=0 total_drops=0 q_high=12 errors=0 write_avg=7774us write_max=569547us
```

Observed timing:

- Capture period stayed close to 64 ms.
- Capture active duty stayed around 3.6% to 4.3%.
- SD queue was usually `q=1`.
- SD queue high-water reached 12 frames.
- No queue drops.
- No SD errors.
- Worst observed SD write stall was about 570 ms.

Interpretation:

The recording queue absorbed the worst SD write stall without losing audio. A
high-water mark of 12 frames is about 768 ms of buffered audio. The configured
recording buffer holds 256 frames, or about 16 seconds, so the observed run had
large buffering headroom.

Outcome:

Pass.

## Test: FTP Download While Idle

Purpose:

Verify that FTP download works when no recording is active and does not disturb
live capture/display diagnostics.

Procedure:

1. Boot the device and connect over FTP using plain FTP, no TLS, max
   connections set to 1.
2. Download `1.wav`.
3. Watch serial diagnostics.

Key observed serial values:

```text
FTP: Finish transfer!
capture avg: period=63947us read(wait)=61500us rec_q=3us fft=578us pub_fft=646us pub_an=657us pub_scope=562us active=2447us duty=3.8%
sd status: idle ready=1 file=/1.wav q=0 free=256 chunks=4677 bytes=9578496 drops=0 total_drops=0 q_high=12 errors=0 write_avg=7774us write_max=569547us
display avg: period=63990us wait=36829us clear=0us frame=0us analyzer=9502us fft=8143us scope=9493us active=27144us duty=42.4%
```

Observed timing:

- FTP transfer completed.
- Capture period stayed close to 64 ms.
- Capture active duty stayed around 3.6% to 3.8%.
- Display duty stayed around the normal range.
- No new SD recording queue movement occurred because recording was idle.

Interpretation:

FTP download while idle is healthy. The SD stats in this test still reflected
the previous recording because no new recording was active.

Outcome:

Pass.

## Test: FTP Download While Recording

Purpose:

Verify that recording a new WAV file can continue while downloading an existing
WAV file over FTP.

Procedure:

1. Start recording `2.wav`.
2. Start FTP download of existing `1.wav`, about 9 MB.
3. Continue recording while the FTP transfer runs.
4. Stop recording after the FTP transfer finishes.
5. Watch SD queue depth, drops, errors, and write latency.

Key observed serial values:

```text
record start requested
recording started: /2.wav
FTP: Connected!
sd status: recording ready=1 file=/2.wav q=1 free=255 chunks=156 bytes=319488 drops=0 total_drops=0 q_high=1 errors=0 write_avg=6094us write_max=17553us
sd status: recording ready=1 file=/2.wav q=1 free=255 chunks=1476 bytes=3022848 drops=0 total_drops=0 q_high=2 errors=0 write_avg=10758us write_max=55388us
sd status: recording ready=1 file=/2.wav q=1 free=255 chunks=1836 bytes=3760128 drops=0 total_drops=0 q_high=4 errors=0 write_avg=12661us write_max=300169us
FTP: Finish transfer!
sd status: recording ready=1 file=/2.wav q=1 free=255 chunks=1956 bytes=4005888 drops=0 total_drops=0 q_high=4 errors=0 write_avg=12817us write_max=300169us
record stop requested
sd status: idle ready=1 file=/2.wav q=0 free=256 chunks=2163 bytes=4429824 drops=0 total_drops=0 q_high=4 errors=0 write_avg=12190us write_max=300169us
```

Observed timing:

- Capture period stayed close to 64 ms throughout.
- Capture active duty rose modestly during FTP, generally around 4% to 5.2%.
- SD write average rose from about 6 ms before FTP to about 12.8 ms during FTP.
- Worst SD write latency reached about 300 ms.
- SD queue high-water reached 4 frames.
- No recording queue drops.
- No SD errors.
- FTP transfer completed.

Interpretation:

Concurrent FTP download and recording is viable with the current buffering. A
high-water mark of 4 frames is about 256 ms of buffered audio, well below the
available 256-frame buffer. SD contention is visible in the higher average and
maximum write times, but it did not cause audio loss.

Outcome:

Pass.

## Test: FTP Control-Channel Polling

Purpose:

Verify that the FTP server accepts connections and sends the welcome message.

Initial symptom:

The FTP client could establish a TCP connection but timed out waiting for the
welcome message:

```text
Connection established, waiting for welcome message...
Connection timed out after 20 seconds of inactivity
```

Cause:

`ftpServer.handleFTP()` was being called inside a loop that also used a blocking
10 ms delay. This did not pump the FTP state machine often enough.

Fix:

`ftpServer.handleFTP()` now runs every Arduino `loop()` pass. Button
housekeeping remains on a non-blocking 10 ms `millis()` cadence.

Outcome:

Pass. FTP connections and downloads now work.

## Test: FTP Transfer Callback Logging

Purpose:

Verify that FTP download callback events are labelled correctly.

Initial symptom:

FTP downloads produced repeated lines like:

```text
FTP: Unknown Operation (transfer)
```

Cause:

The callback handled upload events but did not handle `FTP_DOWNLOAD_START` and
`FTP_DOWNLOAD`.

Fix:

The callback now handles:

- `FTP_UPLOAD_START`
- `FTP_UPLOAD`
- `FTP_DOWNLOAD_START`
- `FTP_DOWNLOAD`
- `FTP_TRANSFER_STOP`
- `FTP_TRANSFER_ERROR`

The library aliases upload/download stop to the same enum value, and upload/
download error to the same enum value. The application tracks the active
transfer direction so stop/error logs can still say upload or download.

Outcome:

Build passed after the change. Runtime verification of the new log wording is
still pending.

## Test: I2S LRCLK Wiring and Capacitance Fault

Purpose:

Diagnose a startup/capacitance problem where the displays showed no useful audio
data until a hand or finger was placed near the I2S wiring. After touching the
wiring, audio would begin but could initially be very noisy.

Initial observations:

- The microphone supply measured about 3.4 V with about 50 mV ripple.
- The Adafruit microphone `SEL` line was disconnected from GPIO and pulled to
  ground through 20k, but the capacitance-sensitive behaviour did not change.
- Increasing the I2S sample rate to 32 kHz did not fix the issue and overloaded
  the display task because capture frames arrived about every 32 ms.
- Reducing the I2S sample rate to 8 kHz doubled the observed BCLK period,
  confirming that the probed clock was derived from the I2S sample rate.

Fault isolation:

1. GPIO 42 was configured as a simple firmware square-wave output before I2S
   initialisation.
2. No square wave was seen at the suspected LRCLK probe point.
3. A clean square wave was seen when probing the actual GPIO 42 pin.
4. Normal I2S was restored and GPIO 42 produced a clean LRCLK-like square wave
   at the GPIO pin.
5. Continuity testing showed the microphone `LRCL` pin was actually wired to
   GPIO 40, not GPIO 42.

Root cause:

Firmware was generating LRCLK/word-select on GPIO 42 while the microphone LRCLK
wire was physically connected to GPIO 40. The microphone LRCLK input was
therefore not receiving a valid logic-level word-select signal. The small,
dirty waveform measured earlier was likely crosstalk/capacitive pickup from
nearby I2S activity, not a driven LRCLK signal.

Fix:

The firmware I2S LRCLK pin was changed to GPIO 40:

```cpp
constexpr int kI2SBclk_GPIO = 39;
constexpr int kI2SLrclk_GPIO = 40;
constexpr int kI2SData_GPIO = 41;
```

The codebase now uses `_GPIO` suffixes for physical pin-number constants.

Outcome:

Pass. With LRCLK generated on GPIO 40, a clean roughly 60 us LRCLK square wave
was present at the connector and the microphone worked normally once connected.

## Summary

The tested architecture is working:

- I2S capture remains stable at the expected 64 ms frame period.
- The confirmed I2S wiring is BCLK on GPIO 39, LRCLK on GPIO 40, and DOUT on
  GPIO 41.
- Recording survives SD write stalls through the PSRAM-backed queue.
- Long recording to SD produced a valid multi-megabyte WAV file with no drops.
- FTP download works while idle.
- FTP download of an existing WAV while recording a new WAV completed with no
  recording drops or SD errors.

Remaining higher-risk tests:

- FTP upload while recording.
- FTP delete/rename while recording.
- Repeated long recordings until the SD card has many files.
- Power-loss behaviour if the user shuts down before stopping recording.
