# Headphone Parametric EQ (STM32F407G-DISC1)

A USB audio device that applies a 5-band parametric EQ to whatever the PC plays and
outputs it through the headphone jack of the STM32F407G-DISC1 board.

```
PC (USB audio, 48 kHz, 16-bit stereo)
   -> USB OTG FS (audio class)
   -> clock-matching buffer (this project's own stage)
   -> 5-band biquad EQ
   -> I2S3 + DMA -> CS43L22 codec -> headphone jack
```

## Features

* Appears on the PC as a standard USB sound card (no driver needed).
* 5-band EQ: peaking, low-shelf, high-shelf, plus the 2nd-order low pass from the
original PC prototype. Settings are a fixed table you edit and reflash.
* Automatic headroom: the combined peak gain of the EQ is computed at start-up and the
input is turned down by that amount so boosts don't clip.
* Bypass button: the blue user button toggles the EQ. The orange LED is on while the EQ
is active.
* Clock matching between the PC and the board (see below), so long listening sessions
stay glitch-free.

## Hardware

* STM32F407G-DISC1 board (STM32F407VGT6, on-board CS43L22 codec and headphone jack).
* Connect the **USB OTG micro-USB connector** (not the ST-LINK one) to the PC for audio.
The ST-LINK connector is only needed for flashing and debugging.
* Headphones in the 3.5 mm jack.

## Building

* Open the folder in STM32CubeIDE (File -> Open Projects from File System).
* Configured with STM32CubeMX 6.18.1 and STM32Cube FW\_F4 V1.28.3 (see `Headphone\_Param\_EQ.ioc`).
* Build the Debug configuration, then flash with **Run**. Starting a Debug session resets
the board, and Windows switches its audio output back to the laptop speakers when the
USB device disappears.
* Your own code lives inside the `USER CODE` sections, so regenerating from the `.ioc`
keeps it. The extra files `Core/Inc/eq.h` and `Core/Src/eq.c` are not touched by CubeMX.

## Editing the EQ

Open `Core/Src/eq.c` and edit the `bands\[]` table near the top:

```c
{ EQ\_LOWSHELF,    100.0f,    +4.0f,  0.71f },
{ EQ\_PEAK,        250.0f,    -2.0f,  1.00f },
...
```

Each row is `type, frequency (Hz), gain (dB), Q`. A gain of 0 on a peak or shelf band
skips that band. For `EQ\_LOWPASS\_R`, the Q column is the pole radius `r` (0 < r < 1).

## How the clock matching works

The PC sends exactly 48,000 samples per second, but the board's I2S clock runs at
47.991 kHz because of the PLL settings available at this crystal frequency. Even with a
perfect I2S clock, the PC and the board have separate crystals and would still drift.

The USB class driver's built-in correction does not work with a circular DMA, so this
project plays from its own DMA buffer (`usbd\_audio\_if.c`). Each time half of it is
played, the next half is copied from the USB class buffer. The code keeps the amount of
unread audio near a target (`TARGET\_FILL\_MS`) and drops or repeats one stereo frame
(about 21 microseconds) when it drifts outside a dead band. If the stream stops, it
mutes and waits for fresh audio before resuming.

Tuning constants are at the top of `USB\_DEVICE/App/usbd\_audio\_if.c`:
`TARGET\_FILL\_MS`, `FILL\_TOL\_MS` and `DMA\_FRAMES`. A smaller target gives lower latency but
less tolerance to stalls on the PC side.

## Debugging

`usbd\_audio\_if.c` exports counters you can add to Live Expressions in the debugger:
`dbgSkips`, `dbgRepeats`, `dbgUnderruns`, `dbgOverruns` and `dbgFill`. In normal playback
only `dbgSkips` should climb, steadily. The other counters move only when the stream
stops or restarts.

## Licence

The code in `Drivers/` and `Middlewares/` and the generated files come from
STMicroelectronics and keep their own licence terms.
