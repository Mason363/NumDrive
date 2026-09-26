# Drive Mad for NumWorks

Drive Mad, the Fancade driving game, on the NumWorks N0120 calculator.

All 200 levels, the same cars, physics puzzles, bridges, water and look as the original.

<p align="center">
  <img src="docs/shot1.png" width="320"> <img src="docs/shot2.png" width="320">
  <img src="docs/shot3.png" width="320"> <img src="docs/shot4.png" width="320">
  <img src="docs/shot5.png" width="320"> <img src="docs/shot6.png" width="320">
</p>

## Install

1. Download `DriveMad.nwa` from the [latest release](https://github.com/Mason363/NumDrive/releases/latest).
2. Plug your calculator into a computer and open [my.numworks.com/apps](https://my.numworks.com/apps) in Chrome or Edge.
3. Upload `DriveMad.nwa` and send it to the calculator.
4. Open Drive Mad from the home screen.

## Controls

| Key | Action |
| --- | --- |
| Right arrow | Drive forward |
| Left arrow | Brake and reverse |
| OK or Back | Pause |
| Arrows, then OK | Pick a button or a level |
| Back on a card | Level list |
| Home | Quit |

Your progress is saved on the calculator.

## Build

You need `arm-none-eabi-gcc` and Node.js.

```
make
```

The app is written to `output/device/drivemad.nwa`.

## Credits

Drive Mad was made by Martin Magni with [Fancade](https://www.fancade.com). This is an unofficial fan port and is not affiliated with Fancade. Levels and art come from the original game.
