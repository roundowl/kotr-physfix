# KoTR PhysFix

Collision fixes for **King of the Road / Hard Truck 2 / Rig'n'Roll's older brother (Дальнобойщики 2)**,
GOG version 1.3.

If you have ever had your truck freeze after a light side bump, wedge itself against a barrel or a
pole, or sit merged with another truck while both spin their wheels forever, this fixes that.

| Before | After |
|---|---|
| Touching another vehicle while sliding along it, or backing away from it, cancels **all** movement: you stay wedged until something else knocks you loose | You slide along, push it, or pull away |
| Slow contact lets vehicles sink into each other, and once inside they never separate | Slow contact stops at the surface; overlapping vehicles drift apart |
| A barrel or a light car is an immovable wall at low speed | A truck shoves a barrel; two trucks share the push by weight |
| Rare crash to desktop: `FATAL dynamic error ... MoveItem_2_3 #1 MI.col.p` | Caught and repaired, logged instead |

Fast collisions are left to the game's own response, so crashes still bounce the way they always did.

## Requirements

- **King of the Road 1.3, GOG build** (`king.exe` SHA-1 `295d134d103bf91b8467d28dc9bf74d90193c5bc`).
  Any other exe is detected and left untouched; the log says so.
- An **ASI loader**. [KoTR Modern Patch](https://github.com/aleko2144/KoTR_Modern_Patch) already
  includes one (its `dsound.dll`) and is recommended anyway. Otherwise use
  [Ultimate ASI Loader](https://github.com/ThirteenAG/Ultimate-ASI-Loader) as `dsound.dll`.

Tested with GOG 1.3 + KoTR Modern Patch 1.05 + [D2GI](https://github.com/REDPOWAR/D2GI) 0.5, on Linux
(Wine 11 + DXVK). It should behave the same on Windows; reports welcome.

## Install

1. Download `KoTR_PhysFix.zip` from Releases.
2. Put `KoTR_PhysFix.asi` and `KoTR_PhysFix.ini` into the game's `plugins` folder
   (create it if you have no Modern Patch; Ultimate ASI Loader also loads `.asi` files from the game folder).
3. Start the game. `plugins/KoTR_PhysFix.log` should say
   `KoTR_PhysFix 1.0 loaded: toi clamp on, contact point guard on, contact fix on`.

Uninstall: delete the two files.

## Settings (`KoTR_PhysFix.ini`)

```ini
[Fixes]
Contacts=1    ; 0 = original contact behaviour (the crash guard stays on)
[Debug]
Log=1         ; statistics and stuck-vehicle reports in KoTR_PhysFix.log
TraceKey=0    ; 1 = F12 records a step-by-step trace of the next two vehicles that touch something
```

## Reporting problems

Set `TraceKey=1`, drive up to where the problem happens, press **F12**, reproduce it, quit, and attach
`KoTR_PhysFix.log` plus `KoTR_PhysFix-trace1.log` / `-trace2.log` to an issue. Say what you did and
what you saw.

## Known limits

- Vehicle-vs-world (fences, poles, buildings) contacts are the game's own code, unchanged. They mostly
  behave; a vehicle pinning you against a fence can still squeeze you in.
- The game's collision boxes are not the visible models. Something can "touch" you a little outside
  the paint (a BMW is a known example), or catch in a pocket between a cab and a trailer.
- What causes the `MI.col.p` garbage contact point in the first place is not known; the plugin
  repairs it when it happens.

## How it works

See [TECHNICAL.md](TECHNICAL.md): what the engine does, the three bugs, and every patch site.

## Credits

- Reverse engineering and code: roundowl, with Claude (Anthropic).
- [KoTR Modern Patch](https://github.com/aleko2144/KoTR_Modern_Patch) by aleko2144 and CookiePLMonster,
  and [D2GI](https://github.com/REDPOWAR/D2GI) by REDPOWAR, NewEXE and CookiePLMonster: the reason
  this game is playable at all today.
- [Ultimate ASI Loader](https://github.com/ThirteenAG/Ultimate-ASI-Loader) by ThirteenAG.

MIT licence, see [LICENSE](LICENSE). King of the Road is © its rights holders; this project contains
no game code or data.
