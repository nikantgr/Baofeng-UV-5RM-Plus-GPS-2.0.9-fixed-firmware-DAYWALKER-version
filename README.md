# **WARNING! EXPERIMENTAL! USE AT OWN RISK!**

# WATCH VIDEO HERE: https://youtu.be/hUGWN7x2NCk

# Baofeng UV-5RM Plus GPS - DARK & DAYWALKER

Two unofficial, modified versions of firmware v2.0.9. Both contain the same brightness, automatic dimming and APRS Signal Path fixes. Choose the screen appearance you prefer.

**Tested only on my "Baofeng UV-5RM Plus GPS". Compatibility with any other radio or hardware revision has not been tested.** Testing on my radio does not mean every feature has been tested or that these files are bug-free. These are not official Baofeng releases.

## Which file should I use?

| Version | File | Screen appearance |
| --- | --- | --- |
| **DARK** | [BF_5RM-Plus-GPS_v2_0_9_DARK_APRS-FIX-R4_EXPERIMENTAL.dat](BF_5RM-Plus-GPS_v2_0_9_DARK_APRS-FIX-R4_EXPERIMENTAL.dat) | Keeps the original dark background and screen colors. |
| **DAYWALKER** | [BF_5RM-Plus-GPS_v2_0_9_DAYWALKER_APRS-FIX-R4_EXPERIMENTAL.dat](BF_5RM-Plus-GPS_v2_0_9_DAYWALKER_APRS-FIX-R4_EXPERIMENTAL.dat) | A light screen theme designed to improve readability in bright sunlight. |

## Fixes included in BOTH versions

- **Brightness levels work in the correct direction.** Level 1 is the dimmest and level 5 is the brightest. This setting is called **Brightness** on the radio and **Tone Level** in CPS.
- **Automatic screen dimming works again.** The screen dims after the configured timeout and returns to the selected brightness when it wakes.
- **APRS possible fixes**

**Already have a wrong APRS path saved?** Re-enter the correct path after installing the firmware. Existing incorrect values are not repaired automatically. This fixes the path-editing bug; it is not a promise that every APRS function works correctly.

## DAYWALKER screen changes and fixes

- **White background and black lettering** on the main screen and in menus.
- **Green battery indicator.**
- **Light blue backgrounds** for labels and menu selections.
- **Yellow background behind the selected A/B display's name and frequency area**, making the active side easier to identify. The other side stays white.
- **Dark red final signal-meter bars.** The actual bars change color, with a white background behind them.
- **Red volume-bar fill** instead of green.
- **Startup picture fix:** removes the black strip caused by the earlier daylight patch at the top of the boot picture.
- **Scan-screen fix:** keeps the daylight colors and A/B highlight during scanning, instead of switching to a mixed dark/light display.

## Support

You can support my work directly at https://www.white-tree.net/donate (100% goes to me), or buy me a coffee at https://www.buymeacoffee.com/SY1EBE ☕
Every bit helps me keep experimenting, building, and sharing! 🙏

## Risk and responsibility

Flashing experimental firmware can make your radio unusable, erase settings or cause unexpected behavior. Back up your settings and keep a known-working firmware before using these files. Only proceed if you understand how to restore your radio.

These files are provided **AS IS**, without any warranty or guarantee of compatibility, reliability or fitness for a particular purpose. You accept the risks of downloading, flashing and using them. To the maximum extent permitted by applicable law, the project maintainer and contributors accept no liability for damage, data loss or other claims arising from their use. You remain responsible for operating your radio legally.

## License

The project's original modifications and documentation are provided under the **[MIT License](LICENSE)**, which includes warranty and liability disclaimers. See the [official MIT License text](https://opensource.org/license/mit).

The original Baofeng firmware and other third-party material remain subject to their respective owners' rights and terms. This project's license does not grant rights to, or relicense, that material. A license or disclaimer cannot guarantee immunity from liability under applicable law.

**WARNING! EXPERIMENTAL! USE AT OWN RISK!**
