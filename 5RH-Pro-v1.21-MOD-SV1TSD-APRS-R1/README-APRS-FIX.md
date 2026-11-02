# CPS 1.21 MOD SV1TSD — APRS repair R1

**WARNING! EXPERIMENTAL! USE AT OWN RISK!**

Prepared for the Baofeng UV-5RM Plus GPS and our patched v2.0.9 firmware. Offline tests passed; radio read/write testing is still required. The original CPS folder and radio firmware files have not been replaced.

Run **CPS.exe from this folder**. Help → About shows **Revision 1.21 MOD SV1TSD**. Keep the accompanying files and folders together.

## What is fixed

- Callsign padding no longer appears as extra or invisible characters.
- The signal path uses the configured entries, instead of counting empty rows as callsigns. Gaps are compacted when written.
- SSIDs remain in their own fields.
- Upload text stays inside its field; its saved length is calculated from the actual text. The limit is 40 printable ASCII characters, matching the transmitter.
- Blank APRS frequencies remain blank. Frequencies retain leading zeroes and work with Greek regional settings.
- The stored APRS transmit-frequency selection is read and preserved.
- High power no longer writes back as medium on the CPS two-power-level setting.
- Opening the APRS page no longer changes the symbol through initialization events.
- Current cell edits are committed before saving or writing.
- Invalid or uninitialized APRS values are bounded so they can be displayed. Existing corrupt settings cannot be reconstructed automatically: check them after reading.
- Negative fixed altitudes can be displayed and saved. The English beacon choices now say GPS Beacon and Fixed Beacon.
- The unsupported 60-minute automatic-beacon choice is removed; this firmware resets that stored value to OFF.
- Unexposed APRS bytes and reserved fields are preserved where possible.


The existing CSV features and unrelated programming routines are retained.

## How to try it

1. Close the old CPS and start CPS.exe here.
2. Read the radio first, and save a separate backup codeplug.
3. Review the APRS page, especially your own callsign/SSID, path, frequency, beacon source and encoding.
4. Write the radio, then read it again and check that the values stayed correct.

For your example, enter the fields separately:

| Field | Callsign | SSID |
|---|---|---|
| Source | SV1TSD | 7 |
| Target | APRS | 0 |
| Signal path row 1 | WIDE1 | 1 |
| Signal path row 2 | WIDE2 | 1 |

Do not put `-7` in the source callsign box, or a comma-separated path in a single grid cell. The SSID drop-downs display `-1`, `-7`, etc.; select the appropriate item. Callsigns are written with the correct padding automatically. MIC-E generates its own destination field, so its received Target will differ from APRS.

Fixed-position coordinates retain the firmware's format: for example, `37.46120` means 37 degrees 46.120 minutes, not decimal degrees. With Greek regional settings the numeric controls may show a comma as the decimal separator. Choose GPS Beacon to use a GPS fix.

If a radio read still reports an error, look for **CPS-APRS-FIX-errors.log** beside this CPS.exe. The original message hides exceptions from every section, so a remaining error may come from another section; the log records the failing routine. No radio transfer was performed during development, and successful physical readback remains to be confirmed.

**WARNING! EXPERIMENTAL! USE AT OWN RISK!**
