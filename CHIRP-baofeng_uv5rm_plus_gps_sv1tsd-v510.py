# Copyright 2025 Mike Iacovacci <ascendr@linuxmail.org>
#
# Baofeng BF-5RH-Pro (UV-5RH Pro) driver for CHIRP issue #11669
# Based on Devin version 4.0.1; adaptation 5.1.0 MOD SV1TSD (2026-11-03)
#
# CREDITS
#   * Original driver (memory map, settings, channel logic): Mike Iacovacci.
#   * Versions 1-3 were written by earlier contributors to CHIRP issue #11669:
#     Mike Iacovacci (attachment 13201), Jonathan Deeds (attachment 13931)
#     and "unit m" (attachment 15492, the patch for firmware v2_0_09 /
#     hardware v1_0_4 whose handshake code this version builds on).
#   * Version 4.0.1: prompted, directed and hardware-tested by Devin Howe
#     (KB7YZA); code written by Claude, an AI assistant made by Anthropic.
#
# Standalone UV-5RM Plus GPS driver: SV1TSD R4.
# See README and comparison report for verification and hardware limits.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import re
import math
from decimal import Decimal, InvalidOperation
import logging
import random
import struct
import time
from datetime import datetime

from chirp import chirp_common, directory, memmap
from chirp import bitwise, errors
from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueInteger, RadioSettingValueList, \
    RadioSettingValueBoolean, RadioSettingValueString, \
    RadioSettings, RadioSettingValueMap, RadioSettingValueFloat

LOG = logging.getLogger(__name__)

DRIVER_VERSION = "5.1.0 MOD SV1TSD (SV1TSD R4)"

MEM_FORMAT = """
# seekto 0x0050;
struct{
    char        soft_ver[8];
    char        hard_ver[6];
    u8          pad_0x5e_f[2];
    char        last_prog[16];
} dev_info;

# seekto 0x0080;
struct{
    lbcd        rx_freq[4];
    lbcd        tx_freq[4];
    bbcd        rx_tone[2];
    bbcd        tx_tone[2];
    u8          unk_0x0d[4];
    u8          power:2,
                bandwidth:1,
                unk1_0x10:3,
                invert:1,
                talkaround:1;
    u8          pttid_mode:4,
                sqlch_mode:4;
    u8          signaling:4,
                jump_mode:4;
    u8          unk_0x13:2,
                launch_banned:1,
                unk1_0x13:5;
    u8          unk_0x14_18[5];
    u8          dtmf;
    u8          twotone;
    u8          unk_0x1b[1];
    u8          mdc;
    u8          unk_0x1d[1];
    u8          emerg_sys;
    u8          unk_0x1f[1];
    char        name[16];
} memory[640];

# seekto 0x7980;
struct{
    u8          band_a_wrkmode;
    u8          band_b_wrkmode;
    u8          unk_0x82_0x85[4];
    u8          band_a_workzone;
    u8          band_b_workzone;
    u8          backlight_tm;
    u8          tone_lvl;
    u8          band_a_chandisp:4,
                band_b_chandisp:4;
    u8          dbl_waiting;
    u8          main_band;
    u8          squelch;
    u8          vox_lvl;
    u8          vox_dly_detect;
    u8          power_save_off;
    u8          power_save_time;
    u8          lone_wrk_remind;
    u8          lone_wrk_resp;
    u8          apo;
    u8          tot;
    u8          tot_alert;
    u8          unk_0x97;
    u8          gps_zone;
    u8          unk_0x99;
    u8          hz1750;
    u8          unk_0x9b_9d[3];
    u8          noaa_chan;
    u8          unk_0x_9f;
    u8          vox:1,
                aprs:1,
                alone_worker:1,
                falling_alarm:1,
                voice_announce:2,
                unk_0xa0:1,
                busy_lock:1;
    u8          auto_keypad;
    u8          beep:1,
                unk_0xa2:7;
    u8          unk_0xa3;
    u8          gps;
    u8          unk_0xa5_b7:1,
                unk_0xa5_b6:1,
                unk_0xa5_b5:1,
                unk_0xa5_b4:1,
                weather_alert:1,
                language:1,
                power_on_display:2;
    u8          tail_mode:3,
                weather_on:1,
                disp_reverse:1,
                unk_0xa6:2,
                enhance_func:1;
} settings;

# seekto 0x79b0;
struct{
    u8      pf1_short;
    u8      pf2_short;
    u8      pf1_long;
    u8      pf2_long;
}prog_key;

# seekto 0x79c0;
struct{
    u8      power_on_pwd[8];
    u8      program_pwd[8];
    u8      power_on_char[8];
} power_on;

# seekto 0x7a20;
struct{
    lbit      bitfield[640];
} chan_empty;

# seekto 0x7a80;
struct{
    u8          zone_cnt;
} zone_hdr;

# seekto 0x7a90;
struct{
    u8          count;
    u8          pad;
    u8          chan[128];
    u8          pad2[6];
    char        name[16];
} zones[10];

# seekto 0x8102;
struct{
    lbcd        lower_freq[2];
    u8          sc_pad[2];
    lbcd        upper_freq[2];
} scan_freq;

# seekto 0x8180;
struct{
    u8      scan_mode;
    u8      flyback;
    u8      rx_recovery;
    u8      tx_recovery;
    u8      channel_rtn;
    u8      priority;
    u8      unk_0x8186[1];
    u8      prio_scan_chan;
    u8      scan_range;
} scan_menu;

# seekto 0x81a0;
struct{
    lbit      bitfield[640];
} scan_skip;

# seekto 0x8200;
struct{
    u8      dtmf_ani;
    u8      sending_rate;
    u8      first_tm_code;
    u8      precarrier_tm;
    u8      delay_tm;
    u8      ptt_pause_tm;
    u8      dtmf_st;
    u8      auto_reset_tm;
    u8      sepr_opts;
    u8      group_num;
    u8      decode_resp;
    u8      unk_0xb_f[5];
    u8      self_id[3];
    u8      unk_0x13_17[5];
    u8      ptt_id[16];
    u8      ptt_id_offline[16];
    u8      stun[11];
    u8      pad_0x43_47[5];
    u8      kill[11];
}dtmf;

# seekto 0x8260;
struct{
    u8      entry[16];
} dtmf_list[16];

# seekto 0x9e00;
struct{
    char        selfid[6];
    u8          self_ssid;
    u8          self_pad;
    char        targetid[6];
    u8          target_ssid;
    u8          target_pad;
    u8          precarrier_tm;
    u8          code_dly_tm;

} aprs_menu;

# seekto 0x9e18;
struct{
    char        entry[6];
    u8          ssid;
    u8          unused;
} ssid_tbl[8];

# seekto 0xa010;
struct{
    u8          code;
    u8          unused2;
    char        name[14];
} contacts[80];
"""

# Protocol addresses of the eleven 4 KiB blocks that make up the image.
ADDRS = [0x000000, 0x100000, 0x200000, 0x300000,
         0x400000, 0x500000, 0x600000, 0x700000,
         0x800000, 0x900000, 0xa00000]
BLK_SZ = 0x1000
IMAGE_SZ = BLK_SZ * len(ADDRS)

ZONE_MAX = 10       # zones in the radio
ZONE_SLOTS = 64     # memory slots reserved per zone (CHIRP rows)
MEM_COUNT = ZONE_MAX * ZONE_SLOTS
ZONE_NAME_LEN = 16

# From the vendor CPS: 12 x 0x00 followed by 4 x 0xFF.
T_INFO = bytes(12) + bytes([0xFF, 0xFF, 0xFF, 0xFF])
# Default (empty) program password is 8 x 0xFF before the seed XOR.
DEFAULT_PWD = bytes([0xFF] * 8)

VALID_CHARSET = "".join(chr(i) for i in range(32, 127))
DTMF_CHARS = "0123456789ABCD*#"


# --------------------------------------------------------------------------
# Serial protocol
# --------------------------------------------------------------------------



















# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def _cstr(arr):
    """NUL-terminated char array -> printable str (right-stripped)."""
    out = []
    for c in arr:
        ch = str(c)
        if ch == '\x00':
            break
        out.append(ch if ch in VALID_CHARSET else ' ')
    return ''.join(out).rstrip()


def _set_cstr(obj, field, text, size, pad='\x00'):
    text = ''.join(c if c in VALID_CHARSET else ' ' for c in text)[:size]
    setattr(obj, field, list(text.ljust(size, pad)))


def _val_to_int(v):
    """Convert any RadioSettingValue to the integer stored in the image."""
    if isinstance(v, RadioSettingValueMap):
        return int(v.get_mem_val())
    if isinstance(v, RadioSettingValueList):
        return v.get_options().index(v.get_value())
    if isinstance(v, RadioSettingValueBoolean):
        return 1 if v.get_value() else 0
    return int(v.get_value())


def _idx(value, n):
    v = int(value)
    return v if 0 <= v < n else 0


def _add_list(grp, path, label, options, cur):
    rs = RadioSettingValueList(options, current_index=_idx(cur, len(options)))
    grp.append(RadioSetting(path, label, rs))


def _add_map(grp, path, label, opmap, cur):
    vals = [v for _, v in opmap]
    cur = int(cur)
    if cur not in vals:
        cur = vals[0]
    grp.append(RadioSetting(path, label, RadioSettingValueMap(opmap, cur)))


def _add_bool(grp, path, label, cur):
    grp.append(RadioSetting(path, label,
                            RadioSettingValueBoolean(bool(int(cur)))))


def _add_int(grp, path, label, lo, hi, cur, step=1):
    cur = min(max(int(cur), lo), hi)
    grp.append(RadioSetting(
        path, label, RadioSettingValueInteger(lo, hi, cur, step)))


def _add_str(grp, path, label, maxlen, text, charset, callback=None, *args):
    rs = RadioSettingValueString(0, maxlen, text, autopad=False,
                                 charset=charset)
    rset = RadioSetting(path, label, rs)
    if callback:
        rset.set_apply_callback(callback, *args)
    grp.append(rset)


def _dtmf_decode(arr):
    """Accept both 0-15 codes and ASCII; 0xFF ends the string."""
    out = []
    for c in arr:
        v = int(c)
        if v == 0xff:
            break
        if 0 <= v < len(DTMF_CHARS):
            out.append(DTMF_CHARS[v])
        elif chr(v) in "ABCD*#0123456789":
            out.append(chr(v))
        elif chr(v) == 'E':
            out.append('*')
        elif chr(v) == 'F':
            out.append('#')
    return ''.join(out)


def _dtmf_encode(obj, field, text, size):
    vals = [0xff] * size
    n = 0
    for ch in text:
        if ch in DTMF_CHARS and n < size:
            vals[n] = DTMF_CHARS.index(ch)
            n += 1
    setattr(obj, field, vals)


class BF5RHPro(chirp_common.CloneModeRadio):
    """Baofeng BF-5RH Pro GPS"""
    VENDOR = "Baofeng"
    MODEL = "BF-5RH-Pro"
    BAUD_RATE = 19200
    NAME_LEN = 16       # size of the radio's name field (NUL-terminated)
    # Radio menu offers Low / Middle / High (stored as 0 / 1 / 2).  Only
    # Low=0 and High=2 have been seen in real images; Middle=1 is inferred.
    # Wattages are approximate and used by CHIRP for display only.
    POWER_LEVELS = [chirp_common.PowerLevel("Low", watts=1.00),
                    chirp_common.PowerLevel("Middle", watts=5.00),
                    chirp_common.PowerLevel("High", watts=10.00)]
    VALID_MODES = ["NFM", "FM"]
    VALID_BANDS = [(108000000, 174000000), (220000000, 260000000),
                   (350000000, 390000000), (400000000, 520000000)]
    VALID_TONES = (63.0, ) + chirp_common.TONES
    VALID_DCS = (17, 50, 645) + chirp_common.DTCS_CODES
    VALID_CHARSET = VALID_CHARSET
    VALID_DTMF = list(DTMF_CHARS) + [' ']

    WX_CHANNELS = ["WX1 (162.550)", "WX2 (162.400)", "WX3 (162.475)",
                   "WX4 (162.425)", "WX5 (162.450)", "WX6 (162.500)",
                   "WX7 (162.525)", "WX8 (161.650)", "WX9 (161.775)",
                   "WX10 (163.275)"]

    SSID_OPMAP = [('None', 255)] + [('0', 0)] + \
        [('-%d' % i, i) for i in range(1, 16)]

    # ------------------------------------------------------------------
    # CHIRP plumbing
    # ------------------------------------------------------------------
    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_bank = False
        rf.has_tuning_step = False
        # Memory frequencies are stored in 10 Hz units, so 12.5 kHz-offset
        # channels (e.g. GMRS 462.5625) are valid.  Tell CHIRP not to reject
        # them when pasting from other radios.
        rf.valid_tuning_steps = [2.5, 5.0, 6.25, 10.0, 12.5, 20.0, 25.0,
                                 50.0, 100.0]
        try:
            rf.has_nostep_tuning = True
        except Exception:
            pass        # attribute not present in this CHIRP version
        rf.has_name = True
        rf.valid_name_length = self.NAME_LEN
        rf.valid_characters = self.VALID_CHARSET
        rf.has_offset = True
        rf.has_mode = True
        rf.has_dtcs = True
        rf.has_rx_dtcs = True
        rf.has_dtcs_polarity = True
        rf.has_ctone = True
        rf.has_cross = True
        rf.valid_tones = self.VALID_TONES
        rf.valid_dtcs_codes = self.VALID_DCS
        rf.can_odd_split = False
        rf.can_delete = True
        rf.valid_modes = self.VALID_MODES
        rf.valid_duplexes = ["", "-", "+", "off"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = [
            "Tone->DTCS", "DTCS->Tone", "->Tone", "Tone->Tone",
            "->DTCS", "DTCS->", "DTCS->DTCS"]
        rf.valid_power_levels = self.POWER_LEVELS
        rf.valid_skips = ["", "S"]
        rf.valid_bands = self.VALID_BANDS
        rf.memory_bounds = (1, MEM_COUNT)
        return rf

    def validate_memory(self, mem):
        # The radio stores frequencies in 10 Hz units and accepts 6.25 / 12.5
        # kHz-offset channels (e.g. GMRS 462.5625).  CHIRP's generic
        # tuning-step check rejects them when pasting from other radios, so
        # drop only those messages and keep every other validation result.
        msgs = super().validate_memory(mem)
        return [m for m in msgs
                if 'tuning step' not in str(m).lower()
                and 'khz step' not in str(m).lower()]

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    def sync_in(self):
        try:
            data = do_download(self)
        except Exception as e:
            err = f'Error during download {e}'
            LOG.error(err)
            raise errors.RadioError(err)
        self._mmap = data
        self.process_mmap()


    @classmethod
    def match_model(cls, filedata, filename):
        # new drivers identified by metadata
        return False

    # ------------------------------------------------------------------
    # Memories
    # ------------------------------------------------------------------
    def get_memory(self, number):
        mem = chirp_common.Memory()
        mem.number = number
        _mem = self._memobj.memory[number - 1]
        mem.freq = int(_mem.rx_freq) * 10
        if self.get_empty(number - 1):
            mem.empty = True
            return mem

        mem.name = _cstr(_mem.name)
        mem.power = self.POWER_LEVELS[_idx(_mem.power, len(self.POWER_LEVELS))]
        mem.mode = self.VALID_MODES[_mem.bandwidth]
        mem.skip = 'S' if self.get_skip(number - 1) else ""

        rx, tx = int(_mem.rx_freq), int(_mem.tx_freq)
        if tx == rx:
            mem.duplex = ''
            mem.offset = 0
        elif tx == 0:
            mem.duplex = 'off'
            mem.offset = 0
        elif tx > rx:
            mem.duplex = '+'
            mem.offset = (tx - rx) * 10
        else:
            mem.duplex = '-'
            mem.offset = (rx - tx) * 10

        self.get_tones(_mem, mem)

        mem.extra = RadioSettingGroup("Extra", "extra")
        _add_bool(mem.extra, "invert", "TX Invert", _mem.invert)
        _add_bool(mem.extra, "talkaround", "Talkaround", _mem.talkaround)
        _add_map(mem.extra, "pttid_mode", "PTT/5-Tone ID",
                 [('Off', 0), ('PTT BOT', 1), ('PTT EOT', 2),
                  ('PTT Both', 3), ('5-Tone BOT', 4),
                  ('5-Tone EOT', 8), ('5-Tone Both', 12)], _mem.pttid_mode)
        _add_map(mem.extra, "sqlch_mode", "Squelch Mode",
                 [('Off', 0), ('CTDCS', 1), ('Optional', 2), ('Both', 3)],
                 _mem.sqlch_mode)
        _add_map(mem.extra, "signaling", "Signaling",
                 [('Off', 0), ('DTMF', 2), ('2-Tone', 4), ('5-Tone', 6),
                  ('MDC', 8), ('BIIS', 12)], _mem.signaling)
        _add_map(mem.extra, "jump_mode", "Jump Freq.",
                 [('Off', 0), ('Normal', 1), ('Strict', 2)], _mem.jump_mode)
        _add_bool(mem.extra, "launch_banned", "Launch Banned",
                  _mem.launch_banned)
        _add_list(mem.extra, "dtmf", "DTMF Index",
                  [str(i) for i in range(16 + 1)], _mem.dtmf)
        _add_list(mem.extra, "twotone", "2-Tone Index",
                  [str(i) for i in range(16 + 1)], _mem.twotone)
        _add_list(mem.extra, "mdc", "MDC Index",
                  [str(i) for i in range(5 + 1)], _mem.mdc)
        _add_list(mem.extra, "emerg_sys", "Emergency System",
                  [str(i) for i in range(10 + 1)], _mem.emerg_sys)
        return mem

    def set_memory(self, mem):
        idx = mem.number - 1
        _mem = self._memobj.memory[idx]

        if mem.empty:
            self.set_empty(idx, 1)
            self.set_skip(idx, 0)
            _mem.fill_raw(b'\x00')
            return

        if self.get_empty(idx):
            _mem.fill_raw(b'\x00')      # new channel: start from clean record
            self.set_empty(idx, 0)

        _mem.rx_freq = mem.freq // 10

        if mem.duplex == '+':
            _mem.tx_freq = (mem.freq + mem.offset) // 10
        elif mem.duplex == '-':
            _mem.tx_freq = (mem.freq - mem.offset) // 10
        elif mem.duplex == 'off':
            _mem.tx_freq = 0
        else:
            _mem.tx_freq = mem.freq // 10

        _set_cstr(_mem, 'name', mem.name.rstrip(), 16)
        self.set_skip(idx, 1 if mem.skip == 'S' else 0)
        _mem.bandwidth = self.VALID_MODES.index(mem.mode)
        _mem.power = (self.POWER_LEVELS.index(mem.power)
                      if mem.power in self.POWER_LEVELS else 2)
        self.set_tone(_mem, mem)

        for setting in mem.extra:
            setattr(_mem, setting.get_name(), _val_to_int(setting.value))

    def get_tones(self, _mem, mem):
        def decode(el):
            val = int(el)
            msb = el.get_raw()[0]
            if val == 16665 or val == 0:
                return ("", 0, None)
            if 0x80 <= msb < 0xc0:
                return ("DTCS", val % 8000, "N")
            if msb >= 0xc0:
                return ("DTCS", val % 12000, "R")
            return ("Tone", val / 10, None)
        chirp_common.split_tone_decode(
            mem, decode(_mem.tx_tone), decode(_mem.rx_tone))

    def set_tone(self, _mem, mem):
        ((txmode, txval, txpol),
         (rxmode, rxval, rxpol)) = chirp_common.split_tone_encode(mem)

        def encode(el, mode, val, pol):
            if mode == "":
                # factory images use 0x0000 for "no tone"; 0xFFFF also works.
                # Leave an existing "none" alone, otherwise write 0xFFFF.
                if int(el) not in (0, 16665):
                    el.set_raw(b'\xff\xff')
            elif mode == "Tone":
                el.set_value(int(val * 10))
            elif mode == "DTCS" and pol == "N":
                el.set_value(int(val + 8000))
            elif mode == "DTCS" and pol == "R":
                el.set_raw(bytes([(val // 100) + 0xc0,
                                  int("%02i" % (val % 100), 16)]))
        encode(_mem.tx_tone, txmode, txval, txpol)
        encode(_mem.rx_tone, rxmode, rxval, rxpol)

    def get_skip(self, chan_n):
        return self._memobj.scan_skip['bitfield'][chan_n].get_value()

    def set_skip(self, chan_n, value):
        self._memobj.scan_skip['bitfield'][chan_n].set_value(value)

    def get_empty(self, chan_n):
        return self._memobj.chan_empty['bitfield'][chan_n].get_value()

    def set_empty(self, chan_n, value):
        self._memobj.chan_empty['bitfield'][chan_n].set_value(value)

    # ------------------------------------------------------------------
    # Settings
    # ------------------------------------------------------------------

    def _g_info(self, top):
        di = self._memobj.dev_info
        grp = RadioSettingGroup("info", "Device Info")
        top.append(grp)
        for name, label, arr in (
                ("soft_ver", "Software Version", di.soft_ver),
                ("hard_ver", "Hardware Version", di.hard_ver),
                ("last_prog", "Last Programmed", di.last_prog)):
            rs = RadioSettingValueString(0, 16, _cstr(arr), autopad=False,
                                         charset=VALID_CHARSET)
            rs.set_mutable(False)
            grp.append(RadioSetting("dev_info." + name, label, rs))
        rs = RadioSettingValueString(0, 64, DRIVER_VERSION, autopad=False,
                                     charset=VALID_CHARSET)
        rs.set_mutable(False)
        grp.append(RadioSetting("driver_version", "Driver Version", rs))





















# Integration fragment for the UV-5RM Plus GPS R4 driver.
# Replaces Devin's protocol functions; adds actual named, indexed MTO banks.
# Imports expected: logging/random/struct/time/datetime; CHIRP common/errors/memmap.
# Firmware evidence: native clone handler 0x08018748..0x08018BB2.
# Native mode R permits a read preflight and subsequent W headers enter write mode.
# Uses 11 known 4-KiB blocks; preserves every byte not explicitly edited.

PROTECTED_PREFIX = 0x80
EXPECTED_VERSION_TAG = b'v2009-SV'
EXPECTED_HARDWARE_TAG = b'v1_0_4\xff\xff'


def _read_exact(serial, count, what):
    deadline = time.monotonic() + max(float(getattr(serial, 'timeout', 5) or 5), 0.5)
    result = bytearray()
    while len(result) < count:
        part = serial.read(count - len(result))
        if part:
            result.extend(part)
        elif time.monotonic() >= deadline:
            raise errors.RadioError('%s: received %d of %d bytes' %
                                    (what, len(result), count))
        else:
            time.sleep(0.005)
    return bytes(result)


def raw_send(serial, data, exlen):
    if data:
        serial.write(data)
    return _read_exact(serial, exlen, 'Radio response')


def _expect_ack(resp, xor, what):
    if resp != bytes([0x41 ^ xor]):
        raise errors.RadioError('%s: expected acknowledgement, got %r' %
                                (what, resp))


def wakeup(serial, xor):
    serial.timeout = 0.75
    serial.write(T_INFO)
    try:
        ack = _read_exact(serial, 1, 'Wake-up')
    except errors.RadioError:
        ack = b''
    if ack != b'A':
        # Same alternate-baud entry as Devin's hardware-tested implementation.
        serial.close()
        serial.baudrate = 115200
        serial.open()
        for name in ('dtr', 'rts'):
            try:
                setattr(serial, name, True)
            except (AttributeError, OSError):
                pass
        if hasattr(serial, 'reset_input_buffer'):
            serial.reset_input_buffer()
        time.sleep(0.1)
    serial.timeout = 5.0
    serial.write(b'PROGRAM' + bytes([xor]))


def do_init(serial, xor, password=None):
    _expect_ack(raw_send(serial, b'', 1), xor, 'PROGRAM')
    pwd = DEFAULT_PWD if password is None else bytes(password)
    if len(pwd) != 8:
        raise errors.RadioError('Programming password must occupy eight bytes')
    _expect_ack(raw_send(serial, bytes(v ^ xor for v in pwd), 1), xor,
                'Programming password')
    identity = bytes(v ^ xor for v in raw_send(
        serial, bytes(v ^ xor for v in b'INFORMATION'), 16))
    if identity[:6] != b'BF-5RH':
        raise errors.RadioError('Wrong radio family: %r; expected BF-5RH' %
                                identity[:8])
    # Retain Devin 4.0.1's entry byte for both operations. Upload sessions
    # send write blocks immediately after entry, without interleaved reads.
    _expect_ack(raw_send(serial, bytes([xor ^ ord('R')]), 1), xor,
                'Programming mode')
    return identity


def read_block(serial, xor, addr, blk_len):
    if blk_len != BLK_SZ:
        raise errors.RadioError('This firmware transfers 4096-byte blocks')
    cmd = struct.pack('>I', 0x52000000 + addr)
    resp = raw_send(serial, bytes(v ^ xor for v in cmd), blk_len + 4)
    expected = bytes(v ^ xor for v in struct.pack('>I', 0x57000000 + addr))
    if resp[:4] != expected:
        raise errors.RadioError('Block address echo mismatch at 0x%06X' % addr)
    return resp[4:]


def write_block(serial, xor, addr, data, blk_len):
    if blk_len != BLK_SZ or len(data) != blk_len:
        raise errors.RadioError('Refusing an incomplete write block')
    header = bytes(v ^ xor for v in struct.pack('>I', 0x57000000 + addr))
    _expect_ack(raw_send(serial, header + bytes(data), 1), xor,
                'Write block 0x%06X' % addr)
    return blk_len


def exit_prog(serial, xor, strict=False):
    try:
        _expect_ack(raw_send(serial, bytes(v ^ xor for v in b'END\x00'), 1),
                    xor, 'End programming')
    except Exception:
        if strict:
            raise
        LOG.debug('Best-effort programming cleanup failed', exc_info=True)


def _validate_target_prefix(prefix):
    if len(prefix) < PROTECTED_PREFIX:
        raise errors.RadioError('Incomplete target radio identification data')
    expected = EXPECTED_VERSION_TAG
    if prefix[0x50:0x58] != expected:
        raise errors.RadioError(
            'Selected firmware profile does not match radio tag %r. '
            'No data was written.' % prefix[0x50:0x58])
    if prefix[0x58:0x60] != EXPECTED_HARDWARE_TAG:
        raise errors.RadioError(
            'Unexpected radio hardware version %r. No data was written.' %
            prefix[0x58:0x60])


def do_download(radio):
    serial = radio.pipe
    xor = random.randint(1, 255)
    complete = False
    try:
        serial.timeout = 5.0
        status = chirp_common.Status()
        status.cur, status.max = 0, IMAGE_SZ
        status.msg = 'Connecting to radio...'
        radio.status_fn(status)
        wakeup(serial, xor)
        do_init(serial, xor, getattr(radio, '_program_password', None))
        chunks = []
        for addr in ADDRS:
            block = bytes(v ^ xor for v in read_block(serial, xor, addr, BLK_SZ))
            if not chunks:
                _validate_target_prefix(block)
            chunks.append(block)
            status.cur += len(block)
            status.msg = 'Downloading...'
            radio.status_fn(status)
        exit_prog(serial, xor, strict=True)
        complete = True
        return memmap.MemoryMapBytes(b''.join(chunks))
    except errors.RadioError:
        raise
    except Exception as exc:
        raise errors.RadioError('Download failed: %s' % exc) from exc
    finally:
        if not complete:
            exit_prog(serial, xor)


def do_upload(radio):
    serial = radio.pipe
    xor = random.randint(1, 255)
    complete = False
    try:
        if len(radio._mmap) != IMAGE_SZ:
            raise errors.RadioError('Image must be exactly %d bytes' % IMAGE_SZ)
        payload = bytearray(radio._mmap.get_packed())
        if hasattr(radio, '_validate_upload_image'):
            radio._validate_upload_image(payload)
        status = chirp_common.Status()
        status.cur, status.max = 0, IMAGE_SZ
        _validate_target_prefix(payload)
        status.msg = 'Connecting to radio...'
        radio.status_fn(status)
        serial.timeout = 5.0
        wakeup(serial, xor)
        do_init(serial, xor, getattr(radio, '_program_password', None))
        # Match the hardware-tested 4.0.1 upload command order: R mode entry,
        # then W blocks. Read preflights are not part of this sequence.
        # Check the saved image's tag before entry; zone edits are compared
        # with the image baseline captured when it was downloaded/opened.
        payload[0x60:0x70] = datetime.now().strftime('%Y.%m.%d %H:%M').encode('ascii')
        status.msg = 'Uploading...'
        for i, addr in enumerate(ADDRS):
            plain = payload[BLK_SZ * i:BLK_SZ * (i + 1)]
            encoded = bytes(v ^ xor for v in plain)
            status.cur += write_block(serial, xor, addr, encoded, BLK_SZ)
            radio.status_fn(status)
        exit_prog(serial, xor, strict=True)
        complete = True
    except errors.RadioError:
        raise
    except Exception as exc:
        raise errors.RadioError('Upload failed: %s' % exc) from exc
    finally:
        if not complete:
            exit_prog(serial, xor)


# ---------------------------------------------------------------------------
# Zone model: radio channels are independent slots; zones are ordered lists.
# Preserve the actual stored memberships, including shared channels and names.
# Expected integration: rf.has_bank=True, has_bank_names=True,
# has_bank_index=True; radio.get_bank_model returns RadioZoneModel(self).
# Remove _rebuild_zones call from sync_out. No automatic upload-time regrouping.
# In set_memory: new occupied channel -> assign_new_memory(number);
# deletion -> remove_deleted_memory(number). Preserve transaction rollback if
# a new channel cannot be placed because all ten zone lists are full.
# ---------------------------------------------------------------------------


def _zone_bytes(radio, zone):
    return bytes(radio._mmap[0x7A90 + zone * 0x98:0x7A90 + (zone + 1) * 0x98])


def _zone_write_bytes(radio, offset, data):
    radio._mmap[offset] = bytes(data)


def _zone_members(radio, zone):
    raw = _zone_bytes(radio, zone)
    count = raw[0] if raw[0] <= ZONE_SLOTS else 0
    return [slot for slot in struct.unpack('>64H', raw[2:130])[:count]
            if slot < MEM_COUNT]


def _zone_name(radio, zone):
    raw = _zone_bytes(radio, zone)[0x88:0x98]
    raw = raw.split(b'\x00', 1)[0].split(b'\xff', 1)[0]
    try:
        return raw.decode('gb2312') or 'Zone %d' % (zone + 1)
    except UnicodeDecodeError:
        return raw.decode('ascii', errors='replace') or 'Zone %d' % (zone + 1)


def _zone_set_name(radio, zone, name):
    try:
        data = name.encode('gb2312')
    except UnicodeEncodeError as exc:
        raise errors.RadioError('Zone name is not supported by the radio') from exc
    if len(data) > ZONE_NAME_LEN or b'\x00' in data or b'\xff' in data:
        raise errors.RadioError('Zone names must fit in 16 encoded bytes')
    _zone_write_bytes(radio, 0x7A90 + zone * 0x98 + 0x88,
                      data.ljust(ZONE_NAME_LEN, b'\x00'))


def _zone_set_members(radio, zone, members):
    if len(members) > ZONE_SLOTS or any(not 0 <= i < MEM_COUNT for i in members):
        raise errors.RadioError('Zone accepts at most 64 valid memory slots')
    if len(set(members)) != len(members):
        raise errors.RadioError('The same memory cannot appear twice in one zone')
    raw = bytearray(_zone_bytes(radio, zone))
    raw[0] = len(members)
    raw[2:130] = struct.pack('>64H', *(list(members) +
                                     [0xFFFF] * (ZONE_SLOTS - len(members))))
    # Leave padding and existing names unchanged on membership edits.
    _zone_write_bytes(radio, 0x7A90 + zone * 0x98, raw)
    # Do not compact logical bank numbers when an earlier zone becomes empty.
    count = max([z + 1 for z in range(ZONE_MAX) if _zone_members(radio, z)] or [0])
    _zone_write_bytes(radio, 0x7A80, bytes([count]))


class RadioNamedZone(chirp_common.NamedBank):
    def get_name(self):
        return _zone_name(self._model._radio, self._index)

    def set_name(self, name):
        _zone_set_name(self._model._radio, self._index, name)


class RadioZoneModel(chirp_common.MTOBankModel,
                  chirp_common.MappingModelIndexInterface):
    def __init__(self, radio):
        super().__init__(radio, name='Zones')
        self._banks = [RadioNamedZone(self, z, 'Zone %d' % (z + 1))
                       for z in range(ZONE_MAX)]

    def get_num_mappings(self):
        return ZONE_MAX

    def get_mappings(self):
        return self._banks

    def _zone(self, mapping):
        index = int(mapping.get_index())
        if not 0 <= index < ZONE_MAX:
            raise errors.RadioError('Invalid zone number')
        return index

    def add_memory_to_mapping(self, memory, mapping):
        slot, zone = memory.number - 1, self._zone(mapping)
        if self._radio.get_empty(slot):
            raise errors.RadioError('An empty memory cannot be added to a zone')
        members = _zone_members(self._radio, zone)
        if slot not in members:
            _zone_set_members(self._radio, zone, members + [slot])

    def remove_memory_from_mapping(self, memory, mapping):
        slot, zone = memory.number - 1, self._zone(mapping)
        members = _zone_members(self._radio, zone)
        if slot not in members:
            raise errors.RadioError('Memory is not in this zone')
        _zone_set_members(self._radio, zone, [i for i in members if i != slot])

    def get_mapping_memories(self, mapping):
        return [self._radio.get_memory(slot + 1)
                for slot in _zone_members(self._radio, self._zone(mapping))]

    def get_memory_mappings(self, memory):
        return [bank for bank in self._banks
                if memory.number - 1 in _zone_members(self._radio, bank.get_index())]

    def get_index_bounds(self):
        return 0, ZONE_SLOTS - 1

    def get_memory_index(self, memory, mapping):
        members = _zone_members(self._radio, self._zone(mapping))
        try:
            return members.index(memory.number - 1)
        except ValueError as exc:
            raise errors.RadioError('Memory is not in this zone') from exc

    def set_memory_index(self, memory, mapping, index):
        zone = self._zone(mapping)
        members = _zone_members(self._radio, zone)
        slot = memory.number - 1
        if slot not in members or not 0 <= int(index) < len(members):
            raise errors.RadioError('Zone order must be a position in its existing list')
        old = members.index(slot)
        members.pop(old)
        members.insert(int(index), slot)
        _zone_set_members(self._radio, zone, members)

    def get_next_mapping_index(self, mapping):
        count = len(_zone_members(self._radio, self._zone(mapping)))
        if count >= ZONE_SLOTS:
            raise errors.RadioError('Zone is full')
        return count

    def assign_new_memory(self, number):
        slot = number - 1
        if any(slot in _zone_members(self._radio, z) for z in range(ZONE_MAX)):
            return
        settings = self._radio._memobj.settings
        selected = int(settings.band_b_workzone if int(settings.main_band)
                       else settings.band_a_workzone)
        candidates = ([selected] if 0 <= selected < ZONE_MAX else [])
        candidates += [z for z in range(ZONE_MAX) if z not in candidates]
        for zone in candidates:
            members = _zone_members(self._radio, zone)
            if len(members) < ZONE_SLOTS:
                if not members and not _zone_bytes(self._radio, zone)[0x88:0x98].strip(b'\x00\xff '):
                    _zone_set_name(self._radio, zone, 'Zone %d' % (zone + 1))
                _zone_set_members(self._radio, zone, members + [slot])
                return
        raise errors.RadioError('All ten zone lists are full; remove a zone membership first')

    def remove_deleted_memory(self, number):
        slot = number - 1
        for zone in range(ZONE_MAX):
            members = _zone_members(self._radio, zone)
            if slot in members:
                _zone_set_members(self._radio, zone, [i for i in members if i != slot])

# Paste into the standalone driver before its radio class, and inherit this
# mixin. Source evidence: archived CPS DataProtocol.cs read/write pairs; Devin
# hardware observations retained for verified presentation labels.
# Uses imports already in Devin's driver: struct, chirp_common, RadioSetting,
# RadioSettingGroup, RadioSettingValue{Integer,List,Boolean,String}, RadioSettings.

class NonAPRSSettingsMixin:
    """Changed-only EEPROM settings. No guessed writable unknown bits."""
    _CFG_ASCII = ''.join(chr(i) for i in range(32, 127))
    _CFG_CODES = '0123456789ABCD*#'
    _CFG_TEXT_CHARSET = ''.join(chr(i) for i in range(32, 0x10000)
                               if not (127 <= i < 160 or 0xd800 <= i <= 0xdfff))
    _CFG_STANDARDS = ['ZVEI1', 'ZVEI2', 'ZVEI3', 'PZVEI', 'DZVEI',
                     'PDZVEI', 'CCIR1', 'CCIR2', 'PCCIR', 'EEA',
                     'EURO', 'NATEL', 'MODAT', 'CCITT']

    def _cfg_bytes(self, off, size=1):
        return bytes(self._mmap.get_packed()[off:off + size])

    def _cfg_write(self, off, data):
        data = bytes(data)
        if off < 0 or off + len(data) > len(self._mmap.get_packed()):
            raise ValueError('Setting write outside radio image')
        self._mmap[off] = data

    def _cfg_n(self, off, size=1, endian='big'):
        return int.from_bytes(self._cfg_bytes(off, size), endian)

    def _cfg_group(self, parent, name, label):
        group = RadioSettingGroup('cfg.' + name, label)
        parent.append(group)
        return group

    def _cfg_apply_number(self, setting, off, size, mask, shift,
                         scale, add, lo, hi, endian):
        shown = int(setting.value)
        if shown < lo or shown > hi or (shown - add) % scale:
            raise ValueError('Setting is outside its supported range or step')
        encoded = (shown - add) // scale
        if encoded < 0 or encoded > (mask >> shift):
            raise ValueError('Setting does not fit stored field')
        old = self._cfg_n(off, size, endian)
        value = (old & ~mask) | ((encoded << shift) & mask)
        self._cfg_write(off, value.to_bytes(size, endian))

    def _cfg_num(self, group, name, label, off, lo, hi, size=1,
                mask=None, shift=0, scale=1, add=0, step=1, endian='big'):
        mask = ((1 << (8 * size)) - 1) if mask is None else mask
        current = ((self._cfg_n(off, size, endian) & mask) >> shift) * scale + add
        current = max(lo, min(hi, current))
        current = lo + ((current - lo) // step) * step
        rs = RadioSetting('cfg.' + name, label,
                          RadioSettingValueInteger(lo, hi, current, step))
        rs.set_apply_callback(self._cfg_apply_number, off, size, mask, shift,
                              scale, add, lo, hi, endian)
        group.append(rs)
        return rs

    def _cfg_apply_choice(self, setting, off, size, mask, shift, values, endian):
        index = setting.value.get_options().index(str(setting.value))
        encoded = values[index]
        old = self._cfg_n(off, size, endian)
        self._cfg_write(off, ((old & ~mask) | ((encoded << shift) & mask)).to_bytes(size, endian))

    def _cfg_choice(self, group, name, label, off, choices, size=1,
                   mask=None, shift=0, endian='big'):
        pairs = [(str(label), value) for label, value in choices]
        values = [value for label, value in pairs]
        current = (self._cfg_n(off, size, endian) &
                   (((1 << (size * 8)) - 1) if mask is None else mask)) >> shift
        labels = [label for label, value in pairs]
        # Invalid stored values are presented explicitly and remain unchanged.
        if current not in values:
            labels.append('Stored unsupported value (%d)' % current)
            values.append(current)
        index = values.index(current)
        mask = ((1 << (8 * size)) - 1) if mask is None else mask
        rs = RadioSetting('cfg.' + name, label,
                          RadioSettingValueList(labels, current_index=index))
        rs.set_apply_callback(self._cfg_apply_choice, off, size, mask, shift,
                              values, endian)
        group.append(rs)
        return rs

    def _cfg_apply_bit(self, setting, off, bit, inverted):
        value = bool(setting.value) ^ inverted
        old = self._cfg_n(off)
        self._cfg_write(off, bytes([(old & ~(1 << bit)) | (int(value) << bit)]))

    def _cfg_bit(self, group, name, label, off, bit=0, inverted=False):
        rs = RadioSetting('cfg.' + name, label, RadioSettingValueBoolean(
            bool(self._cfg_n(off) & (1 << bit)) ^ inverted))
        rs.set_apply_callback(self._cfg_apply_bit, off, bit, inverted)
        group.append(rs)
        return rs

    def _cfg_decode_text(self, off, size, codec):
        raw = self._cfg_bytes(off, size).split(b'\xff', 1)[0].split(b'\0', 1)[0]
        # Full Unicode display where GB2312 is valid. For malformed text, show
        # a blank editable replacement but retain original bytes when untouched.
        try:
            text = raw.decode(codec)
            return text if all(ord(c) >= 32 and not 127 <= ord(c) < 160 for c in text) else ''
        except UnicodeDecodeError:
            return ''

    def _cfg_apply_text(self, setting, off, size, pad, codec):
        text = str(setting.value)
        raw = text.encode(codec)
        if len(raw) > size:
            raise ValueError('Text exceeds %d encoded bytes' % size)
        self._cfg_write(off, raw.ljust(size, bytes([pad])))

    def _cfg_text(self, group, name, label, off, size, pad=0, codec='ascii'):
        text = self._cfg_decode_text(off, size, codec)
        # Shared printable BMP charset permits GB2312 editing; callback validates
        # actual encodability and byte count before any write.
        rs = RadioSetting('cfg.' + name, label, RadioSettingValueString(
            0, size, text, autopad=False, charset=(self._CFG_TEXT_CHARSET if codec == 'gb2312' else self._CFG_ASCII)))
        rs.set_apply_callback(self._cfg_apply_text, off, size, pad, codec)
        group.append(rs)
        return rs

    def _cfg_apply_frequency(self, setting, off, allow_blank):
        from decimal import Decimal, InvalidOperation
        text = str(setting.value).strip().replace(',', '.')
        if not text and allow_blank:
            self._cfg_write(off, b'\xff' * 4)
            return
        try:
            scaled = Decimal(text) * 100000
        except InvalidOperation:
            raise ValueError('Enter a frequency in MHz')
        if not scaled.is_finite() or scaled != scaled.to_integral_value() or not 0 <= scaled < 100000000:
            raise ValueError('Frequency must fit 10 Hz precision and 8 BCD digits')
        digits = '%08d' % int(scaled)
        self._cfg_write(off, bytes.fromhex(digits)[::-1])

    def _cfg_freq(self, group, name, label, off, allow_blank=True):
        raw = self._cfg_bytes(off, 4)
        digits = raw[::-1].hex()
        text = ''
        if raw not in (b'\xff' * 4, b'\0' * 4) and all(c in '0123456789' for c in digits):
            text = digits[:3].lstrip('0') + '.' + digits[3:]
            if text.startswith('.'):
                text = '0' + text
        rs = RadioSetting('cfg.' + name, label, RadioSettingValueString(
            0, 10, text, autopad=False, charset='0123456789.,'))
        rs.set_apply_callback(self._cfg_apply_frequency, off, allow_blank)
        group.append(rs)
        return rs

    def _cfg_code_read(self, off, size, packed=False, length_off=None):
        raw = self._cfg_bytes(off, size)
        if packed:
            vals = [n for b in raw for n in (b >> 4, b & 15)]
            count = min(self._cfg_n(length_off), size * 2)
            return ''.join(self._CFG_CODES[v] for v in vals[:count])
        vals = []
        for b in raw:
            if b > 15:
                break
            vals.append(b)
        return ''.join(self._CFG_CODES[v] for v in vals)

    def _cfg_apply_code(self, setting, off, size, packed, length_off, bitmap, index, max_chars):
        text = str(setting.value).upper()
        if any(c not in self._CFG_CODES for c in text) or len(text) > max_chars:
            raise ValueError('Invalid signaling code')
        vals = [self._CFG_CODES.index(c) for c in text]
        if packed:
            # Length is explicit; unused trailing nibble is F, as in CPS.
            vals += [15] * (size * 2 - len(vals))
            raw = bytes((vals[i] << 4) | vals[i + 1] for i in range(0, len(vals), 2))
            self._cfg_write(length_off, bytes([len(text)]))
        else:
            raw = bytes(vals).ljust(size, b'\xff')
        self._cfg_write(off, raw)
        if bitmap is not None:
            at = bitmap + index // 8
            value = self._cfg_n(at)
            self._cfg_write(at, bytes([(value | (1 << (index % 8))) if not text
                                      else (value & ~(1 << (index % 8)))]))

    def _cfg_code(self, group, name, label, off, size, packed=False,
                 length_off=None, bitmap=None, index=0, max_chars=None):
        limit = size * (2 if packed else 1) if max_chars is None else max_chars
        text = self._cfg_code_read(off, size, packed, length_off)
        if bitmap is not None and self._cfg_n(bitmap + index // 8) & (1 << (index % 8)):
            text = ''
        rs = RadioSetting('cfg.' + name, label, RadioSettingValueString(
            0, limit, text[:limit], autopad=False, charset=self._CFG_CODES))
        rs.set_apply_callback(self._cfg_apply_code, off, size, packed, length_off,
                              bitmap, index, limit)
        group.append(rs)
        return rs

    def _g_cfg_non_aprs(self, top):
        self._cfg_general(top)
        self._cfg_scan(top)
        self._cfg_keys(top)
        self._cfg_dtmf(top)
        self._cfg_two_tone(top)
        self._cfg_five_tone(top)
        self._cfg_mdc(top)
        self._cfg_emergency(top)
        self._cfg_contacts(top)
        self._cfg_vfo(top)

    def _cfg_weather(self, group):
        enabled = bool(self._cfg_n(0x79a6) & 16)
        channel = self._cfg_n(0x799e)
        choices = ['Off'] + ['WX%d' % i for i in range(1, 11)]
        index = channel + 1 if enabled and channel < 10 else 0
        setting = RadioSetting('r4.weather', 'NOAA weather (Off / channel)',
                               RadioSettingValueList(choices, current_index=index))
        setting.set_apply_callback(self._cfg_apply_weather)
        group.append(setting)

    def _cfg_apply_weather(self, setting):
        index = setting.value.get_options().index(str(setting.value))
        value = self._cfg_n(0x79a6)
        if index:
            self._cfg_write(0x799e, bytes([index - 1]))
            self._cfg_write(0x79a6, bytes([value | 16]))
        else:
            self._cfg_write(0x79a6, bytes([value & ~16]))

    def _cfg_general(self, top):
        g = self._cfg_group(top, 'general', 'Radio Settings')
        ch = self._cfg_choice; nu = self._cfg_num; bi = self._cfg_bit
        ch(g, 'brightness', 'Brightness (1 dimmest - 5 brightest)', 0x7989,
           [(str(i + 1), i) for i in range(5)])
        ch(g, 'backlight', 'Backlight timeout (s)', 0x7988,
           [('Always', 0)] + [(str(i), i) for i in range(5, 31)])
        nu(g, 'squelch', 'Squelch', 0x798d, 0, 9)
        bi(g, 'vox', 'VOX enabled', 0x79a0, 7)
        nu(g, 'vox_level', 'VOX level', 0x798e, 1, 9)
        nu(g, 'vox_delay', 'VOX delay (ms)', 0x798f, 1000, 10000,
           scale=100, step=500)
        for i, bank in enumerate('AB'):
            ch(g, 'mode_' + bank, bank + ' mode', 0x7980 + i,
               [('VFO', 0), ('MR', 1)])
            nu(g, 'channel_' + bank, bank + ' current zone position (1-64)', 0x7982 + 2 * i,
               1, 64, size=2, add=1)
            nu(g, 'zone_' + bank, bank + ' current zone (1-10)', 0x7986 + i,
               1, 10, add=1)
            ch(g, 'display_' + bank, bank + ' channel display', 0x798a,
               [('Frequency', 0), ('Name', 1), ('Number', 2), ('Frequency + name', 3)],
               mask=(0xf0 if i == 0 else 0x0f), shift=(4 if i == 0 else 0))
        ch(g, 'dual', 'Dual Watch', 0x798b,
           [('Off', 0), ('Double Wait', 1), ('Signal Wait', 2)])
        ch(g, 'main', 'Main band', 0x798c, [('A', 0), ('B', 1)])
        ch(g, 'power_save', 'Battery saving ratio', 0x7990,
           [('Off', 0), ('1:1', 1), ('1:2', 2), ('1:4', 3)])
        ch(g, 'power_save_delay', 'Battery saving delay (s)', 0x7991,
           [(str((i + 1) * 5), i) for i in range(5)])
        nu(g, 'lone_reminder', 'Lone worker reminder (s)', 0x7992, 0, 255)
        nu(g, 'lone_response', 'Lone worker response (min)', 0x7993, 0, 255)
        ch(g, 'apo', 'Auto power off (min)', 0x7994,
           [('Off', 0)] + [(str(v), i + 1) for i, v in enumerate([30, 60, 120, 240, 480])])
        ch(g, 'tot', 'Transmit timeout (s)', 0x7995,
           [('Off', 0)] + [(str(i * 15), i) for i in range(1, 15)])
        nu(g, 'tot_alert', 'Transmit timeout advance warning (s)', 0x7996, 0, 10)
        ch(g, 'gps_zone', 'GPS time zone (UTC)', 0x7998,
           [('%+d' % (i - 12), i) for i in range(25)])
        ch(g, 'toneburst', 'Tone burst', 0x799a,
           [(str(v) + ' Hz', i) for i, v in enumerate([1000, 1450, 1750, 2100])])
        nu(g, 'gps_contact', 'GPS contact (0 none, 1-80)', 0x799f, 0, 80)
        bi(g, 'aprs_enable', 'APRS enabled', 0x79a0, 6)
        bi(g, 'lone_enable', 'Lone worker enabled', 0x79a0, 5)
        bi(g, 'falling', 'Falling alarm', 0x79a0, 4)
        ch(g, 'voice', 'Voice announcements', 0x79a0,
           [('Off', 0), ('Chinese', 1), ('English', 2)], mask=12, shift=2)
        ch(g, 'busy_lock', 'Global busy lock', 0x79a0,
           [('Off', 0), ('On', 1)], mask=3)
        bi(g, 'key_lock', 'Keypad lock', 0x79a1, 7)
        bi(g, 'auto_key_lock', 'Automatic keypad lock', 0x79a1, 6)
        bi(g, 'beep', 'Key beep', 0x79a2, 7)
        ch(g, 'end_tone', 'Call end tone', 0x79a2,
           [('Off', 0), ('Mode 1', 1), ('Mode 2', 2), ('Mode 3', 3)], mask=0x60, shift=5)
        bi(g, 'gps_enable', 'GPS enabled', 0x79a3, 7)
        ch(g, 'gps_mode', 'Satellite system', 0x79a3,
           [('GPS', 0), ('BeiDou', 1), ('GPS + BeiDou', 2)], mask=0x60, shift=5)
        bi(g, 'gps_share', 'GPS sharing', 0x79a3, 4)
        bi(g, 'gps_request', 'GPS request', 0x79a3, 3)
        bi(g, 'weather_alert', 'NOAA weather alert', 0x79a5, 3)
        ch(g, 'language', 'Language', 0x79a5,
           [('English', 0), ('Chinese', 1)], mask=4, shift=2)
        ch(g, 'boot_display', 'Power-on display', 0x79a5,
           [('Picture', 0), ('Message', 1), ('Voltage', 2)], mask=3)
        self._cfg_text(g, 'boot_message', 'Power-on message (16 encoded bytes)', 0x79d0, 16, codec='gb2312')
        ch(g, 'tail', 'Tail elimination', 0x79a6,
           [('Off', 0), ('55 Hz', 1), ('120 degrees', 2), ('180 degrees', 3), ('240 degrees', 4)],
           mask=0xe0, shift=5)
        self._cfg_weather(g)
        ch(g, 'lcd_direction', 'LCD direction', 0x79a6,
           [('STAN', 0), ('FAIL', 1)], mask=8, shift=3)
        bi(g, 'fm_interrupt', 'FM radio interrupt', 0x79a6, 2)
        bi(g, 'noise_cancel', 'Noise cancellation', 0x79a6, 1)
        bi(g, 'enhance', 'Enhanced function', 0x79a6, 0)

    def _cfg_scan(self, top):
        g = self._cfg_group(top, 'scan', 'Scan Settings')
        for i in range(10):
            entry = self._cfg_group(g, 'scan%d' % i, 'Frequency scan range %d' % (i + 1))
            self._cfg_freq(entry, 'scan_lower%d' % i, 'Lower limit (MHz)', 0x8100 + 8 * i)
            self._cfg_freq(entry, 'scan_upper%d' % i, 'Upper limit (MHz)', 0x8104 + 8 * i)
        ch = self._cfg_choice; nu = self._cfg_num
        ch(g, 'scan_mode', 'Scan resume mode', 0x8180, [('TO', 0), ('CO', 1), ('SO', 2)])
        nu(g, 'scan_flyback', 'Scan flyback time (ms)', 0x8181, 500, 5000, scale=100, step=100)
        nu(g, 'scan_rx_resume', 'RX resume delay (ms)', 0x8182, 100, 5000, scale=100, step=100)
        nu(g, 'scan_tx_resume', 'TX resume delay (ms)', 0x8183, 100, 5000, scale=100, step=100)
        ch(g, 'scan_return', 'Scan return channel', 0x8184,
           [(v, i) for i, v in enumerate(['Selected', 'Selected + current', 'Last received',
                                        'Last used', 'Priority', 'Priority + current'])])
        self._cfg_bit(g, 'scan_priority', 'Priority scan', 0x8185)
        nu(g, 'scan_priority_channel', 'Priority channel (1-640)', 0x8186, 1, 640, size=2, add=1)
        ch(g, 'scan_range', 'Scan range', 0x8188, [('All', 0), ('Memory scan', 1)])

    def _cfg_keys(self, top):
        g = self._cfg_group(top, 'keys', 'Programmable Keys')
        labels = ['None', 'Scan On/Off', 'Monitor', 'Flashlight', 'FM Radio',
                  'Emergency', 'GPS', 'Frequency measuring', 'Bluetooth', 'Tone burst',
                  'Falling alarm', 'One-touch call', 'Zone change', 'Battery voltage',
                  'TX power', 'VOX On/Off', 'Secondary PTT']
        order = [(1, 'short'), (2, 'short'), (1, 'long'), (2, 'long'),
                 (3, 'short'), (4, 'short'), (3, 'long'), (4, 'long')]
        for i, (key, duration) in enumerate(order):
            self._cfg_choice(g, 'key%d' % i, 'PF%d %s (if fitted)' % (key, duration),
                            0x79b0 + i, list(zip(labels, range(len(labels)))))

    def _cfg_dtmf(self, top):
        g = self._cfg_group(top, 'dtmf', 'DTMF')
        self._cfg_bit(g, 'dtmf_ani', 'DTMF ANI', 0x8200)
        self._cfg_choice(g, 'dtmf_speed', 'Code speed (ms)', 0x8201,
                        [(str(v), i) for i, v in enumerate([50, 100, 200, 300, 500])])
        for off, name, label, minimum in [(2, 'first', 'First code time', 0),
                                          (3, 'pre', 'Precarrier time', 0),
                                          (4, 'delay', 'Code delay', 10)]:
            self._cfg_num(g, 'dtmf_' + name, label + ' (ms)', 0x8200 + off,
                         minimum, 2500, scale=10, step=10)
        self._cfg_choice(g, 'dtmf_ptt_pause', 'PTT-ID pause (s)', 0x8205,
                        [(str(i * 5), i) for i in range(16)])
        self._cfg_bit(g, 'dtmf_sidetone', 'DTMF sidetone', 0x8206)
        self._cfg_num(g, 'dtmf_reset', 'Decode reset (s)', 0x8207, 10, 25)
        self._cfg_choice(g, 'dtmf_sep', 'Separator', 0x8208,
                        [(v, i + 10) for i, v in enumerate('ABCD*#')])
        self._cfg_choice(g, 'dtmf_group', 'Group code', 0x8209,
                        [('None', 255)] + [(v, i + 10) for i, v in enumerate('ABCD*#')])
        self._cfg_choice(g, 'dtmf_response', 'Decode response', 0x820a,
                        [('None', 0), ('Tone', 1), ('Tone + reply', 2)])
        for name, label, off, size in [('self', 'Self ID', 0x8210, 3),
                                      ('bot', 'PTT ID online', 0x8218, 16),
                                      ('eot', 'PTT ID offline', 0x8228, 16),
                                      ('stun', 'Stun code', 0x8238, 16),
                                      ('kill', 'Kill code', 0x8248, 16)]:
            self._cfg_code(g, 'dtmf_' + name, label, off, size)
        enc = self._cfg_group(g, 'dtmf_enc', 'Encode list')
        for i in range(16):
            self._cfg_code(enc, 'dtmf_enc%d' % i, 'Entry %d' % (i + 1),
                          0x8260 + i * 16, 16, bitmap=0x8258, index=i)

    def _cfg_apply_two_frequency(self, setting, off):
        frequency = int(setting.value)
        if frequency and not 288 <= frequency <= 3116:
            raise ValueError('2-tone frequency must be 288-3116 Hz, or 0 for unset')
        self._cfg_write(off, frequency.to_bytes(2, 'big'))

    def _cfg_two_frequency(self, group, name, label, off):
        setting = self._cfg_num(group, name, label, off, 0, 3116, size=2)
        setting.set_apply_callback(self._cfg_apply_two_frequency, off)

    def _cfg_two_tone(self, top):
        g = self._cfg_group(top, 'two_tone', '2-Tone Signaling')
        for i, label in enumerate(['First tone', 'Second tone', 'Long tone']):
            self._cfg_num(g, 'two_time%d' % i, label + ' (ms)', 0x8400 + i,
                         500, 10000, scale=100, step=500)
        self._cfg_num(g, 'two_interval', 'Tone interval (ms)', 0x8403,
                     0, 2000, scale=100, step=100)
        self._cfg_bit(g, 'two_sidetone', 'Sidetone', 0x8404)
        enc = self._cfg_group(g, 'two_enc', 'Encode list')
        for i in range(16):
            e = self._cfg_group(enc, 'two_enc%d' % i, 'Entry %d' % (i + 1))
            off = 0x8410 + i * 16
            self._cfg_two_frequency(e, 'two_f1_%d' % i, 'First frequency (Hz, 0 unset)', off)
            self._cfg_two_frequency(e, 'two_f2_%d' % i, 'Second frequency (Hz, 0 unset)', off + 2)
            self._cfg_text(e, 'two_name%d' % i, 'Name', off + 4, 12)
        self._cfg_choice(g, 'two_response', 'Decode response', 0x8510,
                        [('None', 0), ('Tone', 1), ('Tone + reply', 2)])
        self._cfg_num(g, 'two_reset', 'Decode reset (s)', 0x8511, 10, 25)
        self._cfg_choice(g, 'two_format', 'Decode format', 0x8512,
                        [(v, i) for i, v in enumerate(['A-B', 'A-C', 'A-D', 'B-A', 'B-C', 'B-D',
                                                     'C-A', 'C-B', 'C-D', 'D-A', 'D-B', 'D-C',
                                                     'Long A', 'Long B', 'Long C'])])
        for i, label in enumerate('ABCD'):
            self._cfg_two_frequency(g, 'two_dec_' + label, label + ' decode tone (Hz, 0 unset)',
                                  0x8514 + i * 2)

    def _cfg_five_tone(self, top):
        g = self._cfg_group(top, 'five_tone', '5-Tone Signaling')
        enc = self._cfg_group(g, 'five_enc', 'Encode list (100 entries)')
        standards = list(zip(self._CFG_STANDARDS, range(14)))
        times = [(str(v * 10), v) for v in range(7, 13)]
        for i in range(100):
            off = 0x8680 + i * 32
            e = self._cfg_group(enc, 'five_enc%d' % i, 'Entry %d' % (i + 1))
            self._cfg_bit(e, 'five_enable%d' % i, 'Enabled', 0x9380 + i // 8, i % 8, True)
            self._cfg_choice(e, 'five_standard%d' % i, 'Standard', off, standards)
            self._cfg_choice(e, 'five_time%d' % i, 'Tone duration (ms)', off + 1, times)
            self._cfg_code(e, 'five_code%d' % i, 'Code', off + 3, 12,
                          packed=True, length_off=off + 2, bitmap=0x9380, index=i, max_chars=24)
            self._cfg_choice(e, 'five_special%d' % i, 'Special call', off + 23,
                            [('None', 0), ('Call 1', 1), ('Call 2', 2), ('Call 3', 3)])
            self._cfg_text(e, 'five_name%d' % i, 'Name', off + 24, 8)
        for off, name in [(0x9400, 'BOT'), (0x9410, 'EOT')]:
            e = self._cfg_group(g, 'five_ptt_' + name, 'PTT-ID ' + name)
            self._cfg_choice(e, 'five_ptt_std_' + name, 'Standard', off, standards)
            self._cfg_choice(e, 'five_ptt_time_' + name, 'Tone duration (ms)', off + 1, times)
            self._cfg_code(e, 'five_ptt_code_' + name, 'Code', off + 3, 12,
                          packed=True, length_off=off + 2, max_chars=20)
        dec = self._cfg_group(g, 'five_dec', 'Decode')
        self._cfg_choice(dec, 'five_dec_response', 'Response', 0x9440,
                        [('None', 0), ('Tone', 1), ('Tone + reply', 2)])
        self._cfg_choice(dec, 'five_dec_standard', 'Standard', 0x9441, standards)
        self._cfg_choice(dec, 'five_dec_time', 'Tone duration (ms)', 0x9442, times)
        self._cfg_code(dec, 'five_self', 'Self ID', 0x9443, 5)
        for off, name, minimum in [(0x944b, 'Precarrier', 0), (0x944c, 'Code delay', 0),
                                    (0x944f, 'First code time', 0), (0x9453, 'Stop code time', 10),
                                    (0x9454, 'Decode code time', 0)]:
            self._cfg_num(dec, 'five_' + name.replace(' ', '_'), name + ' (ms)', off,
                         minimum, (2000 if off == 0x9454 else 2550), scale=10, step=10)
        self._cfg_choice(dec, 'five_ptt_pause', 'PTT-ID pause (s)', 0x944d,
                        [('Off', 0)] + [(str(i), i) for i in range(5, 76)])
        self._cfg_num(dec, 'five_reset', 'Decode reset (s)', 0x944e, 10, 25)
        self._cfg_bit(dec, 'five_ani', 'ANI', 0x9450)
        self._cfg_choice(dec, 'five_stop', 'Stop code', 0x9452,
                        [('Off', 0), ('B', 11), ('C', 12), ('D', 13), ('#', 15)])
        info = self._cfg_group(g, 'five_info', 'Information codes')
        for i in range(8):
            off = 0x9480 + i * 16
            e = self._cfg_group(info, 'five_info%d' % i, 'Information code %d' % (i + 1))
            self._cfg_choice(e, 'five_info_function%d' % i, 'Function', off,
                            [(v, j) for j, v in enumerate(['Squelch', 'Stun', 'Kill', 'Wake', 'Emergency'])])
            self._cfg_choice(e, 'five_info_response%d' % i, 'Response', off + 1,
                            [('None', 0), ('Tone', 1), ('Tone + reply', 2)])
            self._cfg_code(e, 'five_info_code%d' % i, 'Code', off + 3, 6,
                          packed=True, length_off=off + 2, max_chars=5)
            self._cfg_text(e, 'five_info_name%d' % i, 'Name', off + 10, 6)

    def _cfg_mdc(self, top):
        g = self._cfg_group(top, 'mdc', 'MDC / BIIS Signaling')
        for i in range(5):
            self._cfg_choice(g, 'mdc_general%d' % i, 'System %d general profile' % (i + 1),
                            0x9580 + i, [(str(j + 1), j) for j in range(5)], mask=0xf0, shift=4)
            self._cfg_choice(g, 'mdc_ptt%d' % i, 'System %d PTT profile' % (i + 1),
                            0x9580 + i, [(str(j + 1), j) for j in range(5)], mask=15)
        for i in range(5):
            off = 0x9588 + i * 8
            e = self._cfg_group(g, 'mdc_para%d' % i, 'MDC system %d' % (i + 1))
            self._cfg_bit(e, 'mdc_control%d' % i, 'Control enabled', off, 7)
            self._cfg_bit(e, 'mdc_dec_tone%d' % i, 'Decode tone', off, 6)
            self._cfg_num(e, 'mdc_id%d' % i, 'Encode ID (decimal)', off + 1, 0, 65535, size=2)
            for k, label in [(3, 'Precarrier'), (4, 'Squelch delay'), (5, 'Decode reset')]:
                self._cfg_num(e, 'mdc_%s%d' % (label.replace(' ', '_'), i), label + ' (ms)',
                             off + k, 0, 2550, scale=10, step=10)
            self._cfg_num(e, 'mdc_enc_sync%d' % i, 'Encode sync', off + 6, 0, 255)
            self._cfg_num(e, 'mdc_dec_sync%d' % i, 'Decode sync', off + 7, 29, 40)
            ptt = 0x95b0 + i * 8
            self._cfg_num(e, 'mdc_bot%d' % i, 'BOT interval (ms)', ptt, 0, 60000,
                         size=2, scale=100, step=100)
            self._cfg_num(e, 'mdc_eot%d' % i, 'EOT interval (ms)', ptt + 2, 0, 60000,
                         size=2, scale=100, step=100)
            for bit, label in [(7, 'Encode'), (6, 'Decode'), (5, 'BOT'), (4, 'EOT'),
                                (3, 'TX tone'), (2, 'RX tone')]:
                self._cfg_bit(e, 'mdc_%s%d' % (label.replace(' ', '_'), i), label,
                             ptt + 4, bit)
        dec = self._cfg_group(g, 'mdc_dec', 'Decode list (100 entries)')
        for i in range(100):
            off = 0x95f0 + i * 16
            e = self._cfg_group(dec, 'mdc_dec%d' % i, 'Entry %d' % (i + 1))
            self._cfg_bit(e, 'mdc_enable%d' % i, 'Enabled', 0x95d8 + i // 8, i % 8, True)
            self._cfg_num(e, 'mdc_decode_id%d' % i, 'Decode ID (decimal)', off, 0, 65535, size=2)
            self._cfg_choice(e, 'mdc_response%d' % i, 'Response', off + 2,
                            [(v, j) for j, v in enumerate(['None', 'Pi', 'PiPi', 'Pi-', 'PiPiPiPiPi', 'PiRo1', 'PiRo2'])])
            self._cfg_text(e, 'mdc_name%d' % i, 'Name', off + 4, 12, codec='gb2312')
        self._cfg_num(g, 'biis_self', 'BIIS self ID', 0x9c80, 0, 65535, size=2)
        self._cfg_num(g, 'biis_group', 'BIIS group ID', 0x9c82, 0, 65535, size=2)
        self._cfg_num(g, 'biis_sync', 'BIIS sync', 0x9c84, 0, 61440, size=2)
        self._cfg_num(g, 'biis_zone', 'BIIS zone code', 0x9c86, 0, 31)
        self._cfg_num(g, 'biis_pre', 'BIIS precarrier (ms)', 0x9c87,
                     0, 2550, scale=10, step=10)
        self._cfg_bit(g, 'biis_tone', 'BIIS sidetone', 0x9c88)

    def _cfg_emergency(self, top):
        group = self._cfg_group(top, 'emergency', 'Emergency Systems')
        for i in range(10):
            off = 0x9d08 + i * 16
            g = self._cfg_group(group, 'emergency%d' % i, 'System %d' % (i + 1))
            self._cfg_num(g, 'emerg_duration%d' % i, 'Cycles (0 continuous)', off, 0, 255)
            self._cfg_choice(g, 'emerg_channel_select%d' % i, 'Channel selection', off + 1,
                            [('Assigned channel', 0), ('Selected channel', 1)])
            for n, label in [(2, 'RX'), (3, 'TX'), (4, 'Exchange')]:
                self._cfg_num(g, 'emerg_%s%d' % (label, i), label + ' time (s)', off + n, 1, 255)
            self._cfg_choice(g, 'emerg_group%d' % i, 'Group', off + 5,
                            [('None', 0), ('Selected', 1)])
            self._cfg_choice(g, 'emerg_signaling%d' % i, 'Signaling', off + 6,
                            [('None', 0), ('DTMF', 1), ('5-Tone', 2)])
            self._cfg_choice(g, 'emerg_type%d' % i, 'Alarm type', off + 7,
                            [('Local alarm', 0), ('ENI + background, silent local', 1),
                             ('ENI + alarm, silent local', 2), ('ENI + alarm, audible local', 3)])
            self._cfg_num(g, 'emerg_channel%d' % i, 'Assigned channel (1-64)', off + 14,
                         1, 64, add=1)
            self._cfg_num(g, 'emerg_zone%d' % i, 'Assigned zone (1-10)', off + 15,
                         1, 10, add=1)

    def _cfg_contacts(self, top):
        group = self._cfg_group(top, 'contacts', 'GPS Contacts')
        for i in range(80):
            off = 0xa010 + i * 16
            g = self._cfg_group(group, 'contact%d' % i, 'Contact %d' % (i + 1))
            self._cfg_bit(g, 'contact_enable%d' % i, 'Enabled', 0xa000 + i // 8, i % 8, True)
            self._cfg_num(g, 'contact_code%d' % i, 'Code', off, 0, 255)
            self._cfg_text(g, 'contact_name%d' % i, 'Name (14 encoded bytes)', off + 2, 14, codec='gb2312')

    def _cfg_tone_read(self, off):
        b = self._cfg_bytes(off, 2)
        if b in (b'\0\0', b'\xff\xff'):
            return 'Off'
        if b[0] & 128:
            digits = '%x%02x' % (b[0] & 15, b[1])
            if all(c in '01234567' for c in digits):
                return 'D' + digits + ('R' if b[0] & 64 else 'N')
        elif all(c in '0123456789' for c in b.hex()):
            n = int(b.hex())
            return '%d.%d' % (n // 10, n % 10)
        return ''

    def _cfg_apply_tone(self, setting, off):
        from decimal import Decimal, InvalidOperation
        import re
        text = str(setting.value).strip().upper()
        if text == 'OFF':
            self._cfg_write(off, b'\0\0')
            return
        match = re.fullmatch(r'D?([0-7]{3})([NR])', text)
        if match:
            digits, pol = match.groups()
            self._cfg_write(off, bytes([(0xc0 if pol == 'R' else 0x80) | int(digits[0]),
                                      (int(digits[1]) << 4) | int(digits[2])]))
            return
        try:
            tone = Decimal(text.replace(',', '.')) * 10
        except InvalidOperation:
            raise ValueError('Tone must be Off, a CTCSS frequency, or D023N / D023R')
        if not tone.is_finite() or tone != tone.to_integral_value() or not 500 <= tone <= 3000:
            raise ValueError('Invalid CTCSS frequency')
        self._cfg_write(off, bytes.fromhex('%04d' % int(tone)))

    def _cfg_tone(self, group, name, label, off):
        setting = RadioSetting('cfg.' + name, label, RadioSettingValueString(
            0, 8, self._cfg_tone_read(off), autopad=False,
            charset='OffDNRoffdnr0123456789.,'))
        setting.set_apply_callback(self._cfg_apply_tone, off)
        group.append(setting)

    def _cfg_vfo(self, top):
        group = self._cfg_group(top, 'vfo', 'VFO A / B')
        for i, label in enumerate('AB'):
            off = 0x7900 + i * 48
            g = self._cfg_group(group, 'vfo_' + label, 'VFO ' + label)
            self._cfg_freq(g, 'vfo_rx_' + label, 'RX frequency (MHz)', off, False)
            self._cfg_freq(g, 'vfo_tx_' + label, 'TX frequency (MHz)', off + 4, False)
            self._cfg_freq(g, 'vfo_offset_' + label, 'Offset (MHz)', off + 12)
            self._cfg_tone(g, 'vfo_rx_tone_' + label, 'RX tone (Off / CTCSS / DCS)', off + 8)
            self._cfg_tone(g, 'vfo_tx_tone_' + label, 'TX tone (Off / CTCSS / DCS)', off + 10)
            self._cfg_choice(g, 'vfo_power_' + label, 'Power', off + 16,
                            [('Low', 0), ('Middle', 1), ('High', 2)], mask=0xc0, shift=6)
            self._cfg_choice(g, 'vfo_width_' + label, 'Bandwidth', off + 16,
                            [('12.5 kHz', 0), ('25 kHz', 2)], mask=0x30, shift=4)
            self._cfg_choice(g, 'vfo_offsetdir_' + label, 'Offset direction', off + 16,
                            [('Off', 0), ('+', 1), ('-', 2)], mask=12, shift=2)
            self._cfg_bit(g, 'vfo_reverse_' + label, 'Reverse', off + 16, 1)
            self._cfg_bit(g, 'vfo_talkaround_' + label, 'Talkaround', off + 16)
            for key, shift in [('5-tone', 6), ('DTMF', 4)]:
                self._cfg_choice(g, 'vfo_' + key + '_' + label, key + ' PTT-ID', off + 17,
                                [('Off', 0), ('BOT', 1), ('EOT', 2), ('Both', 3)], mask=3 << shift, shift=shift)
            self._cfg_choice(g, 'vfo_sql_' + label, 'Squelch mode', off + 17,
                            [('None', 0), ('CTDCS', 1), ('Optional', 2), ('Both', 3)], mask=15)
            self._cfg_choice(g, 'vfo_signal_' + label, 'Signaling', off + 18,
                            [('None', 0), ('DTMF', 1), ('2-Tone', 2), ('5-Tone', 3), ('MDC', 4), ('BIIS', 6)],
                            mask=0xe0, shift=5)
            self._cfg_choice(g, 'vfo_jump_' + label, 'Frequency jump', off + 18,
                            [('Off', 0), ('Normal', 1), ('Strict', 2)], mask=3)
            self._cfg_choice(g, 'vfo_busy_' + label, 'Busy lock', off + 19,
                            [('None', 0), ('Repeater', 1)], mask=0xc0, shift=6)
            self._cfg_bit(g, 'vfo_txdisable_' + label, 'TX disabled', off + 19, 5)
            for bit, text in [(4, 'Scrambler'), (5, 'Compander'), (6, 'Frequency-copy DCS'), (7, 'Frequency-copy 24-bit')]:
                self._cfg_bit(g, 'vfo_' + text.replace(' ', '_') + '_' + label,
                             text, off + 20, bit)
            self._cfg_choice(g, 'vfo_step_' + label, 'Tuning step (kHz)', off + 24,
                            [(str(v), j) for j, v in enumerate([2.5, 5, 6.25, 8.33, 10, 12.5, 20, 25, 30, 50, 100])])
            for rel, key, maximum in [(25, 'DTMF', 15), (26, '2-Tone', 15), (27, '5-Tone', 99),
                                       (28, 'MDC', 4), (29, 'Scan list', 10), (30, 'Emergency', 10)]:
                self._cfg_num(g, 'vfo_index_' + key.replace(' ', '_') + '_' + label,
                             key + ' index (stored)', off + rel, 0, maximum)
            self._cfg_text(g, 'vfo_name_' + label, 'Name', off + 32, 16, codec='gb2312')

# APRS settings for the exact v2 EEPROM schema (CPS/native loader cross-checked).
class APRSSettingsMixin:
    APRS_BASE = 0x9E00

    def _ar(self, offset, size=1):
        return bytes(self._mmap[self.APRS_BASE + offset:self.APRS_BASE + offset + size])

    def _aw(self, offset, data):
        self._mmap[self.APRS_BASE + offset] = bytes(data)
        self._aprs_dirty = True

    def _av(self, offset, mask=255, shift=0):
        return (self._ar(offset)[0] & mask) >> shift

    def _aset(self, offset, value, mask=255, shift=0):
        old = self._ar(offset)[0]
        self._aw(offset, bytes([(old & ~mask) | ((int(value) << shift) & mask)]))

    @staticmethod
    def _acall(raw):
        end = next((i for i, b in enumerate(raw) if b in (0, 32, 255)), len(raw))
        value = raw[:end].decode('ascii', 'replace').upper()
        return value if re.fullmatch(r'[A-Z0-9]{0,6}', value) else ''

    @staticmethod
    def _acall_bytes(value):
        value = str(value).strip().upper()
        if value and not re.fullmatch(r'[A-Z0-9]{1,6}', value):
            raise ValueError('APRS callsigns require 1Ã¢â‚¬â€œ6 Latin letters/numbers; choose the SSID separately')
        return value.encode('ascii').ljust(6, b' ')

    def _astr(self, group, name, label, value, length, callback, charset=VALID_CHARSET):
        setting = RadioSetting('aprs.' + name, label,
            RadioSettingValueString(0, length, value, autopad=False, charset=charset))
        setting.set_apply_callback(lambda s: callback(str(s.value)))
        group.append(setting)
        return setting

    def _alist(self, group, name, label, offset, options, mask=255, shift=0):
        cur = self._av(offset, mask, shift)
        choices = [(label, value) for value, label in enumerate(options)]
        if cur not in [v for _, v in choices]:
            choices.append(('Stored unsupported value %d (preserved)' % cur, cur))
        setting = RadioSetting('aprs.' + name, label, RadioSettingValueMap(choices, cur))
        setting.set_apply_callback(lambda s: self._aset(offset, s.value.get_mem_val(), mask, shift))
        group.append(setting)

    def _abool(self, group, name, label, offset, mask):
        shift = mask.bit_length() - 1
        setting = RadioSetting('aprs.' + name, label, RadioSettingValueBoolean(bool(self._av(offset, mask))))
        setting.set_apply_callback(lambda s: self._aset(offset, bool(s.value), mask, shift))
        group.append(setting)

    def _aint(self, group, name, label, offset, maximum, multiplier=1):
        raw = self._av(offset)
        setting = RadioSetting('aprs.' + name, label,
            RadioSettingValueInteger(0, maximum * multiplier, min(raw, maximum) * multiplier, multiplier))
        def apply(s):
            value = int(s.value)
            if value % multiplier:
                raise ValueError('%s must be a multiple of %d' % (label, multiplier))
            self._aset(offset, value // multiplier)
        setting.set_apply_callback(apply)
        group.append(setting)

    def _apath_text(self):
        count = min(self._av(23), 8)
        paths = []
        for i in range(count):
            record = self._ar(24 + i * 8, 8)
            call = self._acall(record[:6])
            if call:
                paths.append('%s-%d' % (call, record[6] if record[6] <= 15 else 0))
        return ','.join(paths)

    def _apath_set(self, text):
        records = []
        if text.strip():
            for entry in text.split(','):
                match = re.fullmatch(r'\s*([A-Za-z0-9]{1,6})\s*(?:-\s*(\d{1,2}))?\s*', entry)
                if not match or int(match.group(2) or 0) > 15:
                    raise ValueError('Use CALL-SSID entries separated by commas; SSID 0Ã¢â‚¬â€œ15')
                records.append((match.group(1).upper(), int(match.group(2) or 0)))
        if len(records) > 8:
            raise ValueError('At most eight APRS path entries are supported')
        for i in range(8):
            old = self._ar(24 + i * 8, 8)
            call, ssid = records[i] if i < len(records) else ('', 0)
            self._aw(24 + i * 8, self._acall_bytes(call) + bytes([ssid, old[7]]))
        self._aset(23, len(records))

    @staticmethod
    def _acoordinate_decode(raw, direction, latitude):
        value = struct.unpack('<I', raw)[0]
        maximum = 90 if latitude else 180
        deg, part = divmod(value, 100000)
        if deg > maximum or part >= 60000 or (deg == maximum and part):
            return 0.0
        result = deg + part / 1000.0 / 60.0
        negative = not bool(direction & 1) if latitude else bool(direction & 2)
        return -result if negative else result

    def _acoordinate_set(self, value, latitude):
        value = float(value)
        maximum = 90 if latitude else 180
        if not math.isfinite(value) or abs(value) > maximum:
            raise ValueError('Coordinate outside valid range')
        absolute = abs(value)
        deg = int(absolute)
        minutes = int(round((absolute - deg) * 60 * 1000))
        if minutes >= 60000:
            deg += 1
            minutes = 0
        self._aw(100 if latitude else 96, struct.pack('<I', deg * 100000 + minutes))
        self._aset(15, int(value >= 0) if latitude else int(value < 0), 1 if latitude else 2, 0 if latitude else 1)

    @staticmethod
    def _afreq_decode(raw):
        if raw in (b'\xff' * 4, bytes(4)):
            return ''
        digits = ''.join('%02x' % b for b in raw[::-1])
        return digits[:3] + '.' + digits[3:] if digits.isdigit() else ''

    def _afreq_set(self, i, value):
        value = str(value).strip().replace(',', '.')
        if not value:
            raw = b'\xff' * 4
        else:
            try:
                number = Decimal(value)
            except InvalidOperation:
                raise ValueError('Enter an APRS frequency in MHz')
            scaled = number * 100000
            if not number.is_finite() or not 0 < number < 1000 or scaled != scaled.to_integral_value():
                raise ValueError('APRS frequency must be positive MHz with at most five decimal places')
            digits = '%08d' % int(scaled)
            raw = bytes.fromhex(digits)[::-1]
        self._aw(168 + i * 4, raw)

    def _g_aprs(self, top):
        group = RadioSettingGroup('aprs', 'APRS')
        top.append(group)
        identity = RadioSettingGroup('aprs_identity', 'Identity and radio transmission')
        group.append(identity)
        cs = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789'
        for name, label, at in [('source', 'Source callsign', 8), ('target', 'Target / destination callsign', 0)]:
            value = self._acall(self._ar(at, 6))
            # Invalid downloaded bytes display blank but remain unchanged unless edited.
            if not re.fullmatch('[A-Z0-9]{0,6}', value):
                value = ''
            self._astr(identity, name, label, value, 6,
                       lambda v, at=at: self._aw(at, self._acall_bytes(v)), cs)
            self._alist(identity, name + '_ssid', label + ' SSID', at + 6, [str(i) for i in range(16)])
        self._astr(identity, 'path', 'Signal path (e.g. WIDE1-1,WIDE2-1)', self._apath_text(), 95,
                   self._apath_set, cs + '-, ')
        self._alist(identity, 'power', 'APRS power', 15, ['Low', 'Middle', 'High'], 0x30, 4)
        self._alist(identity, 'bandwidth', 'APRS bandwidth', 15, ['12.5 kHz', '25 kHz'], 8, 3)
        self._abool(identity, 'beep', 'APRS transmit beep', 15, 4)
        self._aint(identity, 'precarrier', 'Precarrier time (ms)', 16, 250, 10)
        self._aint(identity, 'code_delay', 'Code delay (ms)', 17, 250, 10)
        tone = self._ar(18, 2)
        options = [('Off', b'\x00\x00')]
        for t in chirp_common.TONES:
            options.append(('%0.1f Hz' % t, bytes.fromhex('%04d' % round(t * 10))))
        for t in chirp_common.DTCS_CODES:
            for polarity, flag in [('N', 0x80), ('R', 0xC0)]:
                options.append(('D%03d%s' % (t, polarity), bytes([flag + t // 100, int('%02d' % (t % 100), 16)])))
        selected = next((i for i, (_, raw) in enumerate(options) if tone == raw), None)
        if selected is None:
            options.append(('Stored 0x%s (preserved)' % tone.hex(), tone))
            selected = len(options) - 1
        setting = RadioSetting('aprs.tone', 'APRS CTCSS / DCS',
                    RadioSettingValueList([label for label, _ in options], current_index=selected))
        setting.set_apply_callback(lambda s: self._aw(18, options[int(s.value)][1]))
        identity.append(setting)
        beacon = RadioSettingGroup('aprs_beacon', 'Beacon and position')
        group.append(beacon)
        self._alist(beacon, 'beacon_source', 'Position source', 92, ['GPS Beacon', 'Fixed Beacon'], 128, 7)
        self._alist(beacon, 'encoding', 'Encoding', 92, ['Position', 'MIC-E'], 8, 3)
        self._alist(beacon, 'ptt', 'PTT beacon', 92, ['Off', 'Beginning of transmission', 'End of transmission'], 0x30, 4)
        self._alist(beacon, 'mice_status', 'MIC-E status', 92,
                    ['Emergency', 'Priority', 'Special', 'Committed', 'Returning', 'In service', 'En route', 'Off duty'], 7)
        self._aint(beacon, 'send_interval', 'Send interval (seconds)', 88, 250)
        self._alist(beacon, 'auto_beacon', 'Automatic beacon period', 89,
                    ['Off', '30 seconds', '1 minute', '2 minutes', '3 minutes', '5 minutes', '10 minutes', '15 minutes', '20 minutes', '30 minutes'])
        self._alist(beacon, 'display_time', 'Received APRS display time', 90,
                    ['Off'] + ['%d seconds' % i for i in range(1, 16)])
        for latitude in [True, False]:
            name = 'latitude' if latitude else 'longitude'
            value = self._acoordinate_decode(self._ar(100 if latitude else 96, 4), self._av(15), latitude)
            setting = RadioSetting('aprs.' + name, 'Fixed ' + name + ' (decimal degrees)',
                RadioSettingValueFloat(-90 if latitude else -180, 90 if latitude else 180, value, resolution=0.000001, precision=6))
            setting.set_apply_callback(lambda s, latitude=latitude: self._acoordinate_set(float(s.value), latitude))
            beacon.append(setting)
        self._alist(beacon, 'height_units', 'Fixed altitude units', 92, ['Feet', 'Metres'], 64, 6)
        height = struct.unpack('<i', self._ar(104, 4))[0]
        setting = RadioSetting('aprs.altitude', 'Fixed altitude',
            RadioSettingValueInteger(-10000, 328084, max(-10000, min(328084, height))))
        setting.set_apply_callback(lambda s: self._aw(104, struct.pack('<i', int(s.value))))
        beacon.append(setting)
        raw = self._ar(108, min(self._av(95), 40))
        text = raw.split(b'\0', 1)[0].split(b'\xff', 1)[0].decode('ascii', 'replace')
        text = ''.join(c if c in VALID_CHARSET else ' ' for c in text)
        def text_set(v):
            data = v.encode('ascii')
            self._aw(108, data.ljust(60, b'\0'))
            self._aset(95, len(data))
        self._astr(beacon, 'comment', 'Position comment / upload text (40 ASCII bytes)', text, 40, text_set)
        freq = RadioSettingGroup('aprs_frequencies', 'Transmit frequencies')
        group.append(freq)
        # The native v2 loader copies only four frequencies into its transmit cache.
        # Bytes for the unused CPS frequency slots 5..8 remain untouched.
        for i in range(4):
            self._astr(freq, 'frequency_%d' % (i + 1), 'APRS frequency %d (MHz; blank = unused)' % (i + 1),
                self._afreq_decode(self._ar(168 + i * 4, 4)), 9, lambda v, i=i: self._afreq_set(i, v), '0123456789.,')
        self._alist(freq, 'frequency_index', 'Transmit frequency selection', 93, ['Frequency %d' % (i + 1) for i in range(4)])
        symbols = RadioSettingGroup('aprs_symbol', 'APRS symbol')
        group.append(symbols)
        for name, label, offset, fallback in [('symbol_table', 'Symbol table / overlay', 20, '/'), ('symbol', 'Symbol character', 21, '>')]:
            value = chr(self._av(offset))
            value = value if 33 <= ord(value) <= 126 else fallback
            def set_symbol(v, offset=offset):
                if len(v) != 1 or not 33 <= ord(v) <= 126:
                    raise ValueError('APRS symbol fields require one printable non-space ASCII character')
                self._aw(offset, v.encode('ascii'))
            self._astr(symbols, name, label, value, 1, set_symbol, VALID_CHARSET[1:])
        filters = RadioSettingGroup('aprs_filters', 'Receive packet filters')
        group.append(filters)
        for name, label, offset, mask in [('all', 'Receive all', 7, 128), ('position', 'Position', 7, 64),
                ('mice', 'MIC-E', 7, 32), ('object', 'Object', 7, 16), ('item', 'Item', 7, 8), ('message', 'Message', 7, 4),
                ('weather', 'Weather', 7, 2), ('nmea', 'NMEA', 7, 1), ('status', 'Status', 15, 128), ('other', 'Other', 15, 64)]:
            self._abool(filters, 'filter_' + name, label, offset, mask)
        rx = RadioSettingGroup('aprs_receive_callsigns', 'Receive callsign list')
        group.append(rx)
        def recount():
            self._aset(22, sum(bool(self._acall(self._ar(200 + i * 8, 6))) for i in range(32)))
        for i in range(32):
            record = RadioSettingGroup('aprs_rx_%d' % i, 'Receive callsign %d' % (i + 1))
            rx.append(record)
            at = 200 + i * 8
            value = self._acall(self._ar(at, 6))
            if not re.fullmatch('[A-Z0-9]{0,6}', value):
                value = ''
            def apply(v, at=at):
                self._aw(at, self._acall_bytes(v))
                recount()
            self._astr(record, 'rx_call_%d' % i, 'Callsign', value, 6, apply, cs)
            self._alist(record, 'rx_ssid_%d' % i, 'SSID', at + 6, [str(n) for n in range(16)])
            self._abool(record, 'rx_filter_%d' % i, 'Filter enabled', at + 7, 1)

    def _validate_aprs(self):
        if not getattr(self, '_aprs_dirty', False):
            return
        if self._av(93) > 3:
            raise ValueError('This firmware loads only four APRS transmit frequencies; select Frequency 1Ã¢â‚¬â€œ4')
        if self._av(92, 128):
            for offset, maximum in [(100, 90), (96, 180)]:
                raw = struct.unpack('<I', self._ar(offset, 4))[0]
                degree, part = divmod(raw, 100000)
                if degree > maximum or part >= 60000 or (degree == maximum and part):
                    raise ValueError('Set valid fixed coordinates or choose GPS Beacon')
            height = struct.unpack('<i', self._ar(104, 4))[0]
            limit = 100000 if self._av(92, 64) else 328084
            if not -10000 <= height <= limit:
                raise ValueError('Fixed altitude is outside the range supported by the selected unit')


class UV5RMPlusGPS(APRSSettingsMixin, NonAPRSSettingsMixin, BF5RHPro):
    """UV-5RM Plus GPS: preserved images, actual zones, verified CPS fields."""
    VENDOR = 'Baofeng'
    MODEL = 'UV-5RM Plus GPS'

    def process_mmap(self):
        super().process_mmap()
        # Retain the downloaded/opened image for changed-only zone validation.
        # This is intentionally not refreshed after edits or failed uploads.
        self._zone_validation_baseline = self._mmap.get_packed()

    def get_features(self):
        rf = super().get_features()
        rf.has_bank = rf.has_bank_names = rf.has_bank_index = True
        rf.can_odd_split = True
        rf.valid_duplexes = ['', '-', '+', 'split', 'off']
        return rf

    def get_bank_model(self):
        return RadioZoneModel(self)

    def sync_out(self):
        do_upload(self)

    def get_memory(self, number):
        if not 1 <= number <= MEM_COUNT:
            raise errors.RadioError('Channel number must be 1-640')
        if self.get_empty(number - 1):
            mem = chirp_common.Memory()
            mem.number, mem.empty = number, True
            return mem
        mem = super().get_memory(number)
        raw = self._memobj.memory[number - 1].tx_freq.get_raw()
        if raw in (bytes(4), b'\xff' * 4):
            mem.duplex, mem.offset = 'off', 0
        return mem

    def set_memory(self, mem):
        if not 1 <= mem.number <= MEM_COUNT:
            raise errors.RadioError('Channel number must be 1-640')
        snapshot = self._mmap.get_packed()
        try:
            idx = mem.number - 1
            was_empty = bool(self.get_empty(idx))
            obj = self._memobj.memory[idx]
            if mem.empty:
                self.get_bank_model().remove_deleted_memory(mem.number)
                if not was_empty:
                    self.set_empty(idx, 1)
                    self.set_skip(idx, 0)
                    obj.fill_raw(b'\x00')
                return
            if mem.freq % 10 or mem.offset % 10:
                raise errors.RadioError('Frequencies must be representable in 10 Hz steps')
            tx = {'': mem.freq, '+': mem.freq + mem.offset,
                  '-': mem.freq - mem.offset, 'split': mem.offset,
                  'off': 0}.get(mem.duplex)
            if tx is None or not 0 <= tx <= 999999990:
                raise errors.RadioError('Invalid transmit frequency or duplex')
            old = None if was_empty else self.get_memory(mem.number)
            if was_empty:
                obj.fill_raw(b'\x00')
                self.set_empty(idx, 0)
            if old is None or mem.freq != old.freq:
                obj.rx_freq = mem.freq // 10
            if old is None or (mem.duplex, mem.offset, mem.freq) != (old.duplex, old.offset, old.freq):
                obj.tx_freq = tx // 10
            if old is None or mem.name != old.name:
                if len(mem.name.encode('ascii')) > 16:
                    raise errors.RadioError('Channel names have a 16-byte limit')
                _set_cstr(obj, 'name', mem.name.rstrip(), 16)
            if old is None or mem.skip != old.skip:
                self.set_skip(idx, int(mem.skip == 'S'))
            if old is None or mem.mode != old.mode:
                obj.bandwidth = self.VALID_MODES.index(mem.mode)
            if mem.power in self.POWER_LEVELS:
                if old is None or mem.power != old.power:
                    obj.power = self.POWER_LEVELS.index(mem.power)
            elif was_empty:
                obj.power = 2
            tone_fields = ('tmode', 'cross_mode', 'rtone', 'ctone', 'dtcs', 'rx_dtcs', 'dtcs_polarity')
            if old is None or any(getattr(mem, f) != getattr(old, f) for f in tone_fields):
                self.set_tone(obj, mem)
            for setting in (mem.extra or []):
                if was_empty or setting.value.changed():
                    setattr(obj, setting.get_name(), _val_to_int(setting.value))
            if was_empty:
                self.get_bank_model().assign_new_memory(mem.number)
        except Exception:
            self._mmap[0] = snapshot
            raise

    def get_settings(self):
        top = RadioSettings()
        self._g_info(top)
        zones = RadioSettingGroup('zones', 'Zones (actual memberships in Banks tab)')
        top.append(zones)
        for z in range(ZONE_MAX):
            rs = RadioSetting('zone_name_%d' % z, 'Zone %d name (%d channels)' %
                              (z+1, len(_zone_members(self,z))),
                              RadioSettingValueString(0,16,_zone_name(self,z),
                              autopad=False,charset=self._CFG_TEXT_CHARSET))
            rs.set_apply_callback(lambda setting, zone: _zone_set_name(self,zone,str(setting.value)),z)
            zones.append(rs)
        self._g_cfg_non_aprs(top)
        self._g_aprs(top)
        return top

    def set_settings(self, settings):
        snapshot = self._mmap.get_packed()
        self._aprs_dirty = False
        def apply(group):
            for element in group:
                if not isinstance(element, RadioSetting):
                    apply(element)
                elif element.value.get_mutable() and element.value.changed():
                    if not element.has_apply_callback():
                        raise errors.RadioError('Missing setting handler: '+element.get_name())
                    element.run_apply_callback()
        try:
            apply(settings)
            self._validate_aprs()
        except Exception:
            self._mmap[0] = snapshot
            raise
        finally:
            self._aprs_dirty = False

    def _validate_upload_image(self, payload, target=None):
        if len(payload) != IMAGE_SZ:
            raise errors.RadioError('Incorrect image size')
        if target is None:
            target = getattr(self, "_zone_validation_baseline", None)
            if target is None:
                raise errors.RadioError("Download or reopen the radio image before uploading")
        if len(target) != IMAGE_SZ:
            raise errors.RadioError('Incomplete baseline image for zone validation')
        for z in range(ZONE_MAX):
            off = 0x7a90 + z * 0x98
            # Name/padding changes do not change zone membership. Retain any
            # downloaded representation when these bytes are unchanged.
            if payload[off:off+130] == target[off:off+130]:
                continue
            count = payload[off]
            if count in (0, 0xff):
                continue  # Empty or erased/unused zone; keep its bytes intact.
            if count > ZONE_SLOTS:
                raise errors.RadioError('Invalid channel count in edited zone %d' % (z+1))
            members = struct.unpack('>64H', payload[off+2:off+130])[:count]
            if len(set(members)) != len(members) or any(
                    i >= MEM_COUNT or self.get_empty(i) for i in members):
                raise errors.RadioError(
                    'Edited zone %d contains duplicate, invalid or empty channel references' % (z+1))


@directory.register
class UV5RMPlusGPSR4(UV5RMPlusGPS):
    MODEL = 'UV-5RM Plus GPS (SV1TSD R4)'
    VARIANT = ""
