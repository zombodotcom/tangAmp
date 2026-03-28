r"""
amp_presets_from_schematics.py
Derive correct amp simulator preset values from real amplifier schematics.

Each amp's component values are documented from the actual circuit, then
translated into the parameters used by amp_sim.py's AmpSim class.

Sources:
  - Fender AB763 Deluxe Reverb / Twin Reverb schematics
  - Marshall JCM800 2203 schematic
  - Vox AC30 Top Boost (JMI) schematic
  - Mesa Boogie Dual Rectifier (2-channel) schematic
  - Rob Robinette's circuit analysis pages
  - David Yeh, Stanford CCRMA tone stack analysis
  - compute_tonestack.py in this project (Bassman/Marshall tone stack values)
"""

import numpy as np

# =============================================================================
# Real Schematic Component Values
# =============================================================================

# Each dict documents the actual schematic values, then derives sim parameters.

SCHEMATICS = {}

# ─────────────────────────────────────────────────────────────────────────────
# 1. Fender Deluxe Reverb (AB763)
# ─────────────────────────────────────────────────────────────────────────────
SCHEMATICS['fender_deluxe'] = {
    'name': 'Fender Deluxe Reverb (AB763)',

    # --- Preamp stages (vibrato channel, normal signal path) ---
    # V1A: first gain stage
    #   Plate resistor: 100k
    #   Cathode resistor: 1.5k
    #   Cathode bypass cap: 25uF/25V
    #   Coupling cap to next stage: 0.02uF (vibrato ch) or 0.047uF (normal ch)
    #   Grid leak: 1M (input)
    #   B+ to plate: ~180V (loaded, from schematic voltage chart)
    #
    # V1B: second gain stage (after volume pot)
    #   Plate resistor: 100k
    #   Cathode resistor: 820 ohm (shared with V2B on normal channel)
    #   Cathode bypass cap: 25uF/25V
    #   Coupling cap: 0.02uF
    #   Grid leak: 1M (from wiper of volume pot)
    #
    # V4B: third gain stage (vibrato channel recovery / reverb mix)
    #   Plate resistor: 100k
    #   Cathode resistor: 1.5k (or 820 shared)
    #   Cathode bypass cap: 25uF/25V
    #   Grid leak: 220k
    #   Coupling cap: 0.1uF to tone stack
    #
    # So: 3 preamp stages on the vibrato channel (V1A -> V1B -> V4B)
    # The normal channel has only 2 (V2A -> V2B)
    'preamp_stages_count': 3,
    'preamp_tube': '12AX7',
    'stages': [
        {'name': 'V1A', 'Rp': 100e3, 'Rk': 1500, 'Ck': 25e-6, 'Cc': 0.02e-6, 'Rg': 1e6},
        {'name': 'V1B', 'Rp': 100e3, 'Rk': 820,  'Ck': 25e-6, 'Cc': 0.02e-6, 'Rg': 1e6},
        {'name': 'V4B', 'Rp': 100e3, 'Rk': 1500, 'Ck': 25e-6, 'Cc': 0.1e-6,  'Rg': 220e3},
    ],
    'preamp_Bplus': 180.0,  # V (preamp plate supply, loaded)

    # --- Tone stack (Fender TMB, same across AB763 models) ---
    # Slope resistor: 100k (between plate and tone stack input)
    # Treble pot: 250k audio taper
    # Bass pot: 250k audio taper
    # Mid pot: 10k (reverse audio, on Deluxe Reverb -- some AB763 use 25k)
    # Treble cap (C1): 250pF
    # Bass cap (C2): 0.1uF
    # Mid cap (C3): 0.047uF (Deluxe Reverb; Bassman uses 0.02uF)
    # Mid fixed resistor: 6.8k
    'tonestack': {
        'R_slope': 100e3,
        'R_treble_pot': 250e3,
        'R_bass_pot': 250e3,
        'R_mid_pot': 10e3,
        'C_treble': 250e-12,
        'C_bass': 0.1e-6,
        'C_mid': 0.047e-6,
        'R_mid_fixed': 6800,
    },

    # --- Interstage coupling ---
    # Signal passes through coupling cap into next stage's grid leak.
    # Voltage divider ratio = Rg_next / (Rp + Rg_next)
    # V1A -> V1B: 1M / (100k + 1M) = 0.909 -> -0.83 dB
    # V1B -> V4B: 220k / (100k + 220k) = 0.688 -> -3.25 dB
    # Average: ~ -2.0 dB
    'interstage_atten_db': -2.0,

    # --- Power amp ---
    # 2x 6V6GT push-pull, fixed bias
    # B+ plate: ~420V (loaded)
    # Output transformer: 6.6k primary : 8 ohm secondary
    # Cathode resistor: none (fixed bias)
    # NFB resistor: 820 ohm from secondary to V3B (phase inverter tail)
    # NFB tail resistor: 47 ohm
    # NFB ratio: 820/(820+47) = 0.946 of output fed back -> moderate NFB
    'power_tube': '6V6',     # (we model as 6L6 since we don't have 6V6 Koren constants)
    'power_Bplus': 420.0,
    'power_OT_primary_Z': 6600,
    'power_rk': 0,           # fixed bias, no cathode resistor
    'power_NFB_resistor': 820,
    'power_NFB_tail': 47,
    'power_tube_count': 2,
    'cabinet': '1x12_open',

    # --- Rectifier ---
    'rectifier': 'GZ34',     # tube rectifier -> more sag
}

# ─────────────────────────────────────────────────────────────────────────────
# 2. Marshall JCM800 (2203, 100W)
# ─────────────────────────────────────────────────────────────────────────────
SCHEMATICS['marshall_jcm800'] = {
    'name': 'Marshall JCM800 2203 (100W)',

    # --- Preamp stages ---
    # The 2203 has 4 gain stages: V1B -> V1A -> V2A -> V2B
    # (Note: Marshall labels differ from Fender; V1B is FIRST in signal path)
    #
    # V1B: first gain stage (high treble input)
    #   Plate resistor: 220k
    #   Cathode resistor: 2.7k
    #   Cathode bypass cap: 0.68uF
    #   Coupling cap: 0.022uF to V1A
    #   Grid leak: 1M (at input jack)
    #   Bright cap: 470pF across preamp gain pot (500k)
    #
    # V1A: "cold clipper" stage (high cathode R = low gain, early clipping)
    #   Plate resistor: 220k
    #   Cathode resistor: 10k (unbypassed! -> heavy NFB, gain ~10x not ~50x)
    #   No cathode bypass cap
    #   Coupling cap: 0.022uF to mixing network
    #   Grid leak: 470k
    #
    # Mixing network: 470k + 470k resistors sum V1A output to next stage
    #
    # V2A: third gain stage
    #   Plate resistor: 100k
    #   Cathode resistor: 820 ohm
    #   Cathode bypass cap: none on stock 2203 (some versions add one)
    #   Coupling cap: 0.022uF to tone stack
    #   Grid leak: 1M
    #
    # V2B: phase inverter (cathodyne / long-tail pair -- not a gain stage)
    #   This is the PI, not counted as a preamp gain stage
    #
    # Effective preamp gain stages: 3 (V1B, V1A, V2A)
    # V1A is deliberately low-gain (cold clipper), so effective gain is less
    # than 3 full-gain stages
    'preamp_stages_count': 3,
    'preamp_tube': '12AX7',
    'stages': [
        {'name': 'V1B', 'Rp': 220e3, 'Rk': 2700,  'Ck': 0.68e-6,  'Cc': 0.022e-6, 'Rg': 1e6},
        {'name': 'V1A', 'Rp': 220e3, 'Rk': 10000,  'Ck': None,     'Cc': 0.022e-6, 'Rg': 470e3},
        {'name': 'V2A', 'Rp': 100e3, 'Rk': 820,    'Ck': None,     'Cc': 0.022e-6, 'Rg': 1e6},
    ],
    'preamp_Bplus': 305.0,  # typical JCM800 preamp B+ (varies 280-320V)
    'bright_cap': 470e-12,  # across 500k gain pot

    # --- Tone stack (Marshall variant of Fender TMB) ---
    # From compute_tonestack.py and schematic:
    # Slope resistor: 33k (Marshall uses much lower than Fender's 100k)
    # Treble pot: 220k (some versions 250k)
    # Bass pot: 1M
    # Mid pot: 25k
    # Treble cap (C1): 470pF (JCM800 increased from Plexi's 250pF)
    # Bass cap (C2): 0.022uF
    # Mid cap (C3): 0.022uF
    'tonestack': {
        'R_slope': 33e3,
        'R_treble_pot': 220e3,
        'R_bass_pot': 1e6,
        'R_mid_pot': 25e3,
        'C_treble': 470e-12,
        'C_bass': 0.022e-6,
        'C_mid': 0.022e-6,
        'R_mid_fixed': 0,  # Marshall: mid pot goes directly to ground
    },

    # --- Interstage coupling ---
    # V1B -> V1A: 470k / (220k + 470k) = 0.681 -> -3.3 dB
    # V1A -> V2A (through mixing network): 470k mixing resistors
    #   Effective: ~0.5 (mixing network) -> -6 dB
    # V2A -> tone stack: direct through coupling cap
    # Average interstage: ~ -4.7 dB
    'interstage_atten_db': -4.7,

    # --- Power amp ---
    # 2x EL34 push-pull, fixed bias
    # B+ plate: ~460-490V (loaded)
    # Output transformer: ~1.7k primary : 4/8/16 ohm secondary
    # Screen grid resistor: 1k/5W
    # NFB: 100k resistor from 4-ohm speaker tap
    # Presence: 4.7k + 22k pot in series, 0.1uF cap
    'power_tube': 'EL34',
    'power_Bplus': 480.0,
    'power_OT_primary_Z': 1700,
    'power_rk': 0,           # fixed bias
    'power_NFB_resistor': 100e3,
    'power_NFB_tail': 0,     # NFB goes to PI input, not to ground resistor
    'power_tube_count': 2,
    'cabinet': '4x12_closed',

    # --- Rectifier ---
    'rectifier': 'solid_state',  # diode rectifier -> less sag than Fender
}

# ─────────────────────────────────────────────────────────────────────────────
# 3. Vox AC30 (Top Boost, JMI era)
# ─────────────────────────────────────────────────────────────────────────────
SCHEMATICS['vox_ac30'] = {
    'name': 'Vox AC30 Top Boost (JMI)',

    # --- Preamp stages ---
    # The AC30 Top Boost channel has 3 preamp stages:
    #
    # V1 (one half): first gain stage (Normal or Brilliant input)
    #   Plate resistor: 220k (JMI; some reissues use 100k)
    #   Cathode resistor: 1.5k (shared between both halves of V1)
    #   Cathode bypass cap: 25uF
    #   Coupling cap: 0.01uF
    #   Grid leak: 1M (at input)
    #
    # V2 (Top Boost gain stage): driven from V1 output
    #   Plate resistor: 220k
    #   Cathode resistor: 1.8k
    #   Cathode bypass cap: 25uF
    #   Coupling cap: 0.047uF (into tone circuit)
    #   Grid leak: 1M
    #
    # Top Boost tone circuit (passive, then into V2 other half as cathode follower)
    #   Bass pot: 1M log
    #   Treble pot: 1M log
    #   Treble cap: 100pF (some versions 47pF)
    #   Bass cap: 0.047uF (some versions 0.05uF)
    #   Cut resistor: 100k
    #   The tone circuit drives the second half of V2 as a cathode follower
    #
    # V3: after cathode follower, into mixer and then to phase inverter
    #   (Handled by the volume control and mixer; not a separate gain stage)
    #
    # Effective gain stages: 2 triode stages + cathode follower (not gain)
    'preamp_stages_count': 2,
    'preamp_tube': '12AX7',
    'stages': [
        {'name': 'V1',  'Rp': 220e3, 'Rk': 1500, 'Ck': 25e-6,   'Cc': 0.01e-6,  'Rg': 1e6},
        {'name': 'V2a', 'Rp': 220e3, 'Rk': 1800, 'Ck': 25e-6,   'Cc': 0.047e-6, 'Rg': 1e6},
    ],
    'preamp_Bplus': 275.0,  # JMI AC30 preamp B+ (varies 260-290V)

    # --- Top Boost tone circuit ---
    # This is NOT a Fender/Marshall TMB stack. It's a different topology:
    # Two interactive RC networks (bass and treble) between gain stage and
    # cathode follower. Much less mid-scoop than Fender.
    # Approximate as shelving EQ with these characteristics:
    #   Bass shelf: ~200Hz, up to +/-10dB
    #   Treble shelf: ~3kHz, up to +/-10dB
    #   Mid: relatively flat (no mid pot, no mid scoop)
    'tonestack': {
        'type': 'vox_top_boost',
        'R_bass_pot': 1e6,
        'R_treble_pot': 1e6,
        'C_treble': 100e-12,   # some versions 47pF
        'C_bass': 0.047e-6,    # some versions 0.05uF
        'R_cut': 100e3,
    },

    # --- Interstage coupling ---
    # V1 -> V2a: 1M / (220k + 1M) = 0.820 -> -1.7 dB
    'interstage_atten_db': -1.7,

    # --- Power amp ---
    # 4x EL84 push-pull (2 pairs), Class A/B (often called "Class A" but
    # actually cathode-biased Class A/B)
    # B+ plate: ~330V
    # Cathode resistor: 47 ohm (shared, self-biased)
    # Output transformer: ~3.5k primary (center-tapped) : 15 ohm secondary
    # NFB: NONE in many JMI versions (some later versions add light NFB)
    # This is key to the AC30's raw, open sound
    'power_tube': 'EL84',    # (we model as EL34 since we lack EL84 Koren constants)
    'power_Bplus': 330.0,
    'power_OT_primary_Z': 3500,
    'power_rk': 47,          # self-biased (cathode resistor)
    'power_NFB_resistor': None,  # NO negative feedback!
    'power_NFB_tail': None,
    'power_tube_count': 4,
    'cabinet': '2x12_open',  # classic AC30 has 2x12 Celestion Blue/Silver

    # --- Rectifier ---
    'rectifier': 'GZ34',     # tube rectifier -> sag
}

# ─────────────────────────────────────────────────────────────────────────────
# 4. Mesa Boogie Dual Rectifier (2-channel, Rev F/G)
# ─────────────────────────────────────────────────────────────────────────────
SCHEMATICS['mesa_dual_rec'] = {
    'name': 'Mesa Boogie Dual Rectifier',

    # --- Preamp stages (Channel 2 / high gain) ---
    # 5 gain stages total on the high-gain channel (Orange/Red):
    #
    # V1a: input stage
    #   Plate resistor: 220k
    #   Cathode resistor: 1.5k (approximately, from bias point)
    #   Cathode bypass cap: 0.68uF (with additional 15uF for low freq)
    #   Coupling cap: 0.022uF
    #   Grid: ferrite bead at input (RF suppression)
    #
    # V1b: second gain stage (after gain pot)
    #   Plate resistor: 100k
    #   Cathode resistor: 1.8k
    #   Cathode bypass cap: 1uF + 15uF (parallel)
    #   Coupling cap: 0.022uF
    #   Grid resistor: 475k (gain pot wiper)
    #
    # V2a: third gain stage
    #   Plate resistor: 100k
    #   Cathode resistor: 39k (!! "cold clipper" -- Soldano SLO technique)
    #   Cathode bypass cap: NONE (unbypassed for controlled clipping)
    #   Coupling cap: 0.022uF
    #   This stage has very low gain (~3-5x) but clips symmetrically
    #
    # V2b: fourth gain stage (some channels)
    #   Plate resistor: 330k
    #   Cathode resistor: variable (channel switching)
    #   Coupling cap: 0.047uF
    #
    # V3: additional gain / tone shaping
    #   Plate resistor: 100k
    #   Various cathode resistor values per channel
    #
    # The clean channel uses only 2-3 stages; high gain uses 4-5
    'preamp_stages_count': 5,  # high gain channel
    'preamp_tube': '12AX7',
    'stages': [
        {'name': 'V1a', 'Rp': 220e3, 'Rk': 1500,  'Ck': 0.68e-6,  'Cc': 0.022e-6, 'Rg': 68e3},
        {'name': 'V1b', 'Rp': 100e3, 'Rk': 1800,  'Ck': 1e-6,     'Cc': 0.022e-6, 'Rg': 475e3},
        {'name': 'V2a', 'Rp': 100e3, 'Rk': 39000,  'Ck': None,     'Cc': 0.022e-6, 'Rg': 1e6},
        {'name': 'V2b', 'Rp': 330e3, 'Rk': 1500,  'Ck': 0.68e-6,  'Cc': 0.047e-6, 'Rg': 1e6},
        {'name': 'V3',  'Rp': 100e3, 'Rk': 820,   'Ck': 25e-6,    'Cc': 0.022e-6, 'Rg': 1e6},
    ],
    'preamp_Bplus': 365.0,  # preamp B+ (tube rectifier mode; SS is ~385V)

    # --- Tone stack ---
    # Mesa uses a modified Fender/Marshall TMB stack:
    # Slope resistor: 39k
    # Treble pot: 250k
    # Bass pot: 250k
    # Mid pot: 25k
    # Treble cap: 500pF (Channel 3 Modern: 680pF)
    # Bass cap: 0.022uF
    # Mid cap: 0.022uF
    'tonestack': {
        'R_slope': 39e3,
        'R_treble_pot': 250e3,
        'R_bass_pot': 250e3,
        'R_mid_pot': 25e3,
        'C_treble': 500e-12,
        'C_bass': 0.022e-6,
        'C_mid': 0.022e-6,
        'R_mid_fixed': 0,
    },

    # --- Interstage coupling ---
    # Many stages have frequency-shaping networks (bright boosts, presence)
    # V1a -> V1b: 475k / (220k + 475k) = 0.683 -> -3.3 dB
    # V1b -> V2a: 1M / (100k + 1M) = 0.909 -> -0.8 dB
    # V2a has only ~3-5x gain due to 39k unbypassed cathode
    # Net interstage average: ~ -2.0 dB (but the cold clipper dominates character)
    'interstage_atten_db': -2.0,

    # --- Power amp ---
    # 2x 6L6GC push-pull, switchable fixed bias
    # B+ plate: ~460V (tube rectifier) to ~480V (solid-state rectifier)
    # Output transformer: ~1.9k primary : 4/8 ohm secondary
    # NFB: moderate (from speaker tap through resistor to PI)
    # Unique feature: switchable tube (5U4) vs solid-state (silicon diode) rectifier
    #   Tube: lower B+, more sag, spongier feel
    #   SS: higher B+, tighter response
    'power_tube': '6L6',
    'power_Bplus': 460.0,    # tube rectifier mode
    'power_OT_primary_Z': 1900,
    'power_rk': 0,           # fixed bias
    'power_NFB_resistor': 47e3,
    'power_NFB_tail': 0,
    'power_tube_count': 2,
    'cabinet': '4x12_closed',

    # --- Rectifier ---
    'rectifier': 'switchable',  # tube (5U4) or solid-state
}

# ─────────────────────────────────────────────────────────────────────────────
# 5. Fender Twin Reverb (AB763 / AA769)
# ─────────────────────────────────────────────────────────────────────────────
SCHEMATICS['fender_twin'] = {
    'name': 'Fender Twin Reverb (AB763)',

    # --- Preamp stages (vibrato channel, same topology as Deluxe Reverb) ---
    # Same stage topology as Deluxe Reverb, but with HIGHER B+ voltages
    # This means more headroom before clipping -> cleaner sound
    #
    # V1A: first gain stage
    #   Plate resistor: 100k
    #   Cathode resistor: 1.5k
    #   Cathode bypass cap: 25uF
    #   Coupling cap: 0.02uF (vibrato channel)
    #   B+ to plate: ~270V (!!! vs 180V on Deluxe -> much more headroom)
    #
    # V1B: second gain stage
    #   Plate resistor: 100k
    #   Cathode resistor: 820 (shared)
    #   Cathode bypass cap: 25uF
    #   Coupling cap: 0.02uF
    #
    # V4B: third gain stage (reverb recovery)
    #   Plate resistor: 100k
    #   Cathode resistor: 1.5k
    #   Cathode bypass cap: 25uF
    #   Coupling cap: 0.1uF
    #
    # Same 3 stages as Deluxe, but the higher plate voltage means each stage
    # has more headroom. Combined with 4x6L6 power amp (vs 2x6V6), the Twin
    # stays clean much longer.
    'preamp_stages_count': 3,
    'preamp_tube': '12AX7',
    'stages': [
        {'name': 'V1A', 'Rp': 100e3, 'Rk': 1500, 'Ck': 25e-6, 'Cc': 0.02e-6, 'Rg': 1e6},
        {'name': 'V1B', 'Rp': 100e3, 'Rk': 820,  'Ck': 25e-6, 'Cc': 0.02e-6, 'Rg': 1e6},
        {'name': 'V4B', 'Rp': 100e3, 'Rk': 1500, 'Ck': 25e-6, 'Cc': 0.1e-6,  'Rg': 220e3},
    ],
    'preamp_Bplus': 270.0,  # Much higher than Deluxe's 180V!

    # --- Tone stack (identical to Deluxe Reverb) ---
    'tonestack': {
        'R_slope': 100e3,
        'R_treble_pot': 250e3,
        'R_bass_pot': 250e3,
        'R_mid_pot': 10e3,
        'C_treble': 250e-12,
        'C_bass': 0.1e-6,
        'C_mid': 0.047e-6,
        'R_mid_fixed': 6800,
    },

    # --- Interstage coupling (same topology as Deluxe) ---
    'interstage_atten_db': -2.0,

    # --- Power amp ---
    # 4x 6L6GC push-pull, fixed bias, solid-state rectifier
    # B+ plate: ~460V (loaded)
    # Output transformer: 2k primary : 4 ohm secondary (huge iron!)
    # NFB: 820 ohm from secondary (same as Deluxe, but with 100 ohm tail
    #       on some versions, or 47 ohm on others)
    # The Twin's massive power section + high NFB = extremely clean amp
    # Stays clean up to about 6 on the volume, while Deluxe breaks up at ~4
    'power_tube': '6L6',
    'power_Bplus': 460.0,
    'power_OT_primary_Z': 2000,  # 2k:4 ohm (from AB763 model differences page)
    'power_rk': 0,               # fixed bias
    'power_NFB_resistor': 820,
    'power_NFB_tail': 100,       # higher tail R on some Twin versions
    'power_tube_count': 4,       # KEY difference: 4 tubes = 2x the headroom
    'cabinet': '2x12_open',

    # --- Rectifier ---
    'rectifier': 'solid_state',  # no sag at all -> super tight and clean
}


# =============================================================================
# Derive Simulator Parameters from Schematic Values
# =============================================================================

def compute_interstage_atten_linear(schematic):
    """Convert dB attenuation to linear for amp_sim.py's interstage_atten parameter.

    amp_sim.py divides by interstage_lin = 10^(atten_db/20), so we need to
    express the attenuation correctly. The current code uses a positive dB value
    and divides by it, meaning interstage_atten should be the dB LOSS (positive).
    """
    # amp_sim.py: interstage_lin = 10^(interstage_atten/20); x = ac_out / interstage_lin
    # For -2 dB coupling loss, we want the signal reduced by 2 dB,
    # which means dividing by 10^(2/20) = 1.26
    # So interstage_atten should be +2.0 (positive, dB of loss)
    return abs(schematic['interstage_atten_db'])


def compute_power_sag(schematic):
    """Estimate power supply sag from rectifier type."""
    rect = schematic.get('rectifier', 'solid_state')
    if rect == 'GZ34':
        return 0.35   # tube rectifier, moderate sag
    elif rect == 'switchable':
        return 0.40   # Mesa tube rectifier mode, designed for sag
    else:
        return 0.15   # solid-state rectifier, minimal sag


def compute_power_clip_level(schematic):
    """Estimate clipping level from B+ and transformer impedance.

    The maximum clean swing is approximately B+ * 0.9 for push-pull amps
    (each tube can swing nearly rail-to-rail through the OT).
    Higher B+ and more tubes = higher clip level.
    """
    bplus = schematic['power_Bplus']
    n_tubes = schematic['power_tube_count']
    # Rough approximation: clip level scales with B+ and sqrt of tube count
    # (more tubes share the current, each clips less)
    base_clip = bplus * 0.45
    if n_tubes >= 4:
        base_clip *= 1.2  # 4 tubes have ~20% more headroom than 2
    return round(base_clip, 1)


def compute_effective_power_rp(schematic):
    """Compute reflected plate resistance from OT impedance.

    For push-pull amps, the effective plate load each tube sees is
    approximately OT_primary_Z / 4 (for 2 tubes) or OT_primary_Z / 8 (for 4).
    But for our single-tube power amp model, we use a simplified Rp that gives
    similar gain characteristics.
    """
    z_ot = schematic['power_OT_primary_Z']
    n_tubes = schematic['power_tube_count']
    # Each tube sees Z_primary / (n_tubes / 2) in push-pull
    # But we model as single-ended equivalent
    rp = z_ot / 2  # simplified: half the primary Z for our model
    return rp


def derive_preset(key):
    """Derive amp_sim.py preset dict from schematic data."""
    s = SCHEMATICS[key]

    preset = {
        'name': s['name'],
        'preamp_stages': s['preamp_stages_count'],
        'preamp_tube': s['preamp_tube'],
        'interstage_atten': compute_interstage_atten_linear(s),
        'power_tube': s.get('power_tube', '6L6'),
        'power_vb': s['power_Bplus'],
        'power_rp': compute_effective_power_rp(s),
        'power_rk': s.get('power_rk', 250),
        'power_sag': compute_power_sag(s),
        'power_clip': compute_power_clip_level(s),
        'cabinet': s['cabinet'],
    }

    # Tone knob positions (5 = noon, same as current)
    # These are knob positions, not component values -- keep defaults
    # The actual tonal character comes from the tone stack component values
    # which should be implemented in a future tone_stack upgrade
    preset['tone_bass'] = 5
    preset['tone_mid'] = 5
    preset['tone_treble'] = 5

    # Gain and master depend on playing style, not schematic
    preset['input_gain'] = 5
    preset['master_vol'] = 5

    return preset


# =============================================================================
# Current (guessed) values from amp_sim.py
# =============================================================================

CURRENT_PRESETS = {
    'fender_deluxe': dict(
        name='Fender Deluxe Reverb',
        preamp_stages=2, preamp_tube='12AX7',
        interstage_atten=0.35,
        power_tube='6L6', power_vb=400, power_rp=2000, power_rk=250,
        power_sag=0.3, power_clip=200,
        cabinet='1x12_open',
    ),
    'marshall_jcm800': dict(
        name='Marshall JCM800',
        preamp_stages=3, preamp_tube='12AX7',
        interstage_atten=0.35,
        power_tube='EL34', power_vb=450, power_rp=1700, power_rk=200,
        power_sag=0.2, power_clip=180,
        cabinet='4x12_closed',
    ),
    'vox_ac30': dict(
        name='Vox AC30',
        preamp_stages=2, preamp_tube='12AX7',
        interstage_atten=0.35,
        power_tube='EL34', power_vb=380, power_rp=1800, power_rk=220,
        power_sag=0.25, power_clip=170,
        cabinet='1x12_open',
    ),
    'mesa_dual_rec': dict(
        name='Mesa Dual Rectifier',
        preamp_stages=3, preamp_tube='12AX7',
        interstage_atten=0.35,
        power_tube='6L6', power_vb=420, power_rp=1900, power_rk=230,
        power_sag=0.4, power_clip=190,
        cabinet='4x12_closed',
    ),
    'fender_twin': dict(
        name='Fender Twin Clean',
        preamp_stages=1, preamp_tube='12AX7',
        interstage_atten=0.35,
        power_tube='6L6', power_vb=400, power_rp=2000, power_rk=250,
        power_sag=0.15, power_clip=250,
        cabinet='2x12_open',
    ),
}


# =============================================================================
# Comparison Table
# =============================================================================

def print_comparison():
    """Print a table comparing current guessed values vs schematic-derived values."""

    print("=" * 100)
    print("COMPARISON: Current (guessed) vs Schematic-Derived Preset Values")
    print("=" * 100)

    params = [
        ('preamp_stages', 'Preamp Stages'),
        ('interstage_atten', 'Interstage Atten'),
        ('power_tube', 'Power Tube'),
        ('power_vb', 'Power B+ (V)'),
        ('power_rp', 'Power Rp (ohm)'),
        ('power_rk', 'Power Rk (ohm)'),
        ('power_sag', 'Power Sag'),
        ('power_clip', 'Power Clip Level'),
        ('cabinet', 'Cabinet'),
    ]

    for key in ['fender_deluxe', 'marshall_jcm800', 'vox_ac30', 'mesa_dual_rec', 'fender_twin']:
        current = CURRENT_PRESETS[key]
        derived = derive_preset(key)

        print(f"\n{'-' * 100}")
        print(f"  {derived['name']}")
        print(f"{'-' * 100}")
        print(f"  {'Parameter':<25s} {'Current (guessed)':<25s} {'Schematic-derived':<25s} {'Changed?'}")
        print(f"  {'-'*25} {'-'*25} {'-'*25} {'-'*10}")

        for param_key, param_label in params:
            cur_val = current.get(param_key, '???')
            new_val = derived.get(param_key, '???')

            changed = ''
            if str(cur_val) != str(new_val):
                changed = ' *** CHANGED'

            print(f"  {param_label:<25s} {str(cur_val):<25s} {str(new_val):<25s}{changed}")


def print_schematic_summary():
    """Print a summary of all schematic component values found."""

    print("\n" + "=" * 100)
    print("SCHEMATIC COMPONENT VALUES SUMMARY")
    print("=" * 100)

    for key, s in SCHEMATICS.items():
        print(f"\n{'=' * 80}")
        print(f"  {s['name']}")
        print(f"{'=' * 80}")

        print(f"\n  Preamp ({s['preamp_stages_count']} stages, {s['preamp_tube']}):")
        print(f"  B+ = {s['preamp_Bplus']}V")
        for stage in s['stages']:
            ck_str = f"{stage['Ck']*1e6:.2f}uF" if stage['Ck'] else "NONE (unbypassed)"
            print(f"    {stage['name']:>4s}: Rp={stage['Rp']/1e3:.0f}k  "
                  f"Rk={stage['Rk']:.0f}  Ck={ck_str}  "
                  f"Cc={stage['Cc']*1e9:.1f}nF  Rg={stage['Rg']/1e3:.0f}k")

        ts = s['tonestack']
        print(f"\n  Tone Stack:")
        if ts.get('type') == 'vox_top_boost':
            print(f"    Type: Vox Top Boost (NOT Fender/Marshall TMB)")
            print(f"    Bass pot: {ts['R_bass_pot']/1e6:.0f}M  Treble pot: {ts['R_treble_pot']/1e6:.0f}M")
            print(f"    C_treble: {ts['C_treble']*1e12:.0f}pF  C_bass: {ts['C_bass']*1e9:.1f}nF")
            print(f"    R_cut: {ts['R_cut']/1e3:.0f}k")
        else:
            print(f"    R_slope: {ts['R_slope']/1e3:.0f}k  "
                  f"C_treble: {ts['C_treble']*1e12:.0f}pF  "
                  f"C_bass: {ts['C_bass']*1e9:.1f}nF  "
                  f"C_mid: {ts['C_mid']*1e9:.1f}nF")
            if ts.get('R_mid_fixed', 0) > 0:
                print(f"    R_mid_fixed: {ts['R_mid_fixed']:.0f}")

        print(f"\n  Power Amp: {s.get('power_tube_count', 2)}x {s.get('power_tube', '???')}")
        print(f"    B+: {s['power_Bplus']}V  OT primary: {s['power_OT_primary_Z']}ohm")
        nfb = s.get('power_NFB_resistor')
        if nfb is None:
            print(f"    NFB: NONE")
        else:
            tail = s.get('power_NFB_tail', 0)
            print(f"    NFB: {nfb/1e3:.1f}k  tail: {tail}ohm")
        print(f"    Cathode R: {s['power_rk']} (0 = fixed bias)")
        print(f"    Rectifier: {s.get('rectifier', 'unknown')}")
        print(f"    Cabinet: {s['cabinet']}")


def print_recommended_changes():
    """Print what should change in amp_sim.py."""

    print("\n" + "=" * 100)
    print("RECOMMENDED CHANGES TO amp_sim.py PRESETS")
    print("=" * 100)

    for key in ['fender_deluxe', 'marshall_jcm800', 'vox_ac30', 'mesa_dual_rec', 'fender_twin']:
        derived = derive_preset(key)
        current = CURRENT_PRESETS[key]

        changes = []
        for param in ['preamp_stages', 'interstage_atten', 'power_tube', 'power_vb',
                       'power_rp', 'power_rk', 'power_sag', 'power_clip', 'cabinet']:
            cur = current.get(param)
            new = derived.get(param)
            if str(cur) != str(new):
                changes.append((param, cur, new))

        if changes:
            print(f"\n  {derived['name']}:")
            for param, old, new in changes:
                print(f"    {param}: {old} -> {new}")
        else:
            print(f"\n  {derived['name']}: no changes needed")

    print(f"""
  IMPORTANT NOTES:
  ----------------
  1. The Fender Deluxe actually has 3 preamp stages (not 2).
     V1A (input) -> V1B (gain) -> V4B (reverb recovery) on the vibrato channel.

  2. The Fender Twin also has 3 preamp stages (not 1!).
     Same topology as Deluxe but with 270V preamp B+ (vs 180V) and 4x6L6
     power tubes (vs 2x6V6). The cleanliness comes from high headroom, not
     fewer stages.

  3. The Marshall JCM800 has 3 gain stages, but V1A is a "cold clipper" with
     10k unbypassed cathode (gain ~10x instead of ~50x). This is the key to
     the Marshall crunch character.

  4. The Mesa Dual Rectifier has 5 preamp stages on the high-gain channel,
     including a 39k unbypassed "cold clipper" (borrowed from Soldano SLO).
     This is the most gain of any amp here.

  5. The Vox AC30 has NO negative feedback in many versions. This is central
     to its raw, open sound character.

  6. The interstage_atten values were all 0.35 (wrong unit/interpretation).
     They should be the dB of loss from the coupling network voltage divider:
     Fender: ~2.0 dB, Marshall: ~4.7 dB, Vox: ~1.7 dB, Mesa: ~2.0 dB.

  7. The power tube for Fender Deluxe should be 6V6 (not 6L6), and the
     Vox AC30 uses EL84 (not EL34). We don't have Koren constants for these
     tubes yet, so we substitute the closest available.

  8. FUTURE WORK: The tone stack should use the actual component values with
     compute_tonestack.py's transfer function, not generic biquad shelving.
     The Vox Top Boost tone circuit is a completely different topology.
""")


# =============================================================================
# Generate updated preset dicts
# =============================================================================

def get_updated_presets():
    """Return the complete updated preset dictionaries for amp_sim.py."""

    presets = {}
    # Fender Deluxe: tone knobs set to classic "Fender clean" position
    p = derive_preset('fender_deluxe')
    p['tone_bass'] = 6
    p['tone_mid'] = 5
    p['tone_treble'] = 7
    p['input_gain'] = 5
    p['master_vol'] = 5
    presets['fender_deluxe'] = p

    # Marshall JCM800: mid-forward, moderate gain
    p = derive_preset('marshall_jcm800')
    p['tone_bass'] = 5
    p['tone_mid'] = 8
    p['tone_treble'] = 6
    p['input_gain'] = 7
    p['master_vol'] = 6
    presets['marshall_jcm800'] = p

    # Vox AC30: treble-forward, chimey
    p = derive_preset('vox_ac30')
    p['tone_bass'] = 7
    p['tone_mid'] = 4
    p['tone_treble'] = 8
    p['input_gain'] = 6
    p['master_vol'] = 6
    presets['vox_ac30'] = p

    # Mesa Dual Rec: heavy, scooped mids
    p = derive_preset('mesa_dual_rec')
    p['tone_bass'] = 8
    p['tone_mid'] = 3
    p['tone_treble'] = 7
    p['input_gain'] = 8
    p['master_vol'] = 5
    presets['mesa_dual_rec'] = p

    # Fender Twin: pristine clean
    p = derive_preset('fender_twin')
    p['tone_bass'] = 5
    p['tone_mid'] = 6
    p['tone_treble'] = 6
    p['input_gain'] = 3
    p['master_vol'] = 5
    presets['fender_twin'] = p

    return presets


# =============================================================================
# Main
# =============================================================================

if __name__ == '__main__':
    print_schematic_summary()
    print_comparison()
    print_recommended_changes()

    print("\n" + "=" * 100)
    print("UPDATED PRESET DICTIONARIES (for amp_sim.py)")
    print("=" * 100)

    presets = get_updated_presets()
    for key, p in presets.items():
        print(f"\n'{key}': dict(")
        for k, v in p.items():
            if isinstance(v, str):
                print(f"    {k}='{v}',")
            elif isinstance(v, float):
                if v == int(v):
                    print(f"    {k}={int(v)},")
                else:
                    print(f"    {k}={v},")
            else:
                print(f"    {k}={v},")
        print("),")
