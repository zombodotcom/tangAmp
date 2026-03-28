# Amp Schematic Notes

Component values for each amp preset, derived from actual amplifier schematics.

## Sources

- Rob Robinette's AB763 Deluxe Reverb analysis: https://robrobinette.com/How_The_AB763_Deluxe_Reverb_Works.htm
- Rob Robinette's Marshall JCM800 analysis: https://robrobinette.com/How_the_Marshall_JCM800_Works.htm
- Rob Robinette's AB763 model differences: https://robrobinette.com/AB763_Model_Differences.htm
- Rob Robinette's TMB tone stack analysis: https://robrobinette.com/How_The_TMB_Tone_Stack_Works.htm
- Warped Musician Mesa Rectifier analysis: https://warpedmusician.wordpress.com/2015/09/14/mesa-rectifier-design-concepts-pt-1-introduction-input-stage-and-clean-channel/
- Vox AC30 schematics archive: https://www.voxac30.org.uk/vox_ac30_circuit_diagrams.html
- Vox AC30 Top Boost circuit history: https://www.voxac30.org.uk/vox_ac30_top_boost_circuit.html
- David Yeh, Stanford CCRMA: "Discretization of the '59 Fender Bassman Tone Stack" (tone stack transfer function)
- Schematic PDFs: schematicheaven.net, el34world.com, prowessamplifiers.com

---

## 1. Fender Deluxe Reverb (AB763)

The AB763 is the "blackface" Fender circuit (1963-1967), used across the Deluxe Reverb, Twin Reverb, Super Reverb, and others. All share the same topology with different B+ voltages and power tube configurations.

### Preamp (Vibrato Channel)

| Stage | Tube  | Rp    | Rk     | Ck     | Cc       | Rg   | Notes |
|-------|-------|-------|--------|--------|----------|------|-------|
| V1A   | 12AX7 | 100k  | 1.5k   | 25uF   | 0.02uF   | 1M   | Input stage |
| V1B   | 12AX7 | 100k  | 820    | 25uF   | 0.02uF   | 1M   | After volume pot |
| V4B   | 12AX7 | 100k  | 1.5k   | 25uF   | 0.1uF    | 220k | Reverb recovery |

- Preamp B+: 180V (loaded)
- All stages use 100k plate resistors (verified from schematic)
- The 820-ohm cathode on V1B is shared between channels (carries 2x current)
- 25uF bypass caps on all stages give full bass boost

### Tone Stack (Fender TMB)

| Component     | Value  | Ref    |
|---------------|--------|--------|
| Slope resistor| 100k   | From plate to tone stack input |
| Treble pot    | 250k   | Audio taper |
| Bass pot      | 250k   | Audio taper |
| Mid pot       | 10k    | Reverse audio |
| Treble cap    | 250pF  | C1 |
| Bass cap      | 0.1uF  | C2 |
| Mid cap       | 0.047uF| C3 (Deluxe; Bassman uses 0.02uF) |
| Mid resistor  | 6.8k   | Fixed, in series with mid pot |

### Power Amp

| Parameter              | Value              |
|------------------------|--------------------|
| Tubes                  | 2x 6V6GT           |
| Bias                   | Fixed               |
| B+ plate               | ~420V               |
| OT primary impedance   | 6.6k ohm            |
| OT secondary           | 8 ohm               |
| NFB resistor           | 820 ohm             |
| NFB tail resistor      | 47 ohm              |
| Rectifier              | GZ34 (tube)         |

### Key Character

- Moderate headroom (22W) -- breaks up around volume 4-5
- Tube rectifier adds sag and compression
- Classic Fender mid-scoop from TMB tone stack
- The 6V6 power tubes clip more softly than 6L6

---

## 2. Marshall JCM800 (2203, 100W)

The JCM800 2203 evolved from the Plexi/JTM45 which was itself derived from the Fender Bassman. Key differences: more preamp gain stages, "cold clipper" design, lower slope resistor in tone stack.

### Preamp

| Stage | Tube  | Rp    | Rk      | Ck      | Cc       | Rg    | Notes |
|-------|-------|-------|---------|---------|----------|-------|-------|
| V1B   | 12AX7 | 220k  | 2.7k    | 0.68uF  | 0.022uF  | 1M    | First gain stage |
| V1A   | 12AX7 | 220k  | 10k     | NONE    | 0.022uF  | 470k  | "Cold clipper" |
| V2A   | 12AX7 | 100k  | 820     | NONE    | 0.022uF  | 1M    | Third gain stage |

- Preamp B+: ~305V
- Bright cap: 470pF across 500k gain pot
- V1A is the "cold clipper": 10k unbypassed cathode means gain is only ~10x (instead of ~50x for a bypassed 12AX7). This stage clips early and symmetrically, creating the Marshall crunch character.
- V1B uses 220k plate resistor (not 100k like Fender) for higher gain
- 470k mixing resistors between V1A output and V2A input
- 0.022uF coupling caps are smaller than Fender's 0.047/0.1uF -- rolls off more bass between stages, contributing to the tighter Marshall tone

### Tone Stack (Marshall variant of Fender TMB)

| Component     | Value   | Ref    |
|---------------|---------|--------|
| Slope resistor| 33k     | Much lower than Fender's 100k |
| Treble pot    | 220k    | (some versions 250k) |
| Bass pot      | 1M      | Linear |
| Mid pot       | 25k     | Linear |
| Treble cap    | 470pF   | C1 (Plexi was 250pF) |
| Bass cap      | 0.022uF | C2 (same as Plexi) |
| Mid cap       | 0.022uF | C3 |

The 33k slope resistor (vs Fender's 100k) dramatically changes the stack's character: less insertion loss, different mid interaction, less "shimmer."

### Power Amp

| Parameter              | Value              |
|------------------------|--------------------|
| Tubes                  | 2x EL34             |
| Bias                   | Fixed               |
| B+ plate               | ~480V               |
| OT primary impedance   | ~1.7k ohm           |
| Screen resistors       | 1k / 5W             |
| NFB resistor           | 100k                |
| NFB tap                | 4-ohm speaker jack  |
| Presence               | 4.7k + 22k pot, 0.1uF cap |
| Rectifier              | Solid-state (diodes) |

### Key Character

- High preamp gain (3 stages + cold clipper)
- Tight low end (solid-state rectifier, no sag)
- Mid-forward aggression (from cold clipper + tone stack)
- 100k NFB resistor is very high value = light NFB (less damping than Fender)

---

## 3. Vox AC30 (Top Boost, JMI)

The AC30 Top Boost is fundamentally different from Fender/Marshall designs. It uses a unique tone circuit (derived from Gibson GA-77), cathode-biased power amp (no NFB in original JMI versions), and EL84 power tubes.

### Preamp

| Stage | Tube  | Rp    | Rk     | Ck     | Cc       | Rg   | Notes |
|-------|-------|-------|--------|--------|----------|------|-------|
| V1    | 12AX7 | 220k  | 1.5k   | 25uF   | 0.01uF   | 1M   | Input stage (shared cathode) |
| V2a   | 12AX7 | 220k  | 1.8k   | 25uF   | 0.047uF  | 1M   | Top Boost gain stage |

- Preamp B+: ~275V
- V1 uses 220k plate resistor (like Marshall, unlike Fender's 100k)
- V2a drives the Top Boost tone circuit
- V2b operates as a cathode follower (unity gain, low output impedance) to drive the volume/mixer -- not counted as a gain stage
- The 0.01uF coupling cap after V1 rolls off bass earlier than Fender (which uses 0.02-0.047uF)

### Top Boost Tone Circuit

This is NOT a Fender/Marshall TMB stack. It is a different topology with two interactive RC networks:

| Component     | Value   | Notes |
|---------------|---------|-------|
| Bass pot      | 1M log  | |
| Treble pot    | 1M log  | |
| Treble cap    | 100pF   | (some versions 47pF) |
| Bass cap      | 47nF    | (some versions 50nF) |
| Cut resistor  | 100k    | |

Key differences from Fender/Marshall TMB:
- No mid control (the AC30 has no "mid scoop")
- Different topology: less insertion loss
- The cathode follower (V2b) buffers the tone circuit output
- Result: flatter mid response, more "open" sound

### Power Amp

| Parameter              | Value              |
|------------------------|--------------------|
| Tubes                  | 4x EL84             |
| Bias                   | Cathode (self-biased, 47 ohm shared) |
| B+ plate               | ~330V               |
| OT primary impedance   | ~3.5k ohm           |
| NFB                    | **NONE** in JMI versions |
| Rectifier              | GZ34 (tube)         |

### Key Character

- NO negative feedback = raw, open, uncompressed power amp response
- EL84 tubes have a distinctive breakup character (more aggressive than 6V6, sweeter than EL34)
- Cathode bias = Class A operation for much of the signal, warm even-harmonic distortion
- Tube rectifier sag adds compression
- The "jangly" Vox chimney comes from 220k plate resistors + small coupling caps + no mid scoop

---

## 4. Mesa Boogie Dual Rectifier (2-Channel, Rev F/G)

The Dual Rectifier is the highest-gain amp here. Its high-gain channel has 5 preamp stages including the "cold clipper" technique borrowed from the Soldano SLO-100. Unique feature: switchable tube vs solid-state rectifier.

### Preamp (High-Gain Channel)

| Stage | Tube  | Rp    | Rk      | Ck      | Cc       | Rg    | Notes |
|-------|-------|-------|---------|---------|----------|-------|-------|
| V1a   | 12AX7 | 220k  | 1.5k    | 0.68uF  | 0.022uF  | 68k   | Input (ferrite bead) |
| V1b   | 12AX7 | 100k  | 1.8k    | 1uF+15uF| 0.022uF | 475k  | After gain pot |
| V2a   | 12AX7 | 100k  | 39k     | NONE    | 0.022uF  | 1M    | Cold clipper (Soldano) |
| V2b   | 12AX7 | 330k  | 1.5k    | 0.68uF  | 0.047uF  | 1M    | Fourth gain stage |
| V3    | 12AX7 | 100k  | 820     | 25uF    | 0.022uF  | 1M    | Final gain/shaping |

- Preamp B+: ~365V (tube rect) / ~385V (SS rect)
- V2a has a 39k unbypassed cathode resistor ("cold clipper" from Soldano SLO-100). Gain is only ~3-5x but it clips symmetrically, creating tight crunch.
- V1b has parallel bypass caps (1uF + 15uF) for stepped frequency shaping
- The clean channel uses only 2-3 of these stages
- Extensive bright-boost and presence circuits throughout

### Tone Stack

| Component     | Value   | Notes |
|---------------|---------|-------|
| Slope resistor| 39k     | Between Fender (100k) and Marshall (33k) |
| Treble pot    | 250k    | |
| Bass pot      | 250k    | |
| Mid pot       | 25k     | |
| Treble cap    | 500pF   | Ch3 Modern: 680pF |
| Bass cap      | 0.022uF | |
| Mid cap       | 0.022uF | |

### Power Amp

| Parameter              | Value              |
|------------------------|--------------------|
| Tubes                  | 2x 6L6GC            |
| Bias                   | Fixed               |
| B+ plate               | ~460V (tube) / ~480V (SS) |
| OT primary impedance   | ~1.9k ohm           |
| NFB                    | Moderate (resistor from speaker tap to PI) |
| Rectifier              | **Switchable**: 5U4 tube or silicon diodes |

### Key Character

- Highest gain of any amp here (5 preamp stages)
- Cold clipper (V2a) creates tight, focused distortion
- Switchable rectifier: tube for sag/compression, SS for tightness
- Deep bass capability from 4x12 closed-back cabinet
- Scooped mids in typical use (bass/treble cranked, mid low)

---

## 5. Fender Twin Reverb (AB763)

The Twin Reverb uses the **exact same AB763 preamp topology** as the Deluxe Reverb. The extreme cleanliness comes from two key differences: much higher B+ voltages and 4x 6L6 power tubes (vs 2x 6V6).

### Preamp (same topology as Deluxe Reverb)

| Stage | Tube  | Rp    | Rk     | Ck     | Cc       | Rg   | Notes |
|-------|-------|-------|--------|--------|----------|------|-------|
| V1A   | 12AX7 | 100k  | 1.5k   | 25uF   | 0.02uF   | 1M   | Input stage |
| V1B   | 12AX7 | 100k  | 820    | 25uF   | 0.02uF   | 1M   | After volume pot |
| V4B   | 12AX7 | 100k  | 1.5k   | 25uF   | 0.1uF    | 220k | Reverb recovery |

- Preamp B+: **270V** (vs Deluxe's 180V -- 50% higher!)
- This is the #1 reason the Twin is cleaner: each tube has ~50% more voltage swing before clipping
- All other component values are identical to the Deluxe Reverb

### Tone Stack

Identical to Deluxe Reverb (same Fender TMB values).

### Power Amp

| Parameter              | Value              |
|------------------------|--------------------|
| Tubes                  | **4x 6L6GC**        |
| Bias                   | Fixed               |
| B+ plate               | **460V** (vs Deluxe's 420V) |
| OT primary impedance   | 2k ohm : 4 ohm     |
| NFB resistor           | 820 ohm             |
| NFB tail               | 100 ohm (some versions 47 ohm) |
| Rectifier              | **Solid-state** (no sag) |

### Key Character: Why Is the Twin So Clean?

Three factors combine to make the Twin the cleanest amp here:

1. **Preamp B+ = 270V** (Deluxe = 180V): Each 12AX7 stage has 50% more headroom
2. **4x 6L6 power tubes** at 460V: ~85W output, enormous headroom
3. **Solid-state rectifier**: No voltage sag under load, power stays stiff

The Twin stays clean up to about volume 6, while the Deluxe breaks up at volume 4. Same circuit, different operating points.

---

## Summary: What Was Wrong in the Original Presets

| Parameter | Fender Deluxe | Marshall JCM800 | Vox AC30 | Mesa Dual Rec | Fender Twin |
|-----------|---------------|------------------|----------|---------------|-------------|
| Preamp stages (old) | 2 | 3 | 2 | 3 | **1** |
| Preamp stages (correct) | **3** | 3 | 2 | **5** | **3** |
| Power tube (old) | 6L6 | EL34 | **EL34** | 6L6 | 6L6 |
| Power tube (correct) | **6V6** | EL34 | **EL84** | 6L6 | 6L6 |
| Power B+ (old) | 400 | 450 | 380 | 420 | 400 |
| Power B+ (correct) | **420** | **480** | **330** | **460** | **460** |
| Cabinet (old) | 1x12 | 4x12 | **1x12** | 4x12 | 2x12 |
| Cabinet (correct) | 1x12 | 4x12 | **2x12** | 4x12 | 2x12 |
| Interstage atten (old) | 0.35 | 0.35 | 0.35 | 0.35 | 0.35 |
| Interstage atten (correct) | **2.0 dB** | **4.7 dB** | **1.7 dB** | **2.0 dB** | **2.0 dB** |

The biggest errors were:
- **Fender Twin had 1 preamp stage** -- it actually has 3 (same as Deluxe)
- **Mesa Dual Rec had 3 stages** -- it actually has 5 on the high-gain channel
- **Vox AC30 used EL34** -- it uses EL84 (we lack Koren constants for EL84)
- **Vox AC30 had 1x12 cabinet** -- it has a 2x12 (Celestion Blue/Silver)
- **interstage_atten was 0.35 for all** -- this value had no clear unit; the actual coupling loss varies from 1.7 dB (Vox) to 4.7 dB (Marshall)
