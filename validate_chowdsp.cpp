/*
 * validate_chowdsp.cpp
 * Independent WDF validation using chowdsp_wdf library.
 *
 * Builds the same triode circuit as our Python/Verilog using
 * chowdsp_wdf's adaptor infrastructure + our Koren tube model.
 *
 * Compile: g++ -std=c++17 -O2 -I chowdsp_wdf/include validate_chowdsp.cpp -o validate_chowdsp
 * Run:     ./validate_chowdsp
 */

#define _USE_MATH_DEFINES
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <array>
#include <vector>

// chowdsp_wdf — header-only WDF library
#include "chowdsp_wdf/chowdsp_wdf.h"

using namespace chowdsp::wdft;

// ═══════════════════════════════════════════════════════════════════════
// Circuit Parameters
// ═══════════════════════════════════════════════════════════════════════

constexpr double VB  = 200.0;      // B+ supply (V)
constexpr double RP  = 100000.0;   // Plate resistor
constexpr double RG  = 1000000.0;  // Grid resistor
constexpr double RK  = 1500.0;     // Cathode resistor
constexpr double CIN = 22e-9;      // Coupling cap
constexpr double CK  = 22e-6;      // Cathode bypass cap
constexpr double FS  = 48000.0;    // Sample rate

// Koren 12AX7 parameters (same as our Python/Verilog)
constexpr double MU  = 100.0;
constexpr double EX  = 1.4;
constexpr double KG1 = 1060.0;
constexpr double KP  = 600.0;
constexpr double KVB = 300.0;

// ═══════════════════════════════════════════════════════════════════════
// Koren Triode Model
// ═══════════════════════════════════════════════════════════════════════

double koren_ip(double vpk, double vgk)
{
    vpk = std::clamp(vpk, 0.0, 500.0);
    vgk = std::clamp(vgk, -10.0, 1.0);
    if (vpk <= 0.0) return 0.0;

    double inner = KP * (1.0/MU + vgk / std::sqrt(KVB + vpk*vpk));
    inner = std::clamp(inner, -500.0, 500.0);
    double Ed = (vpk / KP) * std::log1p(std::exp(inner));
    if (Ed <= 0.0) return 0.0;
    return std::pow(Ed, EX) / KG1;
}

double koren_dip_dvpk(double vpk, double vgk, double h = 0.01)
{
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2.0 * h);
}

double koren_dip_dvgk(double vpk, double vgk, double h = 0.01)
{
    return (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2.0 * h);
}

// ═══════════════════════════════════════════════════════════════════════
// WDF Simulation using chowdsp_wdf library adaptors
// ═══════════════════════════════════════════════════════════════════════

struct TriodeStage
{
    // One-port elements
    ResistorT<double> Rk { RK };
    CapacitorT<double> Ck { CK, FS };

    // Cathode parallel adaptor
    WDFParallelT<double, ResistorT<double>, CapacitorT<double>> cathode { Rk, Ck };

    // Port resistances for root solver
    double R_plate = RP;
    double R_cathode;

    // HP filter state (coupling cap)
    double hp_y = 0.0;
    double hp_x_prev = 0.0;
    double c_hp;
    double hp_gain;

    // Newton state
    double ip_prev = 0.5e-3;

    TriodeStage()
    {
        R_cathode = cathode.wdf.R;

        // Bilinear HP filter coefficients
        double tau = RG * CIN;
        double k = 2.0 * FS * tau;
        c_hp = (k - 1.0) / (k + 1.0);
        hp_gain = (1.0 + c_hp) / 2.0;
    }

    void reset()
    {
        Ck.reset();
        hp_y = 0.0;
        hp_x_prev = 0.0;
        ip_prev = 0.5e-3;
    }

    double process(double vin)
    {
        // ── HP coupling filter ──
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev);
        hp_x_prev = vin;
        double v_grid = hp_y;

        // ── Upward pass ──
        double a_p = VB;       // plate: Thevenin source
        double a_g = v_grid;   // grid: filtered input

        // Cathode: use chowdsp parallel adaptor
        double a_k = cathode.reflected();  // upward reflected wave

        // ── Newton-Raphson at triode root ──
        double R_p = R_plate;
        double R_k = R_cathode;

        double Ip = ip_prev;
        for (int iter = 0; iter < 20; iter++)
        {
            double Vpk = (a_p - a_k) - (R_p + R_k) * Ip;
            double Vgk = a_g - a_k - R_k * Ip;

            double ip_model = koren_ip(Vpk, Vgk);
            double f = Ip - ip_model;

            if (std::abs(f) < 1e-10) break;

            double dip_vpk = koren_dip_dvpk(Vpk, Vgk);
            double dip_vgk = koren_dip_dvgk(Vpk, Vgk);
            double fp = 1.0 + dip_vpk * (R_p + R_k) + dip_vgk * R_k;

            if (std::abs(fp) < 1e-15) break;

            Ip -= f / fp;
            Ip = std::max(Ip, 0.0);
        }
        ip_prev = Ip;

        // ── Root scattering ──
        double b_p = a_p - 2.0 * R_p * Ip;
        double b_k = a_k + 2.0 * R_k * Ip;

        // ── Downward pass: push wave into cathode adaptor ──
        cathode.incident(b_k);

        // ── Output: plate voltage ──
        double v_plate = (a_p + b_p) / 2.0;
        return v_plate;
    }
};

// ═══════════════════════════════════════════════════════════════════════
// Main — run simulation, compare against Python reference
// ═══════════════════════════════════════════════════════════════════════

int main()
{
    printf("=== chowdsp_wdf Validation ===\n\n");

    TriodeStage stage;

    printf("Port resistances:\n");
    printf("  Plate:   %.1f ohm\n", stage.R_plate);
    printf("  Cathode: %.4f ohm (Rk || Ck via chowdsp)\n", stage.R_cathode);
    printf("  HP: c=%.6f, gain=%.6f\n\n", stage.c_hp, stage.hp_gain);

    // Simulation parameters
    const int n_settle = 2000;
    const int n_audio = 4800;
    const int n_total = n_settle + n_audio;

    std::vector<double> audio_in(n_total, 0.0);
    std::vector<double> vplate(n_total);
    std::vector<double> ip_out(n_total);

    // Generate 440Hz sine after settling
    for (int n = n_settle; n < n_total; n++)
    {
        double t = (double)(n - n_settle) / FS;
        audio_in[n] = 0.5 * std::sin(2.0 * M_PI * 440.0 * t);
    }

    // Run simulation
    printf("Simulating %d samples...\n", n_total);
    for (int n = 0; n < n_total; n++)
    {
        vplate[n] = stage.process(audio_in[n]);
    }

    // DC operating point
    double vp_dc = 0.0;
    for (int n = n_settle - 100; n < n_settle; n++)
        vp_dc += vplate[n];
    vp_dc /= 100.0;

    printf("\n=== DC Operating Point (chowdsp_wdf) ===\n");
    printf("Vplate_dc = %.2fV\n", vp_dc);

    // AC analysis
    double in_sum2 = 0, out_sum2 = 0;
    double out_min = 1e9, out_max = -1e9;
    for (int n = n_settle; n < n_total; n++)
    {
        double in_v = audio_in[n];
        double out_v = vplate[n] - vp_dc;
        in_sum2 += in_v * in_v;
        out_sum2 += out_v * out_v;
        out_min = std::min(out_min, out_v);
        out_max = std::max(out_max, out_v);
    }
    double in_rms = std::sqrt(in_sum2 / n_audio);
    double out_rms = std::sqrt(out_sum2 / n_audio);
    double gain_db = 20.0 * std::log10(out_rms / (in_rms + 1e-12));

    printf("\n=== Audio (chowdsp_wdf) ===\n");
    printf("Input:  rms=%.3fV\n", in_rms);
    printf("Output: min=%.2fV  max=%.2fV  rms=%.2fV\n", out_min, out_max, out_rms);
    printf("Gain:   %.1f dB\n", gain_db);

    // Compare against expected
    double expected_gain = 34.1;
    double gain_err = std::abs(gain_db - expected_gain);

    printf("\n=== Comparison vs Python WDF Reference ===\n");
    printf("Expected gain: %.1f dB\n", expected_gain);
    printf("chowdsp gain:  %.1f dB\n", gain_db);
    printf("Delta:         %.1f dB\n", gain_err);

    bool pass = gain_err < 3.0;
    printf("Result:        %s (threshold: 3 dB)\n", pass ? "PASS" : "FAIL");

    // Write output for cross-validation
    FILE* fp = fopen("chowdsp_output.txt", "w");
    if (fp)
    {
        for (int n = 0; n < n_total; n++)
        {
            int in_q16 = (int)(audio_in[n] * 65536.0);
            int out_q16 = (int)((vplate[n] - vp_dc) * 65536.0);
            fprintf(fp, "%d %d %d\n", n, in_q16, out_q16);
        }
        fclose(fp);
        printf("\nSaved: chowdsp_output.txt\n");
    }

    return pass ? 0 : 1;
}
