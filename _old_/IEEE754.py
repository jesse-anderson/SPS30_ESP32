"""
IEEE‑754 single‑precision converters + test harness
===================================================

This file now contains **four** converter functions:

  1. ieee754_to_float            – pure Python bit‑twiddling (correct)
  2. ieee754_to_float_struct     – struct.unpack(">f", …) (correct)
  3. ieee754_number_conversion_old
  4. ieee754_number_conversion_old2
     (the “__ieee754_number_conversion” code you just sent, made stand‑alone)

Only #1 and #2 match the IEEE‑754 spec exactly; the *old* ones are kept for
comparison.  
"""

import struct
import math
import random


# ─────────────────────────────────────────────────────────────────────────────
# 1.  Bit‑twiddling implementation – no heavy imports
# ─────────────────────────────────────────────────────────────────────────────
def ieee754_to_float(u32, *, debug=False):
    """Decode an unsigned 32‑bit IEEE‑754 word to Python float."""
    try:
        u32 &= 0xFFFFFFFF

        sign_bit      =  u32 >> 31
        exponent_bits = (u32 >> 23) & 0xFF
        fraction_bits =  u32 & 0x7FFFFF

        # ±0
        if exponent_bits == 0 and fraction_bits == 0:
            return -0.0 if sign_bit else 0.0
        # ±∞ or NaN
        if exponent_bits == 0xFF:
            if fraction_bits:
                return float("nan")
            return float("inf") * (-1) ** sign_bit

        bias = 127
        if exponent_bits == 0:
            exp      = 1 - bias
            mantissa = fraction_bits / (1 << 23)
        else:
            exp      = exponent_bits - bias
            mantissa = 1 + fraction_bits / (1 << 23)

        return (-1) ** sign_bit * mantissa * (2 ** exp)

    except Exception as exc:
        if debug:
            print(f"🔍 ieee754_to_float error for 0x{u32:08X}: {exc}")
        return 0.0


# ─────────────────────────────────────────────────────────────────────────────
# 2.  struct‑based reference implementation
# ─────────────────────────────────────────────────────────────────────────────
def ieee754_to_float_struct(u32, *, debug=False):
    """Same result as #1, but uses struct.unpack."""
    try:
        raw = (u32 & 0xFFFFFFFF).to_bytes(4, "big")
        return struct.unpack(">f", raw)[0]
    except Exception as exc:
        if debug:
            print(f"🔍 ieee754_to_float_struct error for 0x{u32:08X}: {exc}")
        return 0.0


# ─────────────────────────────────────────────────────────────────────────────
# 3.  Original “old” converter (rounded, incomplete)
# ─────────────────────────────────────────────────────────────────────────────
def ieee754_number_conversion_old(u32, *, debug=False):
    try:
        binary = f"{u32 & 0xFFFFFFFF:032b}"
        sign   = int(binary[0])
        exp    = int(binary[1:9], 2) - 127

        divider = 0
        if exp < 0:
            divider = -exp
            exp = 0

        mantissa = binary[9:]
        real     = int('1' + mantissa[:exp], 2) if exp else 1
        decimal  = mantissa[exp:]

        frac = sum(int(bit) / (2 ** (i + 1)) for i, bit in enumerate(decimal))

        if divider:
            val = ((-1) ** sign * real + frac) / (2 ** divider)
        else:
            val = (-1) ** sign * real + frac

        return round(val, 3)
    except Exception as exc:
        if debug:
            print(f"🔍 ieee754_number_conversion_old error for 0x{u32:08X}: {exc}")
        return 0.0


# ─────────────────────────────────────────────────────────────────────────────
# 4.  “__ieee754_number_conversion” adapted to stand‑alone function
# ─────────────────────────────────────────────────────────────────────────────
def ieee754_number_conversion_old2(u32, *, debug=False):
    try:
        binary = f"{u32 & 0xFFFFFFFF:032b}"

        sign = int(binary[0])
        exp  = int(binary[1:9], 2) - 127

        divider = 0
        if exp < 0:
            divider = -exp
            exp = 0

        mantissa = binary[9:]

        real     = int('1' + mantissa[:exp], 2) if exp else 1
        decimal  = mantissa[exp:]

        frac = sum(int(bit) / (2 ** (i + 1)) for i, bit in enumerate(decimal))

        if divider:
            val = ((-1) ** sign * real + frac) / (2 ** divider)
        else:
            val = (-1) ** sign * real + frac

        return round(val, 3)
    except Exception as exc:
        if debug:
            print(f"🔍 ieee754_number_conversion_old2 error for 0x{u32:08X}: {exc}")
        return 0.0


# ─────────────────────────────────────────────────────────────────────────────
# 5.  Test harness
# ─────────────────────────────────────────────────────────────────────────────
SAMPLE_WORDS = [
    0x00000000, 0x80000000,             # +0, −0
    0x3F800000, 0xBF800000,             # +1, −1
    0x40490FDB, 0xC0490FDB,             # +π, −π
    0x7F800000, 0xFF800000, 0x7FC00000, # +∞, −∞, NaN
    0x3E99999A, 0x3DCCCCCD,             # 0.3, 0.1
    0x00800000, 0x7F7FFFFF              # smallest normal, largest finite
]


def random_words(n=20):
    return [random.getrandbits(32) for _ in range(n)]


def floats_equal(a, b):
    if math.isnan(a) and math.isnan(b):
        return True
    if math.isinf(a) and math.isinf(b):
        return math.copysign(1, a) == math.copysign(1, b)
    if a == b:
        return math.copysign(1, a) == math.copysign(1, b)  # signed zero
    return False  # no tolerance – exact comparison


def test_converters(words=None, *, verbose=True):
    if words is None:
        words = SAMPLE_WORDS

    mismatches = 0
    for w in words:
        results = [f(w) for f in CONVERTERS]
        base = results[0]
        if not all(floats_equal(base, r) for r in results[1:]):
            mismatches += 1
            if verbose:
                print(f"\n❌ 0x{w:08X}")
                for f, r in zip(CONVERTERS, results):
                    print(f"    {f.__name__:30s} → {r!r}")
    if mismatches == 0:
        print("All converters agree on every test word.")
    else:
        print(f"\n{mismatches} mismatching word(s) detected.")


# ─────────────────────────────────────────────────────────────────────────────
# 6.  Registry – add/remove converters here
# ─────────────────────────────────────────────────────────────────────────────
CONVERTERS = [
    ieee754_to_float,
    ieee754_to_float_struct,
    ieee754_number_conversion_old,
    ieee754_number_conversion_old2,
]


# ─────────────────────────────────────────────────────────────────────────────
# 7.  CLI entry point
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Running built‑in sample set …")
    test_converters()

    print("\nRunning 20 random words …")
    test_converters(random_words(20))
