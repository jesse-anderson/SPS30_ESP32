# ------------------------------------------------------------------
"""
SPS30 Particulate Matter Sensor TEST
==================================================================================

SPS30 library with CRC validation

Copyright 2025 Jesse Anderson

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
import network, ntptime, utime, time, os
from sps30 import SPS30

# ───────── USER SETTINGS ─────────
USE_WIFI       = False          # ← set False for completely offline logging
WIFI_SSID      = "YOUR_ISSID_HERE_RIGHT_HERE"
WIFI_PASSWORD  = "WIFI_PSWD"
MAX_WIFI_TRIES = 6             # total attempts (≈ 3 s * tries)

I2C_SCL_PIN    = 22            # change to your wiring
I2C_SDA_PIN    = 27

MEASURE_EVERY  = 30            # seconds between rows
ROLLING_N      = 3             # 1 = no averaging, N = rolling avg
CSV_FILE       = "results.txt"
# ─────────────────────────────────
# ---------- Wi‑Fi ----------
def connect_wifi(ssid, pwd, tries=MAX_WIFI_TRIES) -> bool:
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    if sta.isconnected():
        print("[WiFi] already connected:", sta.ifconfig()); return True
    sta.disconnect(); sta.connect(ssid, pwd); print("[WiFi] connecting…")
    for _ in range(tries):
        if sta.isconnected():
            print("[WiFi] ✅", sta.ifconfig()); return True
        time.sleep(0.5)
    print("[WiFi] ❌ couldn’t connect after", tries, "tries")
    return False

# ---------- Chicago‑time helpers (no ‘calendar’) ----------
def _weekday(y, m, d):          # Zeller‑congruence variant (0 = Sun)
    t = (0,3,2,5,0,3,5,1,4,6,2,4)
    if m < 3: y -= 1
    return (y + y//4 - y//100 + y//400 + t[m-1] + d) % 7

def _second_sunday_march(year):
    return 1 + ((7 - _weekday(year, 3, 1)) % 7) + 7      # date

def _first_sunday_nov(year):
    return 1 + ((7 - _weekday(year,11, 1)) % 7)

def _is_us_dst(utc):
    y, m, d, h = utc[0], utc[1], utc[2], utc[3]
    if   m < 3 or m > 11:  return False
    elif 3 < m < 11:       return True
    elif m == 3:
        return (d > _second_sunday_march(y)) or (d == _second_sunday_march(y) and h >= 2)
    else:  # m == 11
        return (d < _first_sunday_nov(y))   or (d == _first_sunday_nov(y)   and h < 2)

def sync_ntp(max_tries=3):
    for i in range(max_tries):
        try: ntptime.settime(); print("[NTP] synced"); return True
        except OSError as e: print("[NTP] fail", i+1, e); time.sleep(2)
    print("[NTP] giving up – unsynchronised"); return False

def now_chicago_iso():
    utc = utime.gmtime()
    offset = -5*3600 if _is_us_dst(utc) else -6*3600
    loc = utime.localtime(utime.mktime(utc) + offset)
    return "%04d-%02d-%02dT%02d:%02d:%02d" % loc[:6]

# ---------- CSV ----------
def _file_empty(path):
    try: return os.stat(path)[6] == 0
    except OSError: return True

def csv_header(f):
    f.write("ISO8601,PM1.0,PM2.5,PM4.0,PM10,PC0.5,PC1.0,PC2.5,PC4.0,PC10,TypSize,CRCerrs\n"); f.flush()

def csv_write(f, iso, md, pc, size, crc):
    f.write("{},{:.2f},{:.2f},{:.2f},{:.2f},{},{},{},{},{},{:.2f},{}\n".format(
        iso,
        md["pm1.0"], md["pm2.5"], md["pm4.0"], md["pm10"],
        int(pc["pm0.5"]), int(pc["pm1.0"]), int(pc["pm2.5"]),
        int(pc["pm4.0"]), int(pc["pm10"]),
        size, crc
    )); f.flush()

# ---------- MAIN ----------
def main():
    wifi_ok = False
    if USE_WIFI:
        wifi_ok = connect_wifi(WIFI_SSID, WIFI_PASSWORD)
        if wifi_ok:
            sync_ntp()
    else:
        print("[WiFi] disabled by config")

    if not wifi_ok and USE_WIFI:
        print("[WiFi] OFFLINE mode – timestamps use current RTC value")

    sps = SPS30(scl_pin=I2C_SCL_PIN, sda_pin=I2C_SDA_PIN, debug=True)
    if not sps.start_measurement():
        raise SystemExit("sensor failed to start")
    print("[SPS30] stabilising 8 s…"); time.sleep(8)

    with open(CSV_FILE, "a") as log:
        if _file_empty(CSV_FILE): csv_header(log)
        while True:
            iso = now_chicago_iso()
            frame = sps.get_measurement(ROLLING_N)      # actually dumb.
            if not frame:
                print("[log] bad frame – skipped")
            else:
                md  = frame["sensor_data"]["mass_density"]
                pc  = frame["sensor_data"]["particle_count"]
                size = frame["sensor_data"]["particle_size"]
                crc  = frame["measurement_info"]["crc_errors_this_measurement"]
                csv_write(log, iso, md, pc, size, crc)
                print("[log]", iso, "OK")
                
                # Print all sensor data to terminal
                print("─" * 60)
                print(f"📊 SENSOR DATA @ {iso}")
                print("─" * 60)
                print("💨 MASS DENSITY (µg/m³):")
                print(f"   PM1.0:  {md['pm1.0']:6.2f}    PM2.5:  {md['pm2.5']:6.2f}")
                print(f"   PM4.0:  {md['pm4.0']:6.2f}    PM10:   {md['pm10']:6.2f}")
                print()
                print("🔢 PARTICLE COUNT (#/cm³):")
                print(f"   PC0.5:  {int(pc['pm0.5']):6d}    PC1.0:  {int(pc['pm1.0']):6d}")
                print(f"   PC2.5:  {int(pc['pm2.5']):6d}    PC4.0:  {int(pc['pm4.0']):6d}")
                print(f"   PC10:   {int(pc['pm10']):6d}")
                print()
                print(f"📏 TYPICAL SIZE: {size:.2f} µm")
                print(f"🔍 CRC ERRORS:   {crc}")
                print("─" * 60)
                print()
                
            time.sleep(MEASURE_EVERY)

# entry‑point
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopping logger")
