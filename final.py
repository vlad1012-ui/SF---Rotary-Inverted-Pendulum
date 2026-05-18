import time
import math
import numpy as np
import serial
import sys
import json  # Added for file saving

# ── CONFIG ────────────────────────────────────────────────────────────────────
EBB_PORT = "COM4"
ESP_PORT = "COM12"
BAUD     = 115200
ANGLE_OFFSET = -1.5

# ── PARAMETERS ────────────────────────────────────────────────────────────────
ENERGY_GAIN      = 8500000.0
SLEW_LIMIT_SWING = 85
MP, LP, G         = 0.0045, 0.083, 9.81

KP_BALANCE       = 36500.0
KD_BALANCE       = 2250.0
SLEW_LIMIT_BAL   = 500
MAX_SPEED        = 6000
DEADZONE         = 2.0

HANDOFF_DEG      = 25.0
MAX_CATCH_VEL    = 2000.0
FALL_LIMIT       = 45.0

# ── SCIENCE FAIR SETTINGS ─────────────────────────────────────────────────────
TOTAL_TRIALS_TARGET = 100
HOLD_REQUIRED_SEC   = 4.0
TIMEOUT_SEC         = 60.0

# ── INITIALIZATION ────────────────────────────────────────────────────────────
ebb = serial.Serial(EBB_PORT, BAUD, timeout=0.01)
esp = serial.Serial(ESP_PORT, BAUD, timeout=0.01)

mass_pos = input("Enter mass position (0-8): ")
filename = f"results_pos_{mass_pos}.json"

# RESUME LOGIC: Load existing data if file exists, otherwise start fresh
try:
    with open(filename, "r") as f:
        saved_data = json.load(f)
        results = saved_data.get("trials", [])
        trial_num = len(results) + 1
        print(f"Found existing file! Resuming from Trial {trial_num}...")
except:
    results = [] 
    trial_num = 1

def run_startup_kick():
    print("\nSystem Ready. Starting Kick...")
    ebb.write(b"V2000\n");  time.sleep(0.1)
    ebb.write(b"V0\n");     time.sleep(0.25)
    ebb.write(b"V-2000\n"); time.sleep(0.1)
    ebb.write(b"V0\n")
    print("Kick Complete.")

def reset_and_wait():
    print("\n-> Resetting: Flicking arm down...")
    ebb.write(b"V5000\n"); time.sleep(0.15); ebb.write(b"V0\n")
    print("-> Waiting 6s for gravity stabilization...")
    time.sleep(6.0)

def save_progress_to_json():
    """Calculates the current average and saves all trials instantly to disk"""
    # Safe key fallback reader handles both 'time' and old 'time_to_balance_sec' files smoothly
    avg_time = sum(item.get("time", item.get("time_to_balance_sec", 0)) for item in results) / len(results) if results else 0
    export_data = {
        "mass_position": mass_pos,
        "average_time": round(avg_time, 3),
        "total_trials": len(results),
        "parameters": {
            "KP": KP_BALANCE,
            "KD": KD_BALANCE,
            "energy_gain": ENERGY_GAIN
        },
        "trials": results
    }
    with open(filename, "w") as f:
        json.dump(export_data, f, indent=4)

# ── MAIN TEST LOOP ────────────────────────────────────────────────────────────
try:
    while trial_num <= TOTAL_TRIALS_TARGET:
        reset_and_wait()
        run_startup_kick()
        
        mode = "swingup"
        start_time = time.time()
        balance_start_time = None
        current_vel = 0
        last_p_deg = 0.0
        last_time = time.time()
        current_slew = SLEW_LIMIT_SWING
        
        trial_active = True
        while trial_active:
            now = time.time()
            dt = max(now - last_time, 1e-6)
            elapsed = now - start_time

            if elapsed > TIMEOUT_SEC:
                print(f"\n[!] Timeout. Retrying Trial {trial_num}...")
                trial_active = False
                continue

            line = esp.readline().decode('utf-8', errors='ignore').strip()
            if not line or not line.startswith("A:"): continue
            
            try:
                raw_angle = float(line.replace("A:", ""))
                p_deg = raw_angle - ANGLE_OFFSET
            except: continue

            theta = math.radians(p_deg)
            theta_dot = (math.radians(p_deg - last_p_deg)) / dt
            last_p_deg, last_time = p_deg, now

            if mode == "swingup":
                if abs(p_deg) < HANDOFF_DEG and abs(math.degrees(theta_dot)) < MAX_CATCH_VEL:
                    mode = "balance"
                    current_slew = SLEW_LIMIT_BAL
                else:
                    current_slew = SLEW_LIMIT_SWING
            else:
                if abs(p_deg) > FALL_LIMIT:
                    mode = "swingup"
                    current_slew = SLEW_LIMIT_SWING
                    balance_start_time = None

            if mode == "balance":
                if abs(p_deg) < DEADZONE:
                    target_vel = int(current_vel * 0.85)
                else:
                    u = KP_BALANCE * theta + KD_BALANCE * theta_dot
                    target_vel = int(np.clip(u, -MAX_SPEED, MAX_SPEED))
                
                if balance_start_time is None:
                    balance_start_time = now
                elif (now - balance_start_time) >= HOLD_REQUIRED_SEC:
                    final_time = round((now - start_time) - HOLD_REQUIRED_SEC, 3)
                    
                    # Appending both variants keeps older parsing/plotting files working seamlessly
                    results.append({
                        "trial": trial_num, 
                        "time": final_time,
                        "time_to_balance_sec": final_time
                    })
                    
                    # Save immediately on completion of this specific trial
                    save_progress_to_json()
                    
                    print(f"\n[✔] TRIAL {trial_num} SAVED: {final_time}s")
                    trial_num += 1
                    trial_active = False
            else:
                E = MP * G * (LP / 2) * (math.cos(theta) + 1)
                if abs(theta_dot) < 0.1:
                    target_vel = 4000 if p_deg > 0 else -4000
                else:
                    target_vel = int(-ENERGY_GAIN * E * math.cos(theta) * np.sign(theta_dot))

            diff = target_vel - current_vel
            current_vel += int(np.clip(diff, -current_slew, current_slew))
            ebb.write(f"V{current_vel}\n".encode())

            if int(now * 10) % 2 == 0:
                sys.stdout.write(f"\r\033[KTrial {trial_num} | Mode: {mode.upper()} | Angle: {p_deg:6.2f} | Time: {elapsed:4.1f}s")
                sys.stdout.flush()

except KeyboardInterrupt:
    ebb.write(b"V0\n")
    print("\n\nStopped.")

# ── FINAL DATA REPORT ─────────────────────────────────────────────────────────
if results:
    avg_time = sum(item.get("time", item.get("time_to_balance_sec", 0)) for item in results) / len(results)
    print("\n" + "="*40)
    print(f" FINAL DATA SUMMARY SAVED TO {filename}")
    print("="*40)
    print(f"AVERAGE BALANCE TIME: {avg_time:.3f}s")
    print("="*40)
else:
    print("\nNo data collected.")

ebb.close()
esp.close()