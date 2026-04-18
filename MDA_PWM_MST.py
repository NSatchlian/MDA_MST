import threading
import time
from datetime import datetime
from collections import deque
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure



# =========================================================
# CODE SUMMARY 
# =========================================================
"""
TEAM : MST- Mechanical Structural and Thermal 

Members:
        Jacob Brochu 
        Marie-Pier Chabot
        Noha EL Jarissi 
        Omayma Essbaa
        Kodjo Joshua Mercy N’Tsougan
        Issam Meriem
        Juliane Monette
        Nathan Satchlian 

This code's purpose is to create a thermal control system (TCS). For this stage, only a PWM was used; however, the next step anticipates the use of a PID for more
dynamic power control. Please read the report for the full proccess flow of thee code and how it interacts with the other software. 

"""
# =========================================================
# =========================================================
# Serial settings
# =========================================================
"""
These are the ports etc. to choose when laucnhing your code. Important to note that "COM" ports change wildly on 
your computer and which port you plug etc. 

The header and frame_length info are basically because python needs to read info from the thermometer, but because the info comes in 
as a massive continuous clump of bytes, we need to tell it what thee header is (i.e signal for the message starts here) and how long
the message is (Frame_len). The b\ , is just to tell python I am not writing string, but these are bytes in hexadecimal. 

"""
# =========================================================
TEMP_PORT = "COM1"
TEMP_BAUD = 9600

ESP32_PORT = "COM6"
ESP32_BAUD = 115200

HEADER = b"\x55\xAA"
FRAME_LEN = 12

# =========================================================
# PWM output pin names
# =========================================================
"""
These are the digital pins on the ESP32. IF you change something on the breakout board, you have to update it in the code to update here. 
It doessnt happen automagically. 
"""
# =========================================================
GPIO_R1 = 5
GPIO_R2 = 18
GPIO_FAN = 19




# =========================================================
# Shared state
# =========================================================
"""
Initial conditions for shareed variables. Thread locking allows to prevent read/write at same time.
CH1 = Temp at primary heater 
CH2 = Temp at auxiliary heaters
CH3 = Temp at radiative surface 
CH4 = AMBIANT room temp

PWM => Goes from 0 to 255 ; where its basically what fraction (out of 256) of the time is the voltage sent 
        that fraction creates a new "effective" potential. i.e 128/256 = ON half of the time so 24V -> 12V effective 
            because 24V is ON only half the time.


r1_mode = 'auto' to make it default in auto mode. 
"""
# =========================================================

running = True  # for threading lock to be active during code 
lock = threading.Lock()

latest_frame = None
latest_timestamp = None
latest_temps = {
    "ch1": None,
    "ch2": None,
    "ch3": None,
    "ch4": None,
}

latest_pwm = {
    "r1": 0,
    "r2": 0,
    "fan": 0,
}

latest_avg_temp = None
last_command_sent = None

temp_ser = None
esp32_ser = None

# GUI mode for R1
r1_mode = "auto"   # "auto" or "manual"

# =========================================================
# Auto heater hysteresis settings
"""
# =========================================================
IMPORTANT:
Auto control is ONLY based on CH1 (not avg CH1, CH2) for now.. That's because, we technically would have needed to calibrate 
Thermocouple positionning with our math models, and while possible, it just didnt fit our timeframe. 

Temps of 24 and 26°C were determined as physical limitations of the device. Higher temps would take too long and/or make them unsafe 
with stronger resistors. We wanted something warmer than ambiant, but not soooo much that it would take a large amount of time 
to showcase in a display setting
"""
# =========================================================
AUTO_TEMP_MIN = 24.0   # turn ON below this
AUTO_TEMP_MAX = 26.0   # turn OFF at or above this
heater_auto_on = False



# =========================================================
# Realtime plot history

"""
Esssentially everything you need to plot on GUI. 
Deque => double ended queue. Because we want to show a real-time graph, if we storeed all data points for the entire time of the run,
we would end up in real hot water after a certain point of run-time. What this does is hold a slightly larger size of data points than
the plot window and deletes the old data as you go (think of it like a conveyor belt on a treadmil, no pas , no future, you can 
just see the tread that is available to you).

"""
# =========================================================
MAX_PLOT_POINTS = 1200
PLOT_WINDOW_SECONDS = 600

plot_time = deque(maxlen=MAX_PLOT_POINTS)
plot_ch1_temp = deque(maxlen=MAX_PLOT_POINTS)
plot_ch2_temp = deque(maxlen=MAX_PLOT_POINTS)
plot_r1_pwm_percent = deque(maxlen=MAX_PLOT_POINTS)
plot_start_time = time.time()

# =========================================================
# Helpers
# =========================================================

"""
MULTIPLE DIFF DEFS/ code to keep main code simpler to read/manage

1. list_port: 
        lists all ports detected on computer -> for deebugging

2. decode_signeed_16_le 

        e.g. packet : 55 AA 01 0B  9E 00  85 00  7A 00  64 00
        
            1. 55 AA = header info 
            2. 01 0B = packet metadata
            3. 9E 00 = CH1
            4. 85 00 = CH2 
            5. 7A 00 = CH3
            6. 64 00 = CH4

        2 bytes per channel => first two, = low bytes (i.e 16 BITS), last two = high bytes (that way you can go over 256 °C and 
                                have negative valuess as well)

        e.g.    for 85 01    (i.e 0x85 0x01  /hexadecimal notation)
                raw = hi(01) * 256 + lo (85) = 341   which is equivalent in code to : raw = (hi << 8) | lo
                real temp = raw/10   (because you cant really put decimal points in heaxdecimal (even if in name) therefore gotta code
                        in the post processing).


3. decode_temperatures
        Basically " just give me something when you actually have a beginning of a frame" 

4. pwm_to_percent
        To convert what user sees in GUI, vs bit info

5. pwm_to_cfm 
        Just for extimated CFM on GUI. 12V case fan rated at 38CFM at max power output so we went from there


6. compute_avg_temp
        Still there but mostly for GUI at this point, because didnt have time to do better calibration of entire box

7. hysterisis_heater_control
        "Hysterisis",because bassically you have the potion of temp between your upper and inner temp bound where you basically 
            say " keep doing what youre doing until you  meet one of the bounds" 

8. send_all_pwr
        Makes sure power sent is between 0 and 255. (The sender ; technically should write it after 9. apply_crurrent_outputs) 

9. apply_current_outputs
        Jusst makes sure multhread. that we dont write over while reading output being sent (The reader)


"""
# =========================================================

def list_ports():
    print("Ports detected:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} - {p.description}")
    print()


def decode_signed_16_le(lo: int, hi: int) -> float:
    raw = (hi << 8) | lo
    if raw >= 0x8000:  #middle point of hexadecimal. Anything below it is + , and anything above is (-)
        raw -= 0x10000
    return raw / 10.0


def decode_temperatures(frame: bytes):
    if len(frame) != FRAME_LEN:
        return None
    if frame[:2] != HEADER:
        return None

    b = list(frame)

    return {
        "ch1": decode_signed_16_le(b[4], b[5]),
        "ch2": decode_signed_16_le(b[6], b[7]),
        "ch3": decode_signed_16_le(b[8], b[9]),
        "ch4": decode_signed_16_le(b[10], b[11]),
    }


def pwm_to_percent(pwm: int) -> float:
    return (pwm / 255.0) * 100.0


def pwm_to_cfm(pwm: int) -> float:
    return (pwm / 255.0) * 38.0


def compute_avg_temp(ch1, ch2):
    if ch1 is not None and ch2 is not None:
        return (ch1 + ch2) / 2.0
    if ch1 is not None:
        return ch1
    if ch2 is not None:
        return ch2
    return None


def hysteresis_heater_control(temp_c: float) -> int:
    global heater_auto_on

    if temp_c is None:
        return 0

    if temp_c < AUTO_TEMP_MIN:
        heater_auto_on = True
    elif temp_c >= AUTO_TEMP_MAX:
        heater_auto_on = False

    return 255 if heater_auto_on else 0


def send_all_pwm(r1: int, r2: int, fan: int):
    global last_command_sent, esp32_ser

    if esp32_ser is None or not esp32_ser.is_open:
        return

    r1 = max(0, min(255, int(r1)))
    r2 = max(0, min(255, int(r2)))
    fan = max(0, min(255, int(fan)))

    cmd = f"R1={r1},R2={r2},FAN={fan}\n"

    if cmd != last_command_sent:
        try:
            esp32_ser.write(cmd.encode())
            esp32_ser.flush()
            last_command_sent = cmd
            print(f"[ESP32] Sent -> {cmd.strip()}")
        except Exception as e:
            print(f"[ESP32 write error] {e}")


def apply_current_outputs():
    with lock:
        r1 = latest_pwm["r1"]
        r2 = latest_pwm["r2"]
        fan = latest_pwm["fan"]

    send_all_pwm(r1, r2, fan)


# =========================================================
# GUI callbacks
# =========================================================

"""
These are essentially functions of what happens wheen someone interactis with the GUI

1. on_r1_slider : basically locks out user function when outside of manual mode for theermal heaters
2. 




"""
# =========================================================
def on_r1_slider(value):
    global r1_mode
    if r1_mode != "manual":
        return

    pwm = int(float(value))
    with lock:
        latest_pwm["r1"] = pwm
    apply_current_outputs()


def on_r2_slider(value):
    pwm = int(float(value))
    with lock:
        latest_pwm["r2"] = pwm
    apply_current_outputs()


def on_fan_slider(value):
    pwm = int(float(value))
    with lock:
        latest_pwm["fan"] = pwm
    apply_current_outputs()


def set_r1_mode():
    global r1_mode, heater_auto_on
    r1_mode = r1_mode_var.get()

    if r1_mode == "manual":
        r1_slider.state(["!disabled"])
    else:
        r1_slider.state(["disabled"])

        # Re-sync hysteresis state cleanly when returning to auto
        with lock:
            ch1_temp = latest_temps["ch1"]

        heater_auto_on = False
        if ch1_temp is not None and ch1_temp < AUTO_TEMP_MIN:
            heater_auto_on = True


# =========================================================
# Worker thread
# =========================================================
"""
Essentially reads serial code from thermal datalogger and decodes it and updataes variables and send info to ESP32.
As mentionned in comments, technically we wanted to do an average of CH1 and CH2 for PWM, but the thermal path was highly resistif (honeycomb panels)
for the amount of heat being transferred sso it was too much of a temp diffeerence for the range of tmepeerature modulation. 

i.e. we were trying to alternatee between 24 and 26°C, but the temp difference between CH1 and CH2 themselves was about 2°C. Either way as testing showed 
the attenuation of the cyclic nature of hysterisis heating was eveen better, because it gave stable temperatures in the further auxiliary devices. 
"""
# =========================================================
def temp_worker():
    global latest_frame, latest_timestamp, latest_avg_temp, running

    buffer = b""
    last_frame = None

    while running:
        try:
            chunk = temp_ser.read(4096)
            if not chunk:
                time.sleep(0.01)
                continue

            buffer += chunk

            while True:
                idx = buffer.find(HEADER)
                if idx == -1:
                    buffer = buffer[-32:]
                    break

                if idx > 0:
                    buffer = buffer[idx:]

                if len(buffer) < FRAME_LEN:
                    break

                frame = buffer[:FRAME_LEN]
                buffer = buffer[FRAME_LEN:]

                if frame == last_frame:
                    continue
                last_frame = frame

                temps = decode_temperatures(frame)
                if temps is None:
                    continue

                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                avg_temp = compute_avg_temp(temps["ch1"], temps["ch2"])

                with lock:
                    latest_frame = frame
                    latest_timestamp = ts
                    latest_temps["ch1"] = temps["ch1"]
                    latest_temps["ch2"] = temps["ch2"]
                    latest_temps["ch3"] = temps["ch3"]
                    latest_temps["ch4"] = temps["ch4"]
                    latest_avg_temp = avg_temp

                    # IMPORTANT:
                    # Auto mode now depends on CH1 ONLY
                    if r1_mode == "auto":
                        latest_pwm["r1"] = hysteresis_heater_control(temps["ch1"])

                    current_r1 = latest_pwm["r1"]
                    current_r2 = latest_pwm["r2"]
                    current_fan = latest_pwm["fan"]

                    elapsed_s = time.time() - plot_start_time
                    plot_time.append(elapsed_s)
                    plot_ch1_temp.append(temps["ch1"])
                    plot_ch2_temp.append(temps["ch2"])
                    plot_r1_pwm_percent.append((current_r1 / 255.0) * 100.0)

                send_all_pwm(current_r1, current_r2, current_fan)

        except Exception as e:
            print(f"[Temp worker error] {e}")
            time.sleep(0.1)


# =========================================================
# GUI update
# =========================================================
"""
Thois part is to modulate what the user sees in the GUI. Slider mouveement, temp reading etc. 

"""
# =========================================================

def update_gui():
    with lock:
        ts = latest_timestamp
        temps = latest_temps.copy()
        pwm = latest_pwm.copy()
        frame = latest_frame
        avg_temp = latest_avg_temp

        x_data = list(plot_time)
        y_temp = list(plot_ch1_temp)
        y_temp_ch2 = list(plot_ch2_temp)
        y_pwm = list(plot_r1_pwm_percent)

    if r1_mode == "auto":
        r1_slider.set(pwm["r1"])

    # ----------------------------
    # Live temperature labels
    # ----------------------------
    if temps["ch1"] is None:
        ch1_temp_label.config(text="CH1: Temperature of Heater : waiting...")
        ch2_temp_label.config(text="CH2: Temperature of OBC: waiting...")
        ch3_temp_label.config(text="CH3: Temperature of Radiative Surface: waiting...")
        ch4_temp_label.config(text="CH4: Ambient Temperature: waiting...")
        avg_temp_label.config(text="Estimated Average Internal Temperature (CH1 and 2): waiting...")
        raw_label.config(text="Raw frame: waiting...")
        time_label.config(text="Timestamp: waiting...")
    else:
        ch1_temp_label.config(text=f"CH1: Temperature of Heater : {temps['ch1']:.1f} °C")
        ch2_temp_label.config(text=f"CH2: Temperature of OBC: {temps['ch2']:.1f} °C")
        ch3_temp_label.config(text=f"CH3: Temperature of Radiative Surface: {temps['ch3']:.1f} °C")
        ch4_temp_label.config(text=f"CH4: Ambient Temperature: {temps['ch4']:.1f} °C")

        if avg_temp is None:
            avg_temp_label.config(text="Estimated Average Internal Temperature (CH1 and 2): waiting...")
        else:
            avg_temp_label.config(
                text=f"Estimated Average Internal Temperature (CH1 and 2): {avg_temp:.1f} °C"
            )

        raw_label.config(text=f"Raw frame: {frame.hex(' ')}")
        time_label.config(text=f"Timestamp: {ts}")

    # ----------------------------
    # PWM labels
    # ----------------------------
    r1_pwm_label.config(
        text=f"Resistance circuit 1 PWM (GPIO {GPIO_R1}): {pwm['r1']}  |  {pwm_to_percent(pwm['r1']):.1f}%"
    )
    r2_pwm_label.config(
        text=f"Resistance circuit 2 PWM (GPIO {GPIO_R2}): {pwm['r2']}  |  {pwm_to_percent(pwm['r2']):.1f}%"
    )
    fan_pwm_label.config(
        text=f"Fan speed (GPIO {GPIO_FAN}): {pwm['fan']}  |  Est. airflow {pwm_to_cfm(pwm['fan']):.1f} CFM"
    )

    # ----------------------------
    # Plot update
    # ----------------------------
    if len(x_data) > 1:
        line_temp.set_data(x_data, y_temp)
        line_ch2.set_data(x_data, y_temp_ch2)
        line_pwm.set_data(x_data, y_pwm)

        latest_x = x_data[-1]

        if latest_x <= PLOT_WINDOW_SECONDS:
            ax_temp.set_xlim(0, PLOT_WINDOW_SECONDS)
        else:
            ax_temp.set_xlim(latest_x - PLOT_WINDOW_SECONDS, latest_x)

        ax_temp.set_ylim(20, 30)
        ax_pwm.set_ylim(-5, 105)

        ax_temp.set_yticks([20, 22, 24, 26, 28, 30])
        ax_pwm.set_yticks([0, 25, 50, 75, 100])

        canvas.draw_idle()

    root.after(200, update_gui)


# =========================================================
# Clean exit
# =========================================================
"""
This may look like nothing, but is actually a super critical important part of the code in terms of safety. 

It essentailly makes sure that that thee serial ports stay locked and that for example the heaters dont randomly turn on after you close thge code .. 

"""
# =========================================================
def on_close():
    global running
    running = False

    try:
        if esp32_ser is not None and esp32_ser.is_open:
            esp32_ser.write(b"R1=0,R2=0,FAN=0\n")
            esp32_ser.flush()
            time.sleep(0.05)
            esp32_ser.close()
    except Exception:
        pass

    try:
        if temp_ser is not None and temp_ser.is_open:
            temp_ser.close()
    except Exception:
        pass

    root.destroy()


# =========================================================
# Main
# =========================================================

"""
Initialize everything before GUI starts: Find ports, connects to devices, resets outputs, starts background threading 
"""
# =========================================================
list_ports()

try:
    temp_ser = serial.Serial(TEMP_PORT, TEMP_BAUD, timeout=0.2)
    print(f"Connected to thermometer on {TEMP_PORT} @ {TEMP_BAUD}")
except Exception as e:
    print(f"Could not open thermometer port {TEMP_PORT}")
    print(e)
    raise

try:
    esp32_ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=1)
    time.sleep(2)
    print(f"Connected to ESP32 on {ESP32_PORT} @ {ESP32_BAUD}")

    esp32_ser.write(b"R1=0,R2=0,FAN=0\n")
    esp32_ser.flush()
    last_command_sent = "R1=0,R2=0,FAN=0\n"

except Exception as e:
    print(f"Could not open ESP32 port {ESP32_PORT}")
    print(e)
    raise

worker = threading.Thread(target=temp_worker, daemon=True)
worker.start()

# =========================================================
# GUI
# =========================================================
"""
It's everything to make the GUI:
    The window setup
    The main containter
    The temp seection 
    Thee control section
    etc.

"""
# =========================================================

root = tk.Tk()
root.title("Thermal Control GUI")
root.geometry("1280x980")
root.resizable(False, False)

main_frame = ttk.Frame(root, padding=12)
main_frame.pack(fill="both", expand=True)

title_label = ttk.Label(main_frame, text="Thermal Control GUI", font=("Arial", 18, "bold"))
title_label.pack(pady=6, anchor="w")

# ----------------------------
# Temperatures
# ----------------------------
temps_frame = ttk.LabelFrame(main_frame, text="Live Temperatures", padding=12)
temps_frame.pack(fill="x", pady=8)

ch1_temp_label = ttk.Label(
    temps_frame,
    text="CH1: Temperature of Heater : waiting...",
    font=("Arial", 11)
)
ch1_temp_label.pack(anchor="w", pady=2)

ch2_temp_label = ttk.Label(
    temps_frame,
    text="CH2: Temperature of OBC: waiting...",
    font=("Arial", 11)
)
ch2_temp_label.pack(anchor="w", pady=2)

ch3_temp_label = ttk.Label(
    temps_frame,
    text="CH3: Temperature of Radiative Surface: waiting...",
    font=("Arial", 11)
)
ch3_temp_label.pack(anchor="w", pady=2)

ch4_temp_label = ttk.Label(
    temps_frame,
    text="CH4: Ambient Temperature: waiting...",
    font=("Arial", 11)
)
ch4_temp_label.pack(anchor="w", pady=2)

spacer_label = ttk.Label(temps_frame, text="", font=("Arial", 6))
spacer_label.pack(anchor="w", pady=2)

avg_temp_label = ttk.Label(
    temps_frame,
    text="Estimated Average Internal Temperature (CH1 and 2): waiting...",
    font=("Arial", 11, "bold")
)
avg_temp_label.pack(anchor="w", pady=4)

# ----------------------------
# Controls
# ----------------------------
controls_frame = ttk.LabelFrame(main_frame, text="PWM Controls", padding=12)
controls_frame.pack(fill="x", pady=8)

controls_left = ttk.Frame(controls_frame)
controls_left.pack(side="left", fill="both", expand=True, padx=(0, 20))

controls_right = ttk.Frame(controls_frame)
controls_right.pack(side="right", fill="y")

# R1
r1_mode_frame = ttk.Frame(controls_left)
r1_mode_frame.pack(fill="x", pady=4)

ttk.Label(
    r1_mode_frame,
    text=f"Resistance circuit 1 control (GPIO {GPIO_R1})",
    font=("Arial", 11, "bold")
).pack(anchor="w")

r1_mode_var = tk.StringVar(value="auto")
ttk.Radiobutton(
    r1_mode_frame,
    text="Automatic temp based",
    variable=r1_mode_var,
    value="auto",
    command=set_r1_mode
).pack(anchor="w")
ttk.Radiobutton(
    r1_mode_frame,
    text="Manual",
    variable=r1_mode_var,
    value="manual",
    command=set_r1_mode
).pack(anchor="w")

r1_pwm_label = ttk.Label(
    controls_left,
    text=f"Resistance circuit 1 PWM (GPIO {GPIO_R1}): 0  |  0.0%",
    foreground="#2a5adc"
)
r1_pwm_label.pack(anchor="w", pady=2)

r1_slider = ttk.Scale(
    controls_left,
    from_=0,
    to=255,
    orient="horizontal",
    length=760,
    command=on_r1_slider
)
r1_slider.pack(anchor="w", pady=6)
r1_slider.set(0)

# R2
ttk.Label(
    controls_left,
    text=f"Resistance circuit 2 control (GPIO {GPIO_R2})",
    font=("Arial", 11, "bold")
).pack(anchor="w", pady=(18, 0))

r2_pwm_label = ttk.Label(
    controls_left,
    text=f"Resistance circuit 2 PWM (GPIO {GPIO_R2}): 0  |  0.0%",
    foreground="#3c9b5a"
)
r2_pwm_label.pack(anchor="w", pady=2)

r2_slider = ttk.Scale(
    controls_left,
    from_=0,
    to=255,
    orient="horizontal",
    length=760,
    command=on_r2_slider
)
r2_slider.pack(anchor="w", pady=6)
r2_slider.set(0)

# FAN
ttk.Label(
    controls_right,
    text=f"Fan control (GPIO {GPIO_FAN})",
    font=("Arial", 11, "bold")
).pack(anchor="w", pady=(8, 0))

fan_pwm_label = ttk.Label(
    controls_right,
    text=f"Fan speed (GPIO {GPIO_FAN}): 0  |  Est. airflow 0.0 CFM",
    foreground="#e68c1e"
)
fan_pwm_label.pack(anchor="w", pady=10)

fan_slider = ttk.Scale(
    controls_right,
    from_=0,
    to=255,
    orient="horizontal",
    length=320,
    command=on_fan_slider
)
fan_slider.pack(anchor="w", pady=6)
fan_slider.set(0)

# ----------------------------
# Real-time plot
# ----------------------------
plot_frame = ttk.LabelFrame(main_frame, text="Real-time CH1 / Heater Duty Plot", padding=12)
plot_frame.pack(fill="x", pady=8)

fig = Figure(figsize=(12, 3.6), dpi=100)
fig.subplots_adjust(left=0.08, right=0.92, bottom=0.18, top=0.90)

ax_temp = fig.add_subplot(111)
ax_pwm = ax_temp.twinx()

line_temp, = ax_temp.plot(
    [], [], linewidth=1.8, color="#2a5adc",
    label="CH1 Temperature (20-30°C)"
)
line_ch2, = ax_temp.plot(
    [], [],
    linewidth=1.6,
    color="#2fa34a",
    label="CH2 Temperature (Internal)"
)
line_pwm, = ax_pwm.plot(
    [], [], linewidth=1.4, color="#e68c1e",
    label="R1 Duty Cycle (% of 24V input)"
)

ax_temp.set_xlabel("Time (s)")
ax_temp.set_ylabel("Temp (°C)", color="#2a5adc", labelpad=14)
ax_pwm.set_ylabel("Duty (% of 24V input)", color="#e68c1e", labelpad=14)

ax_temp.set_ylim(20, 30)
ax_temp.set_yticks([20, 22, 24, 26, 28, 30])

# Slight margin so 0 and 100 do not sit exactly on border
ax_pwm.set_ylim(-20, 120)
ax_pwm.set_yticks([0, 25, 50, 75, 100])

ax_temp.tick_params(axis="y", colors="#2a5adc")
ax_pwm.tick_params(axis="y", colors="#e68c1e")

ax_temp.grid(True, alpha=0.25)

lines = [line_temp, line_ch2, line_pwm]
labels = [line.get_label() for line in lines]
ax_temp.legend(
    lines, labels,
    loc="center left",
    bbox_to_anchor=(0, 1.05),  # push legend OUTSIDE to the right
    frameon=False,
    ncol=3,
    fontsize=9,
    handlelength=1.0,
    handleheight=1.2
)

canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack(fill="x", expand=False)

# ----------------------------
# Footer
# ----------------------------
status_frame = ttk.LabelFrame(main_frame, text="Serial / Frame Info", padding=12)
status_frame.pack(fill="x", pady=8)

time_label = ttk.Label(status_frame, text="Timestamp: waiting...", font=("Arial", 10))
time_label.pack(anchor="w", pady=2)

raw_label = ttk.Label(status_frame, text="Raw frame: waiting...", font=("Arial", 10))
raw_label.pack(anchor="w", pady=2)

note_label = ttk.Label(
    status_frame,
    text="R1 can be automatic or manual. R2 and fan are manual only in this version.",
    font=("Arial", 10)
)
note_label.pack(anchor="w", pady=2)

set_r1_mode()
root.protocol("WM_DELETE_WINDOW", on_close)
root.after(200, update_gui)
root.mainloop()