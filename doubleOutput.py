import serial
import time
import struct
import numpy as np
from mss import mss
import threading
import queue
from math import sqrt 
from os import _exit
import random

SHIFT = 0
NUM_LEDS = 100
COM_PORT = "COM8"       
BAUDRATE = 1000000  
START = 0xAA
FPS = 20
GAMMA = 2.4
SATURATION_BOOST = 3
MAX_BRIGHTNESS = 255
brightness = 0.3

PC_NUM_LEDS = 25
PC_START = 0xBB

stop_flag = threading.Event()
current_effect = None
effect_lock = threading.Lock()

GAMMA_LUT = np.array(
    [int(((i / 255.0) ** (1.0 / GAMMA)) * 255) for i in range(256)],
    dtype=np.uint8
)

SER = serial.Serial(COM_PORT, BAUDRATE, write_timeout=0)
frame_queue = queue.Queue(maxsize=2)

#times = []

def main():
    previous_colors = None

    y_idx, x_idx = get_samples()
    
    sct = mss()
    monitor = sct.monitors[1]
    sct.with_cursor = False

    threading.Thread(target=serial_writer, daemon=True).start()
    threading.Thread(target=pc_led, daemon=True).start()
    threading.Thread(target=effects, daemon=True).start()
    prefered_frame_time = 1.0 / FPS

    while not stop_flag.is_set():
        start = time.perf_counter()
        #print("start:", time.perf_counter() - start)
        loop_start = time.perf_counter()

        sct_img = sct.grab(monitor)
        # Konwersja BGRA -> RGB 
        img = np.frombuffer(sct_img.bgra, dtype=np.uint8).reshape(sct_img.height, sct_img.width, 4)
        img = img[:, :, :3]

        raw_colors = img[y_idx, x_idx] 
        
        avg_bgr = np.mean(raw_colors, axis=1)

        gray = np.mean(avg_bgr, axis=1, keepdims=True)
        processed = gray + (avg_bgr - gray) * SATURATION_BOOST
        

        processed *= [0.2,0.3,1] 
        processed *= [brightness,brightness,brightness]
        processed = np.clip(processed, 0, MAX_BRIGHTNESS).astype(np.uint8)
        final_rgb = GAMMA_LUT[processed][:, ::-1] # Konwersja BGR -> RGB przez slice

        if previous_colors is not None:
           final_rgb = smooth_colors(final_rgb,previous_colors)
        payload = final_rgb.tobytes()
        previous_colors = final_rgb.copy()
        # Budowa ramki: START (1b) + LEN (2b) + PAYLOAD (300b) + CRC (1b)
        full_frame = gen_frame(START, payload)
        
        put_to_frame_queue(full_frame)

        # Kontrola FPS
        elapsed = time.perf_counter() - loop_start
        sleep_time = prefered_frame_time - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

        #times.append(time.perf_counter() - start)
        #print("średnia:", np.average(times))
    
    print("Zatrzymywanie ambilight...")
    time.sleep(0.2)  # Poczekaj na ostatnie ramki
    
    # Wyczyść kolejkę
    while not frame_queue.empty():
        try:
            frame_queue.get_nowait()
            frame_queue.task_done()
        except queue.Empty:
            break
    
    # Wyślij czarne ramki
    clear_all()
    frame_queue.join()  # Poczekaj aż zostaną wysłane
    time.sleep(0.1)
    
    print("LEDs wyłączone.")

def pc_led():
    global current_effect
    while True:
        key_pressed = input("Podaj tryb: ")
        colors = None
        match key_pressed:
            case "white":
                with effect_lock:
                    current_effect = None
                colors = [255 for i in range(PC_NUM_LEDS*3)]
            case "stop":
                stop_flag.set()
            case "rainbow":
                with effect_lock:
                    current_effect = "rainbow"
            case "shoot":
                with effect_lock:
                    current_effect = "shoot"
            case _:
                pass
        
        if colors != None:
            frame = gen_frame(PC_START, colors)

            put_to_frame_queue(frame)

def effects():
    color = (0,0,0)
    hue = 0
    ammo = 0
    max_fps = 40
    while not stop_flag.is_set():
        with effect_lock:
            effect = current_effect

        match effect:
            case "pulse":
                pass
            case "shoot":
                for ammo in range(0,PC_NUM_LEDS*3,3):
                    colors = [0 for _ in range(PC_NUM_LEDS*3)]
                    
                    colors[ammo] = 255

                    if ammo >= 6:
                        colors[ammo-6] = 55
                        colors[ammo-3] = 128
                    elif ammo >=3:
                            colors[ammo-3] = 55
                    else:
                        pass
                    
                    frame = gen_frame(PC_START, colors)
                    put_to_frame_queue(frame)
                    time.sleep(1/6/(ammo+5))

                frame = gen_frame(PC_START, [0 for _ in range(PC_NUM_LEDS*3)])
                put_to_frame_queue(frame)    
                time.sleep(random.randrange(500,1000)/1000)
            case "rainbow":
                colors = []
                for i in range(PC_NUM_LEDS):
                    h = (hue + i * 360 / PC_NUM_LEDS) % 360
                    c = 1.0
                    x = c * (1 - abs((h / 60) % 2 - 1))
                    
                    if h < 60:
                        r, g, b = c, x, 0
                    elif h < 120:
                        r, g, b = x, c, 0
                    elif h < 180:
                        r, g, b = 0, c, x
                    elif h < 240:
                        r, g, b = 0, x, c
                    elif h < 300:
                        r, g, b = x, 0, c
                    else:
                        r, g, b = c, 0, x
                    
                    colors.extend([int(r * 50), int(g * 50), int(b * 50)])

                frame = gen_frame(PC_START, colors)
                put_to_frame_queue(frame)
                
                hue = (hue + 5) % 360
                time.sleep(1/max_fps)
            case _:
                pass

def put_to_frame_queue(frame):
    try:
        frame_queue.put(frame)
    except:
        try:
            frame_queue.get_nowait()
            frame_queue.put_nowait(frame)
        except queue.Empty:
            pass


def gen_frame(start, payload):
    try:
        header = struct.pack('<BH', start, len(payload))
        crc = bytes([crc8(payload)])
        full_frame = header + payload + crc
        return full_frame
    except:
        payload = bytes(payload)
        header = struct.pack('<BH', start, len(payload))
        crc = bytes([crc8(payload)])
        full_frame = header + payload + crc
        return full_frame
def serial_writer():
    while True:
        frame = frame_queue.get()
        SER.write(frame)
        frame_queue.task_done()

def smooth_colors(colors, prev, alpha=100):
    return ((prev.astype(np.uint16) * alpha +
             colors.astype(np.uint16) * (255 - alpha)) >> 8).astype(np.uint8)

def crc8(data: bytes, poly=0x07):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def get_samples(proportion=[33,17,33,17], width=2560, height=1440, geometry_length=100):
    led_samples_x = []
    led_samples_y = []

    seg_w = width / proportion[0]
    seg_h = (height - 2 * SHIFT) / proportion[1]

    OFFSETS = np.array([
        [0.50, 0.50],  # środek

        [0.25, 0.50], [0.75, 0.50],  # lewo / prawo
        [0.50, 0.25], [0.50, 0.75],  # góra / dół

        [0.20, 0.20], [0.80, 0.20],  # rogi wewnętrzne
        [0.20, 0.80], [0.80, 0.80],

        [0.50, 0.10], [0.50, 0.90],  # góra / dół blisko krawędzi
        [0.10, 0.50], [0.90, 0.50],  # lewo / prawo blisko krawędzi
    ])

    def add_rect(x1, y1, x2, y2):
        w = x2 - x1
        h = y2 - y1

        xs = x1 + OFFSETS[:, 0] * w
        ys = y1 + OFFSETS[:, 1] * h

        led_samples_x.append(xs)
        led_samples_y.append(ys)

    # Bottom
    for i in range(proportion[0]):
        add_rect(
            i * seg_w,
            height - geometry_length - SHIFT,
            (i + 1) * seg_w,
            height - SHIFT
        )

    # Right
    for i in range(proportion[1]):
        add_rect(
            width - geometry_length,
            height - SHIFT - i * seg_h,
            width,
            height - SHIFT - (i + 1) * seg_h
        )

    # Top
    for i in range(proportion[2]):
        add_rect(
            width - i * seg_w,
            SHIFT,
            width - (i + 1) * seg_w,
            SHIFT + geometry_length
        )

    # Left
    for i in range(proportion[3]):
        add_rect(
            0,
            i * seg_h + SHIFT,
            geometry_length,
            (i + 1) * seg_h + SHIFT
        )

    return (
        np.array(led_samples_y, dtype=np.int32),
        np.array(led_samples_x, dtype=np.int32)
    )

def clear_all():
    payload1 = bytes([0 for _ in range(NUM_LEDS*3)])
    payload2 = bytes([0 for i in range(PC_NUM_LEDS*3)])
    
    put_to_frame_queue(gen_frame(START, payload1))
    put_to_frame_queue(gen_frame(PC_START, payload2))


    
if __name__ == '__main__':
    main()