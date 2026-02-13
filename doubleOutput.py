import serial
import time
import struct
import numpy as np
from mss import mss
import threading
import queue
from math import sqrt 
from os import _exit
SHIFT = 0
NUM_LEDS = 100
COM_PORT = "COM8"       
BAUDRATE = 1000000  
START = 0xAA
FPS = 20
GAMMA = 2.4
SATURATION_BOOST = 3
MAX_BRIGHTNESS = 255
brightness = 0.1

PC_NUM_LEDS = 25
PC_START = 0xBB
stop_flag = threading.Event()

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
    sct.with_cursor = False
    monitor = sct.monitors[1]
    
    threading.Thread(target=serial_writer, daemon=True).start()
    threading.Thread(target=pc_led, daemon=True).start()
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
    while True:
        key_pressed = input("Podaj tryb: ")
        colors = None
        match key_pressed:
            case "white":
                colors = [50 for i in range(PC_NUM_LEDS*3)]
            case "stop":
                stop_flag.set()
            case _:
                pass
        
        if colors != None:
            payload = bytes(colors)
            frame = gen_frame(PC_START, payload)

            put_to_frame_queue(frame)



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