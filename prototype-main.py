import serial
import time
import struct
from random import randrange
import webcolors
import numpy as np
from mss import mss
import cv2
import threading
import queue

PROPORTION = [1,1,1,1]
SHIFT = 165
NUM_LEDS = 4
COM_PORT = "COM8"       
BAUDRATE =  500000
START = 0xAA
RGB_TUPLE_FORMAT = '<3B'

SEGMENT_WIDHT = 2560 / NUM_LEDS
FPS = 40
GAMMA = 2.2
SEND = True

GAMMA_LUT = np.array(
    [int(((i / 255.0) ** (1.0 / GAMMA)) * 255) for i in range(256)],
    dtype=np.uint8
)
SER = serial.Serial(COM_PORT, BAUDRATE, write_timeout=None)
ROW_SIZE = struct.calcsize(RGB_TUPLE_FORMAT)
frame_time = 1/FPS
frame_queue = queue.Queue()

state = 0
print(cv2.cuda.getCudaEnabledDeviceCount())

def main():
    

    #debug_rectangles()
    led_samples = get_samples()
    led_points = np.array(led_samples)
    x_idx = led_points[:,:,0].astype(int)
    y_idx = led_points[:,:,1].astype(int)
    previous_colors = None
    for led in led_samples:
        for x,y in led:
            assert 0 <= x < 2560
            assert 0 <= y < 1440
    sct = mss()
    monitor = sct.monitors[1]

    
    writer_thread = threading.Thread(target=serial_writer, daemon=True)
    writer_thread.start()

    
    # checker_thread = threading.Thread(target=checker, daemon=True)
    # checker_thread.start()

    while True:
        start = time.perf_counter()
        
        print("inicjalizacja zegara: ", round(time.perf_counter()-start, 5) )

        colors = get_screen_colors(y_idx,x_idx,monitor,sct)
        print(colors)
        print("get screen colors: ", round(time.perf_counter()-start, 5) )
        colors = avg_colors(colors)
        print("avg colors: ", round(time.perf_counter()-start, 5) )
        colors = postprocess_colors(colors)
        print("postprocess: ", round(time.perf_counter()-start, 5) )
        
        if previous_colors is not None:
           colors = smooth_colors(colors,previous_colors)

        payload = gen_payload(colors)
        frame = gen_frame(payload)
        print("przed wysłaniem: ", round(time.perf_counter()-start, 4) )
        
        #SER.write(frame)
        frame_queue.put(frame)

        previous_colors = colors.copy()

        loop_time = time.perf_counter() - start
        sleep = frame_time - loop_time
        if sleep > 0:
            time.sleep(sleep)
        print("Koniec pętli: ", round(time.perf_counter()-start, 5) )


def serial_writer():
    
    while True:
        
        frame = frame_queue.get()  # blokuje, aż pojawi się ramka
        start = time.perf_counter()
        print("jest ramka", time.perf_counter() - start)
        SER.write(frame)
        
        frame_queue.task_done()    # oznaczamy, że zadanie zostało wykonane
        print("ramka wysłana", time.perf_counter() - start)

def checker():
    last_state = 0
    while True:
        if (last_state != state):
            print(last_state, state, "!!!!!!!!!!")

        last_state = state

def get_screen_colors(y_idx,x_idx, monitor, sct):
    sct_img = sct.grab(monitor)
    frame = np.array(sct_img)[:, :, :3][:, :, ::-1]  # RGB
    led_colors = frame[y_idx, x_idx] 
    return led_colors;

def smooth_colors(colors, prev, alpha=100):
    return ((prev.astype(np.uint16) * alpha +
             colors.astype(np.uint16) * (255 - alpha)) >> 8).astype(np.uint8)

def white_balance(colors):
    r = colors[:, 0].astype(np.float32) * 1.00
    g = colors[:, 1].astype(np.float32) * 0.8
    b = colors[:, 2].astype(np.float32) * 0.65

    c = np.stack([r, g, b], axis=1)
    return np.clip(c, 0, 255).astype(np.uint8)

def avg_colors(colors):
    return colors.reshape(NUM_LEDS,5,3).mean(axis=1).astype(np.uint8)

def debug_rectangles():
    with mss() as sct:
        monitor = sct.monitors[1]
        img = sct.grab(monitor)

        arr = np.array(img)[:, :, :3]
        arr = np.array(img)[:, :, :3].copy()
        height, width, _ = arr.shape

    # rysuj NA arr
    cv2.rectangle(
        arr,
        (0, 0),
        (100, 100),
        (0, 0, 255),   # czerwony w BGR
        2
    )

    cv2.imshow("DEBUG", arr)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def get_monitor_colors(num_leds=NUM_LEDS, proportion=PROPORTION):
    bottom, right, top, left = [], [], [], []

    # ile LEDów na bok
    n_bottom = proportion[0]
    n_right  = proportion[1]
    n_top    = proportion[2]
    n_left   = proportion[3]

    with mss() as sct:
        monitor = sct.monitors[1]
        img = sct.grab(monitor)
        arr = np.array(img)[:, :, :3]
        height, width, _ = arr.shape

        # ===== BOTTOM =====
        y = height - 1
        for i in range(n_bottom):
            x = int((i + 0.5) * width / n_bottom)
            r, g, b = arr[y, x]
            bottom.append((int(b), int(g), int(r)))  # BGR → RGB jeśli trzeba

        # ===== RIGHT =====
        x = width - 1
        for i in range(n_right):
            y = int(height - (i + 0.5) * height / n_right)
            r, g, b = arr[y, x]
            right.append((int(b), int(g), int(r)))

        # ===== TOP =====
        y = 0
        for i in range(n_top):
            x = int(width - (i + 0.5) * width / n_top)
            r, g, b = arr[y, x]
            top.append((int(b), int(g), int(r)))

        # ===== LEFT =====
        x = 0
        for i in range(n_left):
            y = int((i + 0.5) * height / n_left)
            r, g, b = arr[y, x]
            left.append((int(b), int(g), int(r)))

    # kolejność LEDów wokół ekranu
    return bottom + right + top + left

def postprocess_colors(colors, max_brightness=100, saturation_boost=5):
    c = colors.astype(np.float32)
    c = apply_saturation(c, saturation_boost, max_brightness)
    c = white_balance(c)
    c = np.clip(c, 0, 255).astype(np.uint8)
    c = apply_gamma(c)
    
    return c.astype(np.uint8)

def apply_saturation(colors, saturation_boost, max_brightness):
    c = colors.astype(np.float32)

    gray = c.mean(axis=1, keepdims=True)
    c = gray + (c - gray) * saturation_boost

    c = np.clip(c, 0, max_brightness)
    return c

def apply_gamma(colors):

    # wymuś uint8 PRZED LUT
    colors_u8 = np.clip(colors, 0, 255).astype(np.uint8)

    return GAMMA_LUT[colors_u8]

def get_samples(num_leds=NUM_LEDS, proportion=PROPORTION, width = 2560, height = 1440, geometry_lendth = 100):
    led_samples = []
    segment_width = width / 33
    segment_height = (height-2*SHIFT) / 17
    for i in range(proportion[0]):
        
        x1 = i * segment_width
        x2 = (i+1) * segment_width
        y1 = height - geometry_lendth - SHIFT
        y2 = height - SHIFT

        # próbki w prostokącie (procentowo)
        points = [
            (x1 + 0.25*(x2-x1), y1 + 0.25*(y2-y1)),
            (x1 + 0.75*(x2-x1), y1 + 0.25*(y2-y1)),
            (x1 + 0.5*(x2-x1),  y1 + 0.5*(y2-y1)),
            (x1 + 0.25*(x2-x1), y1 + 0.75*(y2-y1)),
            (x1 + 0.75*(x2-x1), y1 + 0.75*(y2-y1)),
        ]
        led_samples.append([ (int(x), int(y)) for x,y in points ])

    for i in range(proportion[1]):
        
        x1 = width - geometry_lendth
        x2 = width 
        y1 = height -SHIFT - i * segment_height
        y2 = height - SHIFT - (i+1) * segment_height

        # próbki w prostokącie (procentowo)
        points = [
            (x1 + 0.25*(x2-x1), y1 + 0.25*(y2-y1)),
            (x1 + 0.75*(x2-x1), y1 + 0.25*(y2-y1)),
            (x1 + 0.5*(x2-x1),  y1 + 0.5*(y2-y1)),
            (x1 + 0.25*(x2-x1), y1 + 0.75*(y2-y1)),
            (x1 + 0.75*(x2-x1), y1 + 0.75*(y2-y1)),
        ]
        led_samples.append([ (int(x), int(y)) for x,y in points ])

    for i in range(proportion[2]):
        x1 = width - i * segment_width
        x2 = width - (i+1) * segment_width
        y1 = SHIFT
        y2 = geometry_lendth + SHIFT

        # próbki w prostokącie (procentowo)
        points = [
            (x1 + 0.25*(x2-x1), y1 + 0.25*(y2-y1)),
            (x1 + 0.75*(x2-x1), y1 + 0.25*(y2-y1)),
            (x1 + 0.5*(x2-x1),  y1 + 0.5*(y2-y1)),
            (x1 + 0.25*(x2-x1), y1 + 0.75*(y2-y1)),
            (x1 + 0.75*(x2-x1), y1 + 0.75*(y2-y1)),
        ]
        led_samples.append([ (int(x), int(y)) for x,y in points ])

    for i in range(proportion[3]):
        x1 = 0
        x2 = geometry_lendth
        y1 = i * segment_height + SHIFT
        y2 = (i+1) * segment_height + SHIFT

        # próbki w prostokącie (procentowo)
        points = [
            (x1 + 0.25*(x2-x1), y1 + 0.25*(y2-y1)),
            (x1 + 0.75*(x2-x1), y1 + 0.25*(y2-y1)),
            (x1 + 0.5*(x2-x1),  y1 + 0.5*(y2-y1)),
            (x1 + 0.25*(x2-x1), y1 + 0.75*(y2-y1)),
            (x1 + 0.75*(x2-x1), y1 + 0.75*(y2-y1)),
        ]
        led_samples.append([ (int(x), int(y)) for x,y in points ])

    return led_samples

def get_lower_pixels_colors(num_leds=NUM_LEDS):
    p = []
    height = 1080
    width = 1920
    with mss() as sct:
        monitor = sct.monitors[1]
        img = sct.grab(monitor)
        arr = np.array(img)
        arr = arr[:, :, :3]
        height,width, _= arr.shape
        for i in range(num_leds):
                x = int(i * width / num_leds + width / (2*num_leds))
                y = height - 1  # dolna linia
                p.append((
                        int(arr[y, x, 2]),
                        int(arr[y, x, 1]),
                        int(arr[y, x, 0])
                    ))
    return p

def get_colors_from_pixels(pixels):
    colors = []
    for i in pixels:
        colors.append(get_screen_pixel(i))
    return colors

def color_to_rgb(color, num_leds=NUM_LEDS):
    try:
        return webcolors.name_to_rgb(color)  # zwraca tuple RGB
    except ValueError:
        raise ValueError("Color not found")
    
def gen_random_colors_list(num_leds=NUM_LEDS):
    return [(randrange(0,256),randrange(0,256),randrange(0,256)) for i in range(num_leds)]

def gen_color_list(color=(0,0,0),num_leds=NUM_LEDS):
    return [color for i in range(num_leds)]

def decode_frame(frame):
    frame_length = len(frame)
    if frame_length < 2:
        raise ValueError("Frame too short")
    elif frame_length == 2:
        raise ValueError("No data")
    
    if frame[0] != START:
        raise ValueError("Invalid START byte")
    
    payload_length = frame[1]

    if payload_length + 4 != len(frame):
        raise ValueError("Incomplete frame")
    
    payload_bytes = frame[3:2+payload_length]
        
    if payload_length % 3 != 0:
        raise ValueError("Payload length is not divisible by 3")
    
    values = [tuple(payload_bytes[i:i+3]) for i in range(0, payload_length, 3)]
    print(values)

def gen_payload(data_list, payload_format='<3B'):
    payload = bytearray()
    for row in data_list:
        payload.extend(struct.pack(payload_format, *row)) 
    return payload

def gen_frame(payload, start=START):
    crc = crc8(payload)
    return struct.pack('<BH', start, len(payload)) + payload  + bytes([crc])

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

def send_frame(frame):
    SER.write(frame)


if __name__ == '__main__':
    main()