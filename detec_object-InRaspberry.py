import sys
import argparse
import cv2
import numpy as np
from ultralytics import YOLO
import serial
import threading
import time
import math

# --- PARÁMETROS ROBOT ---
R_WHEEL = 0.0508
L_TRACK = 0.28
OFFSET_X = 0.085
F_CALIB = 887.5
H_REAL = 0.08

global_x, global_y = 0.0, 0.0
object_detected = False
running = True

try:
    ser = serial.Serial('/dev/serial0', 9600, timeout=1)
except:
    sys.exit(0)

def calculate_v(dx, dy, t):
    if t <= 0: return 0, 0
    v = dy / t
    w_chassis = math.atan2(dx, dy) / t
    wl = (v - (L_TRACK * w_chassis) / 2) / R_WHEEL
    wr = (v + (L_TRACK * w_chassis) / 2) / R_WHEEL
    return wl, wr

def terminal_thread():
    global running, object_detected, global_x, global_y
    while running:
        if object_detected:
            print(f"\n[INFO] x:{global_x:.2f} y:{global_y:.2f} (Ref: Gripper)")
            conf = input(">> ¿Confirmar deteccion? [y/n]: ").strip().lower()
            if conf == 'y':
                print("Refrescando coordenadas...")
                time.sleep(1.0)
                cx, cy = global_x, global_y
                try:
                    t_input = float(input(">> Tiempo (s): "))
                    wl, wr = calculate_v(cx, cy, t_input)
                    
                    # Formato solicitado: {"wR":VALOR,"wL":VALOR,"}
                    trama_json = f'{{"wR":{wr:.2f},"wL":{wl:.2f},"}}'
                    print(trama_json)
                    
                    # Transmisión UART continua durante el tiempo t
                    t_start = time.time()
                    while (time.time() - t_start) < t_input:
                        ser.write(trama_json.encode())
                        time.sleep(0.1)
                    
                    # Stop
                    ser.write(f'{{"wR":0.00,"wL":0.00,"}}'.encode())
                    object_detected = False
                except: print("Error de dato.")
        time.sleep(0.5)

threading.Thread(target=terminal_thread, daemon=True).start()

parser = argparse.ArgumentParser()
parser.add_argument('--model', required=True)
parser.add_argument('--source', required=True)
args = parser.parse_args()
model = YOLO(args.model, task='detect')
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break
    cropped = frame[int(480*0.1):int(480*0.9), int(640*0.1):int(640*0.9)]
    cx_c = cropped.shape[1] / 2
    results = model(cropped, verbose=False)
    detections = results[0].boxes
    found_c = False

    for i in range(len(detections)):
        if detections[i].conf.item() > 0.5:
            xyxy = detections[i].xyxy.cpu().numpy().squeeze().astype(int)
            h_px = xyxy[3] - xyxy[1]
            if h_px > 0:
                dy = (F_CALIB * H_REAL) / h_px
                dx = (((xyxy[0] + xyxy[2]) / 2 - cx_c) * dy) / F_CALIB + OFFSET_X
                global_x, global_y = dx, dy
                found_c = True
                
                # Visualización en cuadro: x e y
                cv2.rectangle(cropped, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 0, 255), 2)
                cv2.putText(cropped, f"x:{dx:.2f} y:{dy:.2f}", (xyxy[0], xyxy[1]-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    object_detected = found_c
    cv2.imshow('Deteccion Barrera - Gripper Ref', cropped)
    if cv2.waitKey(1) == ord('q'):
        running = False
        break

cap.release()
cv2.destroyAllWindows()