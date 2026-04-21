#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Récepteur UART + Base de données SQLite avec synchronisation
Raspberry Pi 5 - Smart Beehive
by Kemajou Anicet

Format trame STM32 : [0xAA][26 octets de données]
'''

import sqlite3
import serial
import time
from datetime import datetime
import os

# === Configuration ===
DB_FILE = 'hive_data.db'
SERIAL_PORT = '/dev/serial0'
BAUDRATE = 9600
FRAME_TOTAL_SIZE = 27  # 1 octet header (0xAA) + 26 octets data
SYNC_BYTE = 0xAA

# === Creation of data base ===
def init_database():
    if not os.path.exists(DB_FILE):
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute('''CREATE TABLE measurements (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT NOT NULL,
            hive_id INTEGER NOT NULL,
            temp_int REAL NOT NULL,
            temp_ext REAL NOT NULL,
            humidity INTEGER NOT NULL,
            sound REAL NOT NULL,
            water_level INTEGER NOT NULL,
            hive_weight REAL NOT NULL,
            accel_x REAL NOT NULL,
            accel_y REAL NOT NULL,
            accel_z REAL NOT NULL,
            latitude REAL NOT NULL,
            longitude REAL NOT NULL,
            gps_valid INTEGER NOT NULL
        )''')
        conn.commit()
        conn.close()
        print("data base was created")

# === Decodage de la trame (26 octets utiles) ===
def decode_frame(data):
    if len(data) != 26:
        return None

    try:
        hive_id = data[0]
        temp_int = ((data[1] << 8) | data[2]) / 100.0
        temp_ext = ((data[3] << 8) | data[4]) / 100.0
        humidity = data[5]
        sound = ((data[6] << 8) | data[7]) / 1000.0
        water_level = data[8]
        accel_x = ((data[9] << 8) | data[10]) / 1000.0
        accel_y = ((data[11] << 8) | data[12]) / 1000.0
        accel_z = ((data[13] << 8) | data[14]) / 1000.0
        latitude = ((data[15] << 24) | (data[16] << 16) | (data[17] << 8) | data[18]) / 1000000.0
        longitude = ((data[19] << 24) | (data[20] << 16) | (data[21] << 8) | data[22]) / 1000000.0
        gps_valid = data[23]
        hive_weight = ((data[24] << 8) | data[25]) / 100.0

        #  Verification de coherence
        if not (1 <= hive_id <= 15):
            return None
        if not (-10.0 <= temp_int <= 60.0) or not (-10.0 <= temp_ext <= 60.0):
            return None
        if not (0 <= humidity <= 100):
            return None
        if not (0 <= water_level <= 255):
            return None
        if abs(accel_x) > 5.0 or abs(accel_y) > 5.0 or not (0.5 <= accel_z <= 1.5):
            return None
        if not (-90.0 <= latitude <= 90.0) or not (-180.0 <= longitude <= 180.0):
            return None
        if hive_weight < 0.0 or hive_weight > 100.0:
            return None

        return {
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'hive_id': hive_id,
            'temp_int': temp_int,
            'temp_ext': temp_ext,
            'humidity': humidity,
            'sound': sound,
            'water_level': water_level,
            'hive_weight': hive_weight,
            'accel_x': accel_x,
            'accel_y': accel_y,
            'accel_z': accel_z,
            'latitude': latitude,
            'longitude': longitude,
            'gps_valid': gps_valid
        }
    except Exception as e:
        print(f" Erreur décodage: {e}")
        return None

# === Sauvegarde dans la base ===
def save_to_db(data_dict):
    try:
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute('''INSERT INTO measurements
            (timestamp, hive_id, temp_int, temp_ext, humidity, sound,
             water_level, hive_weight, accel_x, accel_y, accel_z,
             latitude, longitude, gps_valid)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)''',
            (data_dict['timestamp'], data_dict['hive_id'],
             data_dict['temp_int'], data_dict['temp_ext'],
             data_dict['humidity'], data_dict['sound'],
             data_dict['water_level'], data_dict['hive_weight'],
             data_dict['accel_x'], data_dict['accel_y'],
             data_dict['accel_z'],
             data_dict['latitude'], data_dict['longitude'],
             data_dict['gps_valid']))
        conn.commit()
        conn.close()

        print(f"Hive {data_dict['hive_id']}:\n"

              f"  Poids       : {data_dict['hive_weight']:.2f} kg\n"

              f"  Water_level  : {data_dict['water_level']} cm\n"

              f"  Temp_Indoor    : {data_dict['temp_int']:.1f} °C\n"

              f"  Temp_outtdoor    : {data_dict['temp_ext']:.1f} °C\n"

              f"  Humidity    : {data_dict['humidity']} %\n"

              f"  Sound        : {data_dict['sound']:.3f}\n"

              f"  Accelerometer:\n"
              f"  X = {data_dict['accel_x']:.3f} g\n"
              f"  Y = {data_dict['accel_y']:.3f} g\n"
              f"  Z = {data_dict['accel_z']:.3f} g\n"

              f"   GPS         : ({data_dict['latitude']:.6f}, {data_dict['longitude']:.6f}) "
              f"{'1'if data_dict['gps_valid'] else '0'}")
        print("-" * 50)

    except Exception as e:
        print(f" Erreur sauvegarde: {e}")

# === Boucle principale avec synchronisation ===
def main():
    init_database()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        print(f"Connecté à {SERIAL_PORT} @ {BAUDRATE} bauds")
        print("En attente des trames (27 octets avec 0xAA)...\n")

        buffer = b""
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer += data

                # Rechercher le marqueur de synchronisation
                while len(buffer) >= FRAME_TOTAL_SIZE:
                    # Chercher le premier 0xAA
                    sync_index = buffer.find(bytes([SYNC_BYTE]))
                    if sync_index == -1:
                        # Pas de marqueur → vider le buffer
                        buffer = b""
                        break

                    if sync_index > 0:
                        # Données avant le marqueur → les ignorer
                        buffer = buffer[sync_index:]

                    if len(buffer) < FRAME_TOTAL_SIZE:
                        break

                    # Vérifier que le marqueur est bien au début
                    if buffer[0] == SYNC_BYTE:
                        payload = buffer[1:27]  # 26 octets utiles
                        buffer = buffer[27:]    # Avancer de 27 octets

                        decoded = decode_frame(payload)
                        if decoded:
                            save_to_db(decoded)
                        # Si invalide, on continue (pas de reset)
                    else:
                        # Ne devrait pas arriver, mais sécurité
                        buffer = buffer[1:]

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n Arrêt demandé")
    except Exception as e:
        print(f" Erreur: {e}")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()