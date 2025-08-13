import can
import os
import keyboard
import time
import numpy as np
from datetime import datetime, timedelta

##### HYPER PARAMETERS #####
SAVE_PATH           = "6072/"
BITRATE             = 1000000
SAMPLING_RATE       = 0.1       # seconds (10 Hz)
NUM_FLOATING_POINT  = 2         # Number of floating points
CALIBRATION_TIME    = 3         # seconds
MOVING_AVERAGE_TERM = 5         # Moving Average Window Size
RECORD_BUFFER       = 1         # Record automatically after RECORD_BUFFER seconds for RECORD_TIME seconds
RECORD_TIME         = 5         # Record automatically after RECORD_BUFFER seconds for RECORD_TIME seconds

class Sensor:
    def __init__(self, sensor_id, offset=None):
        self.sensor_id = sensor_id
        self.offset = offset if offset else [0, 0, 0, 0, 0, 0]
        self.force = []
        self.torque = []

    def update(self, ft, data):
        if ft == 0:
            x = ((data[0] << 8) | data[1]) / 1000 - 30 - self.offset[0]
            y = ((data[2] << 8) | data[3]) / 1000 - 30 - self.offset[1]
            z = ((data[4] << 8) | data[5]) / 1000 - 30 - self.offset[2]
            self.force.append([str(datetime.now()), x, y, z])

        else:
            x = ((data[0] << 8) | data[1]) / 100000 - 0.3 - self.offset[3]
            y = ((data[2] << 8) | data[3]) / 100000 - 0.3 - self.offset[4]
            z = ((data[4] << 8) | data[5]) / 100000 - 0.3 - self.offset[5]
            self.torque.append([str(datetime.now()), x*1000, y*1000, z*1000])

    def reset_ft(self):
        self.force = []
        self.torque = []
    
    def reset_offset(self):
        self.offset = [0, 0, 0, 0, 0, 0]

    def is_connected(self):
        if len(self.force) == 0 and len(self.torque) == 0:
            return False
        else:
            return True

    def set_offset(self, force_offset, torque_offset):
        self.offset = force_offset + torque_offset

    def moving_average(self, data, window_size=5):
        if not data or window_size < 2:
            return data
        ma = []
        xs, ys, zs = [], [], []
        for i, row in enumerate(data):
            xs.append(row[1])
            ys.append(row[2])
            zs.append(row[3])
            if len(xs) > window_size:
                xs.pop(0)
                ys.pop(0)
                zs.pop(0)
            if len(xs) == window_size:
                avg_x = sum(xs) / window_size
                avg_y = sum(ys) / window_size
                avg_z = sum(zs) / window_size
                ma.append([row[0], avg_x, avg_y, avg_z])
        return ma

    def save(self, base_filename, directory):
        os.makedirs(directory, exist_ok=True)
        with open(os.path.join(directory, f"{base_filename}_force.txt"), 'w', encoding='utf-8') as f:
            for row in self.force:
                f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in row) + '\n')
        with open(os.path.join(directory, f"{base_filename}_torque.txt"), 'w', encoding='utf-8') as f:
            for row in self.torque:
                f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in row) + '\n')

    def save_ma(self, base_filename, directory, ma_window=5):
        os.makedirs(directory, exist_ok=True)
        ma_force = self.moving_average(self.force, ma_window)
        ma_torque = self.moving_average(self.torque, ma_window)
        with open(os.path.join(directory, f"{base_filename}_force_ma.txt"), 'w', encoding='utf-8') as f:
            for row in ma_force:
                f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in row) + '\n')
        with open(os.path.join(directory, f"{base_filename}_torque_ma.txt"), 'w', encoding='utf-8') as f:
            for row in ma_torque:
                f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in row) + '\n')

    def save_ma_sample(self, base_filename, directory, ma_window=5, sample_rate=0.1):
        def downsample(data):
            if not data:
                return []

            ma_data = self.moving_average(data, window_size=ma_window)
            result = []
            group = []
            t0 = datetime.fromisoformat(ma_data[0][0])

            for row in ma_data:
                t = datetime.fromisoformat(row[0])
                if (t - t0).total_seconds() < sample_rate:
                    group.append(row)
                else:
                    if group:
                        xs = [r[1] for r in group]
                        ys = [r[2] for r in group]
                        zs = [r[3] for r in group]
                        avg = [sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)]
                        result.append([group[-1][0]] + avg)
                    group = [row]
                    t0 = t

            if group:
                xs = [r[1] for r in group]
                ys = [r[2] for r in group]
                zs = [r[3] for r in group]
                avg = [sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)]
                result.append([group[-1][0]] + avg)

            return result

        os.makedirs(directory, exist_ok=True)

        avg_force = downsample(self.force)
        with open(os.path.join(directory, f"{base_filename}_force_ma_sample.txt"), 'w', encoding='utf-8') as f:
            for row in avg_force:
                f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in row) + '\n')

        avg_torque = downsample(self.torque)
        with open(os.path.join(directory, f"{base_filename}_torque_ma_sample.txt"), 'w', encoding='utf-8') as f:
            for row in avg_torque:
                f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in row) + '\n')

    def save_ma_total(self, base_filename, directory, ma_window=5):
        def downsample(data):
            if not data:
                return []

            ma_data = self.moving_average(data, window_size=ma_window)
            total = []

            xs = [r[1] for r in ma_data]
            ys = [r[2] for r in ma_data]
            zs = [r[3] for r in ma_data]

            total = [sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)]
            return total

        os.makedirs(directory, exist_ok=True)

        start = datetime.fromisoformat(self.force[0][0])
        end = datetime.fromisoformat(self.force[-1][0])
        elapsed = end - start

        total_force = downsample(self.force)
        total_torque = downsample(self.torque)
        with open(os.path.join(directory, f"{base_filename}_ma_total.csv"), 'a', encoding='utf-8') as f:
            f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in total_force) + ',')
            f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in total_torque) + '\n')
        return elapsed

    def save_ma_total_auto(self, base_filename, directory, ma_window, buffer, elapsed):
        def downsample(data):
            if not data:
                return []

            ma_data = self.moving_average(data, window_size=ma_window)
            result = []
            group = []
            t0 = datetime.fromisoformat(ma_data[0][0])

            for row in ma_data:
                t = datetime.fromisoformat(row[0])
                if buffer < (t - t0).total_seconds() and (t - t0).total_seconds() < buffer + elapsed:
                    group.append(row)

            if group:
                xs = [r[1] for r in group]
                ys = [r[2] for r in group]
                zs = [r[3] for r in group]
                total = [sum(xs)/len(xs), sum(ys)/len(ys), sum(zs)/len(zs)]

            return total

        os.makedirs(directory, exist_ok=True)
        start = datetime.fromisoformat(self.force[0][0])
        end = datetime.fromisoformat(self.force[-1][0])
        actual_elapsed = end - start

        if actual_elapsed < timedelta(seconds=buffer):
            print("Please ensure that the recording duration is at least RECORD_BUFFER seconds.")
            return None
        if actual_elapsed < timedelta(seconds=buffer + elapsed):
            print("Please ensure that the recording duration is at least RECORD_BUFFER + RECORD_TIME seconds.")
            elapsed = actual_elapsed.total_seconds() - buffer

        total_force = downsample(self.force)
        total_torque = downsample(self.torque)
        with open(os.path.join(directory, f"sensor{self.sensor_id + 1}_ma_total_auto.csv"), 'a', encoding='utf-8') as f:
            f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in total_force) + ',')
            f.write(','.join(f"{x:.{NUM_FLOATING_POINT}f}" if isinstance(x, float) else x for x in total_torque) + '\n')
       
        return elapsed

ID_TO_IDX = {
    0x1A: (0, 0),  0x1B: (0, 1),
    0x2A: (1, 0),  0x2B: (1, 1),
    0x3A: (2, 0),  0x3B: (2, 1),
    0x4A: (3, 0),  0x4B: (3, 1)
}

def receive_message_periodically(bus, interval_seconds):
    return bus.recv()

def flush_can_buffer(bus, timeout=0.005):
    """CAN 버퍼를 모두 비움. timeout=0.0이면 non-blocking."""
    while True:
        msg = bus.recv(timeout)
        if msg is None:
            break

def can_calibration(bus, sensors, directory=None):
    print(f"Calibration will begin in {CALIBRATION_TIME} seconds ...")
    tmp_force = [[] for _ in range(4)]
    tmp_torque = [[] for _ in range(4)]
    
    start_time = time.time()
    end_time = start_time + CALIBRATION_TIME

    while time.time() < end_time:
        msg = receive_message_periodically(bus, SAMPLING_RATE)
        can_id, can_data = msg.arbitration_id, msg.data

        if can_id not in ID_TO_IDX:
            continue

        idx, ft = ID_TO_IDX[can_id]

        if ft == 0:
            x = ((can_data[0] << 8) | can_data[1]) / 1000 - 30
            y = ((can_data[2] << 8) | can_data[3]) / 1000 - 30
            z = ((can_data[4] << 8) | can_data[5]) / 1000 - 30
            tmp_force[idx].append([x, y, z])
        else:
            x = ((can_data[0] << 8) | can_data[1]) / 100000 - 0.3
            y = ((can_data[2] << 8) | can_data[3]) / 100000 - 0.3
            z = ((can_data[4] << 8) | can_data[5]) / 100000 - 0.3
            tmp_torque[idx].append([x, y, z])

    for i in range(4):
        if len(tmp_force[i]) > 0 and len(tmp_torque[i]) > 0:
            force_arr = np.array(tmp_force[i])
            force_offset = np.mean(force_arr, axis=0).tolist()
            torque_arr = np.array(tmp_torque[i])
            torque_offset = np.mean(torque_arr, axis=0).tolist()
            sensors[i].set_offset(force_offset, torque_offset)
            print(f"[Sensor {i+1}] is connected. Calibration: Force: {force_offset}, Torque: {torque_offset}")
        else:
            print(f"[Sensor {i+1}] is unconnected. ")

    print("Calibration completed.")

    if directory is not None:
        os.makedirs(directory, exist_ok=True)

        offset_log = []
        for i, sensor in enumerate(sensors, 1):
            if sensor.offset != [0, 0, 0, 0, 0, 0]:
                force_offset = sensor.offset[:3]
                torque_offset = sensor.offset[3:]
                offset_log.append(f"[Sensor {i}] Calibration offset set. "
                                f"Force: {force_offset}, Torque: {torque_offset}")
            else:
                offset_log.append(f"[Sensor {i}] is unconnected. ")
            offset_file = os.path.join(directory, "calibration_offset.txt")
        with open(offset_file, 'a', encoding='utf-8') as f:
            for line in offset_log:
                f.write(line + '\n')
            f.write('\n')

        print(f"Calibration offset saved to: {offset_file}")
    print("----------------------------------------------------------------")

def can_record(bus, sensors) :

    try:
        while True :
            if keyboard.is_pressed("e") :
                break

            msg = receive_message_periodically(bus, SAMPLING_RATE)
            can_id, can_data = msg.arbitration_id, msg.data

            if can_id not in ID_TO_IDX:
                continue
            
            idx, ft = ID_TO_IDX[can_id]
            sensors[idx].update(ft, can_data)

        return sensors
            
    except KeyboardInterrupt:
        print("something(intrrupt) has come up. Exiting the program.... bye!")
        bus.shutdown()

def can_record_auto(bus, sensors, buffer, record_time):
    print(f"Recording will start after {buffer} seconds...")
    time.sleep(buffer)
    print("Recording started.")

    start_time = time.time()
    elapsed = 0

    try:
        cnt = 0
        while True:
            if keyboard.is_pressed("e"):
                print("Recording manually stopped by user.")
                break

            msg = receive_message_periodically(bus, SAMPLING_RATE)
            can_id, can_data = msg.arbitration_id, msg.data

            if can_id not in ID_TO_IDX:
                continue

            idx, ft = ID_TO_IDX[can_id]
            sensors[idx].update(ft, can_data)

            elapsed = time.time() - start_time

            ## record_time < elapsed 일 때 바로 종료하면 4.99n초에 종료가 되어서, 5.00n초를 보장하기 위해서 각 센서에 대해 한번씩만 더 데이터를 받음(n > 2)
            if elapsed >= record_time:
                cnt += 1
                if cnt > 2:
                    break

        print(f"Recording will end after {buffer} seconds...")
        time.sleep(buffer)
        print("Recording ended automatically." if elapsed >= record_time else "Recording ended by user.")
        return sensors

    except KeyboardInterrupt:
        print("Program interrupted.")
        bus.shutdown()
        return sensors


def save_all_sensors(sensors, directory):
    for i, sensor in enumerate(sensors, 1):
        if sensor.is_connected():
            sensor.save(f"sensor{i}", directory)
            print(f"[Saved] Sensor {i} data (force/torque)")

def save_all_sensors_ma(sensors, directory, ma_window=5):
    for i, sensor in enumerate(sensors, 1):
        if sensor.is_connected():
            sensor.save_ma(f"sensor{i}", directory, ma_window)
            print(f"[Saved] Sensor {i} data (force/torque) with Moving Average of {ma_window}")

def save_all_sensors_ma_sample(sensors, directory, ma_window=5, sample_rate=0.1):
    for i, sensor in enumerate(sensors, 1):
        if sensor.is_connected():
            sensor.save_ma_sample(f"sensor{i}", directory, ma_window, sample_rate)
            print(f"[Saved] Sensor {i} data (force/torque) with Moving Average of {ma_window} and sample rate of {sample_rate}")

def save_all_sensors_ma_total(sensors, directory, ma_window=5):
    for i, sensor in enumerate(sensors, 1):
        if sensor.is_connected():
            elapsed = sensor.save_ma_total(f"sensor{i}", directory, ma_window)
            print(f"[Saved] Sensor {i} data (force/torque) with Moving Average of {ma_window} in {elapsed} seconds")

def save_all_sensors_ma_total_auto(sensors, directory, ma_window=5, buffer=1, elapsed=5):
    for i, sensor in enumerate(sensors, 1):
        if sensor.is_connected():
            elapsed = sensor.save_ma_total(f"sensor{i}", directory, ma_window)
            if elapsed is None:
                continue
            print(f"[Saved] Sensor {i} data (force/torque) with Moving Average of {ma_window} from {timedelta(seconds=buffer)} to {timedelta(seconds=buffer) + elapsed} seconds (Total {elapsed} seconds))")

def reset_all_sensors(sensors):
    for sensor in sensors:
        sensor.reset_ft()
        sensor.reset_offset()

def main_page() :
    os.system("cls")
    print("----------------------------------------------------------------")
    print("Miniature 6-axis Force & Torque Sensor, AFT20-D15 data collecter(Serial V3)")
    print("----------------------------------------------------------------")
    print("1. Record Start : press R")
    print("2. Save Results : press S")
    print("3. Exit the program : press F")
    print("----------------------------------------------------------------")

def main():
    main_page()
    sensors = [Sensor(i) for i in range(4)]
    calibration_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    calibration_dir = os.path.join(SAVE_PATH, calibration_time)

    while True:
        if keyboard.is_pressed("r") and calibration_dir:
            bus = can.interface.Bus(interface='ixxat', channel=0, bitrate=BITRATE)

            record_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            record_dir = os.path.join(calibration_dir, record_time)
            os.makedirs(record_dir, exist_ok=True)

            reset_all_sensors(sensors)
            can_calibration(bus, sensors, calibration_dir)

            bus.shutdown()
            print("Press SPACE to start recording...")
            while not keyboard.is_pressed("space"):
                time.sleep(0.01)

            bus = can.interface.Bus(interface='ixxat', channel=0, bitrate=BITRATE)
            sensors = can_record_auto(bus, sensors, RECORD_BUFFER, RECORD_TIME)
            save_all_sensors(sensors, record_dir)
            save_all_sensors_ma_total_auto(sensors, calibration_dir, MOVING_AVERAGE_TERM, RECORD_BUFFER, RECORD_TIME)
            print("Sensor data saved.")
            print("----------------------------------------------------------------")
            time.sleep(0.1)
            bus.shutdown()
    
        elif keyboard.is_pressed("f"):
            print("Exiting program.")
            break

        time.sleep(0.01)

if __name__ == "__main__" :
    main()