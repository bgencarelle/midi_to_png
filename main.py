import mido
import cv2
import numpy as np
import os
from threading import Event

highest_timecode_seen = 0
clock_counter = 0
stop_event = Event()
mtc_values = [0, 0, 0, 0]

mtc_img = np.zeros((150, 600), dtype=np.uint8)
last_mtc_timecode = '00:00:00:00:00'


def parse_array_file(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()

    png_paths = []
    for line in lines:
        if line.startswith("Iteration"):
            paths = line.strip().split("=(")[1].strip(")").split(",")
            png_paths.append(paths)

    return png_paths


def parse_mtc(msg):
    mtc_type = msg.frame_type
    value = msg.frame_value
    return mtc_type, value


def update_mtc_timecode(mtc_type, value, clock_counter):
    global mtc_values

    if mtc_type == 0:
        mtc_values[3] = (mtc_values[3] & 0xF0) | value
    elif mtc_type == 1:
        mtc_values[3] = (mtc_values[3] & 0x0F) | (value << 4)
    elif mtc_type == 2:
        mtc_values[2] = (mtc_values[2] & 0xF0) | value
    elif mtc_type == 3:
        mtc_values[2] = (mtc_values[2] & 0x0F) | (value << 4)
    elif mtc_type == 4:
        mtc_values[1] = (mtc_values[1] & 0xF0) | value
    elif mtc_type == 5:
        mtc_values[1] = (mtc_values[1] & 0x0F) | (value << 4)
    elif mtc_type == 6:
        mtc_values[0] = (mtc_values[0] & 0xF0) | value
    elif mtc_type == 7:
        mtc_values[0] = (mtc_values[0] & 0x0F) | (value << 4)

    hours = mtc_values[0] & 0x1F
    minutes = mtc_values[1]
    seconds = mtc_values[2]
    frames = mtc_values[3]
    subframes = int((clock_counter / 24) * 100)  # Assuming 24 PPQN (Pulses Per Quarter Note)

    return f'{hours:02}:{minutes:02}:{seconds:02}:{frames:02}:{subframes:02}'


def calculate_total_frames(hours, minutes, seconds, frames):
    total_frames = (hours * 60 * 60 * 30) + (minutes * 60 * 30) + (seconds * 30) + frames
    return total_frames


def process_midi_messages(input_port):
    global clock_counter, mtc_values, frame_counter, last_mtc_timecode
    current_total_frames = 0

    for msg in input_port.iter_pending():
        if msg.type == 'quarter_frame':
            mtc_type, value = parse_mtc(msg)
            last_mtc_timecode = update_mtc_timecode(mtc_type, value, clock_counter)
        elif msg.type == 'clock':
            clock_counter += 1
        elif msg.type == 'sysex' and msg.data[:5] == [0x7F, 0x7F, 0x01, 0x01, 0x00]:
            clock_counter = 0
            mtc_values = [msg.data[5], msg.data[6], msg.data[7], msg.data[8]]
            for mtc_type, value in enumerate(mtc_values):
                last_mtc_timecode = update_mtc_timecode(mtc_type * 2, value & 0x0F, clock_counter)
                last_mtc_timecode = update_mtc_timecode(mtc_type * 2 + 1, value >> 4, clock_counter)

    if last_mtc_timecode is not None:
        hours = mtc_values[0] & 0x1F
        minutes = mtc_values[1]
        seconds = mtc_values[2]
        frames = mtc_values[3]
        current_total_frames = calculate_total_frames(hours, minutes, seconds, frames)

    return last_mtc_timecode, current_total_frames


def display_timecode(mtc_timecode, estimate_frame_counter, png_paths):
    index_mult = 2.0
    if mtc_timecode is not None:
        index = int(estimate_frame_counter / (8.6326/index_mult)) % (int(index_mult) * len(png_paths))
        if index >= len(png_paths):
            index = (int(index_mult) * len(png_paths) - 1) - index
            print(index)

        if 0 <= index < len(png_paths):
            png_file = png_paths[index][3]
            frame = cv2.imread(png_file)
            if frame is not None:
                cv2.putText(frame, mtc_timecode, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2)
                cv2.putText(frame, f'Estimate Frame Counter: {estimate_frame_counter}', (10, 100),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, f'index: {index}', (10, 140),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow(f'MTC Timecode - PID: {os.getpid()}', frame)


def main():
    global clock_counter
    input_port = mido.open_input('IAC Driver Bus 1')
    png_paths = parse_array_file('arrray_test.txt')

    while not stop_event.is_set():
        mtc_timecode, estimate_frame_counter = process_midi_messages(input_port)
        display_timecode(mtc_timecode, estimate_frame_counter, png_paths)

        # Break the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_event.set()

    # Clean up
    input_port.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Script interrupted by user")
        stop_event.set()
