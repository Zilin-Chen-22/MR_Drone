import struct
import ctypes
import os
import matplotlib.pyplot as plt

# to do add a python function to read a matrix of binaries
def ReadBinaryLog(contents, offset_btye, binary_format, formatSize, num_of_cols, num_of_rows):
    data = list()
    for i in range(num_of_cols):
        data.append(list())

    stateSize =  num_of_cols * formatSize
    format_sq = num_of_cols * binary_format
    for i in range(num_of_rows):
        start_idx = offset_btye + i * stateSize
        end_idx = offset_btye + (i + 1) * stateSize
        var = struct.unpack(format_sq, contents[start_idx: end_idx])
        for j in range(num_of_cols):
            data[j].append(var[j])

    return data

# add a python function to extract size information and save to dict
def ReadBinaryInfoInt(contents, offset_bytes, binary_format,
                   binary_format_size, num_of_data, name=None):
    data = ReadBinaryLog(contents, offset_bytes, binary_format, binary_format_size, num_of_data, 1)
    res = dict()
    for i in range(num_of_data):
        if name != None:
            res[name[i]] = int(data[i][0])
        else:
            res["unnamed_entry_" + str(i)] = int(data[i][0])
    return res

if __name__ == '__main__':
    filename = "/home/longhao/source/racing_drone_simulator/build_release/adv_pid_test.dat"
    with open(filename, mode='rb') as f:
        contents = f.read()
    f.close()
    double_len = ctypes.sizeof(ctypes.c_double)
    print(len(contents))
    info = ReadBinaryInfoInt(contents, offset_bytes=0, binary_format='d',
                             binary_format_size=double_len, num_of_data=2,
                             name=["time_step", "num_of_data"])

    print(info)
    data = ReadBinaryLog(contents, 2 * double_len, 'd', 
                         double_len,
                         num_of_cols=info['num_of_data'],
                         num_of_rows=info['time_step'])

    # plot control result

    plt.figure(figsize=(10,10), dpi= 100)
    plt.subplot(311)
    plt.plot(data[0], data[1])
    plt.ylabel('p(rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data[0], data[2])
    plt.ylabel('q(rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data[0], data[3])
    plt.ylabel('r(rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)

    plt.figure(figsize=(10,10), dpi= 100)
    plt.subplot(311)
    plt.plot(data[0], data[4])
    plt.ylabel('torque x (Nm)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data[0], data[5])
    plt.ylabel('torque y (Nm)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data[0], data[6])
    plt.ylabel('torque z (Nm)')
    plt.xlabel('t(s)')
    plt.grid(True)

    plt.figure(figsize=(10,10), dpi= 100)
    plt.subplot(311)
    plt.plot(data[0], data[7])
    plt.ylabel('err x (rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data[0], data[8])
    plt.ylabel('err y (rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data[0], data[9])
    plt.ylabel('err z (rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)

    plt.show(block=False)
    input("Press Enter to continue...")