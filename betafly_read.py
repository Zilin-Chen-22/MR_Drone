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


def ScanBinaryLog(filename, numOfColumns, format, formatSize):
    with open(filename, mode='rb') as f:
        contents = f.read()
    f.close()
    res = list()
    format_sq = numOfColumns * format
    stateSize = numOfColumns * formatSize
    totalSizeByte = len(contents)
    if (totalSizeByte % stateSize) != 0:
        raise Exception("size of " + filename + " is " + str(totalSizeByte) +
                        ", which is not a multiple of stateSize " + str(stateSize) + "!")

    numOfSamples = int(totalSizeByte / stateSize)

    for _ in range(numOfColumns):
        res.append(list())

    for i in range(numOfSamples):
        var = struct.unpack(format_sq, contents[(
            i * stateSize):((i + 1) * stateSize)])
        for j in range(numOfColumns):
            res[j].append(var[j])

    return res

def least_sqare(x, y):  # return a list with k, b, r
    n = len(x)
    ave_x = sum(x) / n 
    ave_y = sum(y) / n

    m, p, q = 0, 0, 0

    for i in range(n):
        m += (x[i] - ave_x) * (y[i] - ave_y)
        p += (x[i] - ave_x) ** 2
        q += (y[i] - ave_y) ** 2
    
    b = m / p
    a = ave_y - b * ave_x
    r = m / ((p ** (1 / 2)) * (q ** (1 / 2)) )

    return [b, a, r]

def mean_deviation(x1, x2):
    n = len(x1)
    ave_x1 = sum(x1) / n
    ave_x2 = sum(x2) / n

    s1, s2 = 0, 0

    for i in range(n):
        s1 += (x1[i] - ave_x1) ** 2
        s2 += (x2[i] - ave_x2) ** 2
    
    s1 = s1 / (n * n)
    s2 = s2 / (n * n)

    t = ((ave_x1 - ave_x2) ** 2) / (s1 + s2)
    
    return t

if __name__ == '__main__':
    filename = "./build_release/betaflight_test.dat"
    with open(filename, mode='rb') as f:
        contents = f.read()
    f.close()
    double_len = ctypes.sizeof(ctypes.c_double)
    info = ReadBinaryInfoInt(contents, offset_bytes=0, binary_format='d',
                             binary_format_size=double_len, num_of_data=2,
                             name=["time_step", "num_of_data"])

    print(info)
    data = ReadBinaryLog(contents, 2 * double_len, 'd', 
                         double_len,
                         num_of_cols=info['num_of_data'],
                         num_of_rows=info['time_step'])

    # read ESC-RPM line
    fp = open("data.txt", 'r')

    new_data = [[], []]

    while True:
        data_read = fp.readline()
        if data_read == '####\n':
            break

    while True:
        data_read = fp.readline()
        if data_read == '':
            break
        else:
            data_read = data_read.split(' ')
            new_data[0].append(float(data_read[0]))
            new_data[1].append(float(data_read[1]))

    max_RPM = 0
    min_RPM = 99999
    for i in range(4, 8):
        if max_RPM < max(data[i]):
            max_RPM = max(data[i]) 
        if min_RPM > min(data[i]):
            min_RPM = min(data[i])
    print(min_RPM)
    print(max_RPM)
    
    min_ESC = 0
    for i in range(len(new_data[1]) - 1):
        if new_data[1][i] <= min_RPM <= new_data[1][i + 1]:
            min_ESC = i
        if new_data[1][i] <= max_RPM <= new_data[1][i + 1]:
            max_ESC = i + 1
        
    active_ESC = new_data[0][min_ESC : max_ESC + 1]
    active_RPM = new_data[1][min_ESC : max_ESC + 1]

    result = least_sqare(active_ESC, active_RPM)
    k, r = result[0], result[2]

    t_1 = mean_deviation(data[1], data[11])

    print("k = ", k , "r = ", r)

    # plot control result

    plt.figure(figsize=(10,10), dpi= 100)
    plt.subplot(311)
    plt.plot(data[0], data[1], label = 'Output')
    # plt.plot(data[0], data[11], 'r', label = 'rcCommand')
    plt.ylabel('p(rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.legend(loc = "best")
    # plt.text((max(data[0]) + min(data[0])) / 2, (max(data[1]) + min(data[1])) / 2, "t = %f"%t_1)
        
    plt.subplot(312)
    plt.plot(data[0], data[2], label = 'Output')
    # plt.plot(data[0], data[12], 'r', label = 'rcCommand')
    plt.ylabel('q(rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.legend(loc = "best")
    
    plt.subplot(313)
    plt.plot(data[0], data[3], label = 'Output')
    # plt.plot(data[0], data[13], 'r', label = 'rcCommand')
    plt.ylabel('r(rad/s)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.legend(loc = "best")
    

    
    plt.figure(figsize=(10,10), dpi= 100)
    plt.subplot(311)
    plt.plot(data[0], data[8])
    plt.ylabel('torque x (Nm)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data[0], data[9])
    plt.ylabel('torque y (Nm)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data[0], data[10])
    plt.ylabel('torque z (Nm)')
    plt.xlabel('t(s)')
    plt.grid(True)


    plt.figure(figsize=(10,10), dpi= 100)
    plt.plot(data[0], data[4])
    plt.plot(data[0], data[5])
    plt.plot(data[0], data[6])
    plt.plot(data[0], data[7])
    plt.legend(["r_0", "r_1", "r_2", "r_3"])
    plt.ylabel('rpm')
    plt.xlabel('t(s)')
    plt.grid(True)

    plt.figure(figsize=(10,10), dpi= 100)
    plt.plot(new_data[0], new_data[1], label = "ESC to RPM line", color = 'b')
    plt.plot(active_ESC, active_RPM, label = "active", color = 'r', linewidth = 2.0)
    plt.xlabel("ESC")
    plt.ylabel("RPM")
    plt.legend(loc = "best")
    plt.grid(True)

    # plot posture
    plt.figure(figsize=(10,10), dpi= 100)
    plt.subplot(311)
    plt.plot(data[0], data[14], label = 'Roll Posture')
    plt.plot(data[0], data[11], 'r', label = 'rcCommand')
    plt.ylabel('roll(rad)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.legend(loc = "best")
    # plt.text((max(data[0]) + min(data[0])) / 2, (max(data[1]) + min(data[1])) / 2, "t = %f"%t_1)
        
    plt.subplot(312)
    plt.plot(data[0], data[15], label = 'Pitch Posture')
    plt.plot(data[0], data[12], 'r', label = 'rcCommand')
    plt.ylabel('pitch(rad)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.legend(loc = "best")
    
    plt.subplot(313)
    plt.plot(data[0], data[16], label = 'Yaw Posture')
    plt.plot(data[0], data[13], 'r', label = 'rcCommand')
    plt.ylabel('yaw(rad)')
    plt.xlabel('t(s)')
    plt.grid(True)
    plt.legend(loc = "best")

    # plt.figure(figsize=(10,10), dpi= 100)
    # plt.plot(data[0], data[17], label = "Velocity")
    # plt.plot(data[0], data[18], label = "Acceleration")
    # plt.legend(loc = "best")
    # # plt.ylabel('rpm')
    # # plt.xlabel('t(s)')
    # plt.grid(True)

    plt.show(block=False)
    input("Press Enter to continue...")