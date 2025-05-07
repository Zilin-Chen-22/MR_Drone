import matplotlib.pyplot as plt

def read_draw(k):
    fp = open("data.txt", 'r')

    new_data = [[], []]
    original_data = [[], []]

    while True:
        data_read = fp.readline()
        if data_read == '####\n':
            break
        else:
            data_read = data_read.replace("{", '')
            data_read = data_read.replace("}", '')
            data_read = data_read.split(',')
            original_data[0].append(float(data_read[0]))
            original_data[1].append(float(data_read[1]))

    while True:
        data_read = fp.readline()
        if data_read == '':
            break
        else:
            data_read = data_read.split(' ')
            new_data[0].append(float(data_read[0]))
            new_data[1].append(float(data_read[1]))
    
    fp.close()

    plt.figure(1)
    plt.plot(original_data[0], original_data[1], label = "original line", color = 'b')

    plt.plot(new_data[0], new_data[1], label = "new line", color = 'r')

    plt.xlabel("ESC")
    plt.ylabel("RPM")

    plt.legend(loc = 'best')
    plt.title("Lines for y = x ^ %f" % k)

    plt.show(block = False)

    input("Press Enter to continue...")

    

def generate(k):
    fp = open("data.txt", 'r')

    original_data = []

    while True:
        data_read = fp.readline()
        if data_read == '####\n':
            break
        else:
            original_data.append(data_read)
        
    fp.close()
    fp = open("data.txt", 'w+')
    
    for lines in original_data:
        fp.write(lines)
    
    fp.write("####\n")

    for i in range(1000, 2050, 50):
        fp.write("%d %f\n"%(i, 300 * ((i-1000) ** (k)) / (1000 ** (k))))
    fp.close()


if __name__ == "__main__":
    k = 2
    generate(k)
    read_draw(k)