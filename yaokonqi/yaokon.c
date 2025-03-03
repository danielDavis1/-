/*
 * ibus_to_uinput.c
 *
 * 示例代码：从 IBus 串口读取数据帧，解析各通道数据归一化后，
 * 通过 uinput 创建虚拟手柄设备，将数据写入系统。
 *
 * 编译：gcc -o ibus_to_uinput ibus_to_uinput.c
 * 运行（需要 root 权限）：./ibus_to_uinput /dev/ttyUSB0
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/uinput.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>

#define FRAME_SIZE 32
#define CHANNEL_COUNT 6  // 假设帧内总共有 6 个通道数据
// 假定原始数据范围（根据实际情况修改）
#define RAW_MIN 172
#define RAW_MAX 1811
// 虚拟手柄归一化范围
#define VJOY_MIN -32768
#define VJOY_MAX 32767

// 打开串口，配置波特率、8N1、无流控等
int open_serial(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("open_serial");
        return -1;
    }
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~PARENB;  // 无校验
    options.c_cflag &= ~CSTOPB;  // 1 个停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8 个数据位
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// 从帧中提取指定通道（通道从 0 开始，每个通道 2 字节，小端）
int extract_channel(uint8_t* frame, int channel_index) {
    // 假设帧第 0 字节为帧头，从第 1 字节开始每 2 字节为一个通道数据
    int offset = 1 + channel_index * 2;
    int value = frame[offset] | (frame[offset + 1] << 8);
    return value;
}

// 归一化函数：线性映射 raw ∈ [RAW_MIN, RAW_MAX] 到 [VJOY_MIN, VJOY_MAX]
int normalize_value(int raw) {
    return (raw - RAW_MIN) * (VJOY_MAX - VJOY_MIN) / (RAW_MAX - RAW_MIN) + VJOY_MIN;
}

// 设置 uinput 虚拟设备
int setup_uinput_device() {
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("open /dev/uinput");
        exit(EXIT_FAILURE);
    }
    
    // 允许发送绝对轴和同步事件
    if (ioctl(fd, UI_SET_EVBIT, EV_ABS) < 0) perror("UI_SET_EVBIT EV_ABS");
    if (ioctl(fd, UI_SET_EVBIT, EV_SYN) < 0) perror("UI_SET_EVBIT EV_SYN");
    
    // 设置支持的轴（这里使用 ABS_X、ABS_Y、ABS_RX、ABS_RY）
    if (ioctl(fd, UI_SET_ABSBIT, ABS_X) < 0) perror("UI_SET_ABSBIT ABS_X");
    if (ioctl(fd, UI_SET_ABSBIT, ABS_Y) < 0) perror("UI_SET_ABSBIT ABS_Y");
    if (ioctl(fd, UI_SET_ABSBIT, ABS_RX) < 0) perror("UI_SET_ABSBIT ABS_RX");
    if (ioctl(fd, UI_SET_ABSBIT, ABS_RY) < 0) perror("UI_SET_ABSBIT ABS_RY");
    
    // 配置虚拟设备
    struct uinput_user_dev uidev;
    memset(&uidev, 0, sizeof(uidev));
    snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Virtual IBus Joystick");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor  = 0x1234;
    uidev.id.product = 0x5678;
    uidev.id.version = 1;
    
    // 配置各轴参数
    uidev.absmin[ABS_X] = VJOY_MIN;
    uidev.absmax[ABS_X] = VJOY_MAX;
    uidev.absmin[ABS_Y] = VJOY_MIN;
    uidev.absmax[ABS_Y] = VJOY_MAX;
    uidev.absmin[ABS_RX] = VJOY_MIN;
    uidev.absmax[ABS_RX] = VJOY_MAX;
    uidev.absmin[ABS_RY] = VJOY_MIN;
    uidev.absmax[ABS_RY] = VJOY_MAX;
    
    if (write(fd, &uidev, sizeof(uidev)) < 0) {
        perror("write uidev");
        exit(EXIT_FAILURE);
    }
    
    if (ioctl(fd, UI_DEV_CREATE) < 0) {
        perror("UI_DEV_CREATE");
        exit(EXIT_FAILURE);
    }
    
    return fd;
}

// 发送 uinput 事件
void send_event(int fd, __u16 type, __u16 code, __s32 value) {
    struct input_event ev;
    memset(&ev, 0, sizeof(ev));
    gettimeofday(&ev.time, NULL);
    ev.type = type;
    ev.code = code;
    ev.value = value;
    if (write(fd, &ev, sizeof(ev)) < 0) {
        perror("write event");
    }
}

int main(int argc, char *argv[]) {
    if(argc < 2) {
        printf("Usage: %s <serial_device>\n", argv[0]);
        return 1;
    }
    const char* serial_device = argv[1];
    int serial_fd = open_serial(serial_device);
    if(serial_fd < 0) {
        return 1;
    }
    
    int uinput_fd = setup_uinput_device();
    
    uint8_t frame[FRAME_SIZE];
    while(1) {
        int bytes_read = 0;
        // 循环读满一帧数据
        while(bytes_read < FRAME_SIZE) {
            int n = read(serial_fd, frame + bytes_read, FRAME_SIZE - bytes_read);
            if(n < 0) {
                perror("read serial");
                break;
            }
            bytes_read += n;
        }
        if(bytes_read != FRAME_SIZE) {
            continue;
        }
        
        // 此处可加入校验和验证代码（根据 IBus 协议）
        
        // 提取通道数据：
        // 这里假设通道 0：左摇杆 X，1：左摇杆 Y，2：右摇杆 X，3：右摇杆 Y
        int raw_left_x = extract_channel(frame, 0);
        int raw_left_y = extract_channel(frame, 1);
        int raw_right_x = extract_channel(frame, 2);
        int raw_right_y = extract_channel(frame, 3);
        
        int norm_left_x  = normalize_value(raw_left_x);
        int norm_left_y  = normalize_value(raw_left_y);
        int norm_right_x = normalize_value(raw_right_x);
        int norm_right_y = normalize_value(raw_right_y);
        
        // 将归一化后的值写入虚拟设备
        send_event(uinput_fd, EV_ABS, ABS_X, norm_left_x);
        send_event(uinput_fd, EV_ABS, ABS_Y, norm_left_y);
        send_event(uinput_fd, EV_ABS, ABS_RX, norm_right_x);
        send_event(uinput_fd, EV_ABS, ABS_RY, norm_right_y);
        
        // 同步事件
        send_event(uinput_fd, EV_SYN, SYN_REPORT, 0);
    }
    
    // 清理工作（实际程序中可能需要捕获信号退出）
    ioctl(uinput_fd, UI_DEV_DESTROY);
    close(uinput_fd);
    close(serial_fd);
    return 0;
}
