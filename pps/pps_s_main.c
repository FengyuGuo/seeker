#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <termios.h>

struct pps_packet {
    uint32_t header;
    uint64_t sec;
    uint64_t nsec;
    uint32_t footer;
} __attribute__ ((packed));

struct kernel_timespec64 {
    int64_t tv_sec;   // 8字节有符号
    int64_t tv_nsec;  // 8字节（实际内核用long，但用户空间统一用64位）
};

static int uart_fd = -1;
static int pps_s_fd = -1;

// CRC-32 校验
uint32_t crc32(const void *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t *bytes = (const uint8_t *)data;

    for (size_t i = 0; i < length; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }

    return ~crc;
}

// 准备数据包
void prepare_packet(struct kernel_timespec64 *ts, struct pps_packet *packet) {
    packet->header = 0XAA;
    packet->sec = ts->tv_sec;
    packet->nsec = ts->tv_nsec;
    packet->footer = crc32(&packet->sec, sizeof(packet->sec) + sizeof(packet->nsec));
}

void print_timestamp(const struct kernel_timespec64 *ts) {
    printf("tv_sec: %lld, tv_nsec: %lld\n",
	ts->tv_sec, ts->tv_nsec);
}

int configure_uart(int fd, int baud_rate, int data_bits, char parity, int stop_bits) {
    struct termios options;

    // 获取当前配置
    if (tcgetattr(fd, &options)) {
        perror("Failed to get UART attributes");
        return -1;
    }

    // 设置波特率
    speed_t speed;
    switch (baud_rate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            fprintf(stderr, "Unsupported baud rate\n");
            return -1;
    }
    cfsetispeed(&options, speed); // 输入波特率
    cfsetospeed(&options, speed); // 输出波特率

    // 设置数据位
    options.c_cflag &= ~CSIZE; // 清除数据位掩码
    switch (data_bits) {
        case 5: options.c_cflag |= CS5; break;
        case 6: options.c_cflag |= CS6; break;
        case 7: options.c_cflag |= CS7; break;
        case 8: options.c_cflag |= CS8; break;
        default:
            fprintf(stderr, "Unsupported data bits\n");
            return -1;
    }

    // 设置校验位
    switch (parity) {
        case 'N': // 无校验
            options.c_cflag &= ~PARENB;
            break;
        case 'O': // 奇校验
            options.c_cflag |= PARENB;
            options.c_cflag |= PARODD;
            break;
        case 'E': // 偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            break;
        default:
            fprintf(stderr, "Unsupported parity\n");
            return -1;
    }

    // 设置停止位
    if (stop_bits == 1) {
        options.c_cflag &= ~CSTOPB; // 1 位停止位
    } else if (stop_bits == 2) {
        options.c_cflag |= CSTOPB;  // 2 位停止位
    } else {
        fprintf(stderr, "Unsupported stop bits\n");
        return -1;
    }

    // 其他配置
    options.c_cflag |= (CLOCAL | CREAD); // 本地连接，启用接收
    // options.c_oflag &= ~OPOST; // 禁用输出处理
    // options.c_cflag &= ~CRTSCTS; // 禁用硬件流控
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始模式
    options.c_oflag &= ~OPOST; // 原始输出
    // 设置超时和最小字符数
    options.c_cc[VMIN] = 0;  // 最小字符数
    options.c_cc[VTIME] = 0; // 超时时间（单位：0.1 秒）

    // 应用配置
    if (tcsetattr(fd, TCSANOW, &options)) {
        perror("Failed to set UART attributes");
        return -1;
    }

    return 0;
}

int main() {
    int ret = 0;
    struct pps_packet st_pps_packet;
    struct kernel_timespec64 ts;

    pps_s_fd = open("/dev/pps_s", O_RDONLY);
    if (pps_s_fd < 0) {
        printf("Failed to open pps device\n");
        return -1;
    }

    uart_fd = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) {
        printf("Failed to open uart device\n");
        ret = -1;
        goto __UART_OPEN_FAIL__;
    }

    // 配置 UART
    if (configure_uart(uart_fd, 115200, 8, 'N', 1)) {
        printf("Failed to configure_uart\n");
        ret = -1;
        goto __UART_OPEN_FAIL__;
    }

    tcflush(uart_fd, TCOFLUSH);
    while (1)
    {
        if(read(pps_s_fd, &ts, sizeof(ts)) == sizeof(ts))
        {
            prepare_packet(&ts, &st_pps_packet);
            ssize_t bytes_written = write(uart_fd, &st_pps_packet,
            sizeof(struct pps_packet));
            if (bytes_written < 0) {
                printf("Failed to write to UART\n");
            }

            print_timestamp(&ts);
        }
    }

__UART_OPEN_FAIL__:
    close(pps_s_fd);
    return ret;
}

