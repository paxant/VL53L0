#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <stdlib.h>

#define I2C_DEV "/dev/i2c-1"
#define VL53L0X_ADDR 0x29

int i2c_write_reg(int fd, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return write(fd, buf, 2);
}

int i2c_read_reg(int fd, uint8_t reg, uint8_t *value) {
    if (write(fd, &reg, 1) != 1) return -1;
    return read(fd, value, 1);
}

int i2c_read16(int fd, uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, buf, 2) != 2) return -1;
    *value = (buf[0] << 8) | buf[1];
    return 0;
}

void delay_ms(int ms) {
    usleep(ms * 1000);
}

int main() {
    int fd = open(I2C_DEV, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, VL53L0X_ADDR) < 0) {
        perror("ioctl");
        close(fd);
        return 1;
    }

    printf("Booting sensor...\n");
    delay_ms(200); // Ждем, пока он загрузится

    // Установка magic регистров — без них он не работает
    // Настроим single ranging mode (по даташиту)

    i2c_write_reg(fd, 0x88, 0x00);
    i2c_write_reg(fd, 0x80, 0x01);
    i2c_write_reg(fd, 0xFF, 0x01);
    i2c_write_reg(fd, 0x00, 0x00);
    i2c_write_reg(fd, 0x91, 0x3c);
    i2c_write_reg(fd, 0x00, 0x01);
    i2c_write_reg(fd, 0xFF, 0x00);
    i2c_write_reg(fd, 0x80, 0x00);

    while (1) {
        // Запуск одиночного измерения
        i2c_write_reg(fd, 0x00, 0x01); // SYSRANGE_START

        // Ждём окончания
        uint8_t status = 0;
        int attempts = 0;
        do {
            i2c_read_reg(fd, 0x13, &status); // RESULT_INTERRUPT_STATUS
            attempts++;
            usleep(1000);
        } while ((status & 0x07) == 0 && attempts < 100);

        // Чтение расстояния
        uint16_t distance = 0;
        i2c_read16(fd, 0x1E, &distance); // RESULT_RANGE_STATUS + 10 (0x14 + 0x0A)

        printf("Distance: %d mm\n", distance);

        // Очистка флага прерывания
        i2c_write_reg(fd, 0x0B, 0x01); // SYSTEM_INTERRUPT_CLEAR

        sleep(1);
    }

    close(fd);
    return 0;
}
