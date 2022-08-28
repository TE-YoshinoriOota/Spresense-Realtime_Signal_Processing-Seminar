#include <MP.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>   
#include <nuttx/timers/pwm.h>
int fd;
struct pwm_info_s info;

const int SPACE = 38000;
const int MARK  = 40000;

void send_signal(uint16_t output_hz) {
  info.frequency = output_hz; // 40kHz
  info.duty      = 0x7fff;
  ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  ioctl(fd, PWMIOC_START, 0);  
  delayMicroseconds(21333);  
}

void setup() {
  MP.begin();
  fd = open("/dev/pwm0", O_RDONLY);
  info.frequency = MARK; 
  info.duty      = 0x7fff;
  ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&info));
  ioctl(fd, PWMIOC_START, 0);   
}

void encode(uint8_t c) {
  send_signal(SPACE);  // send start bit
  for (int n = 0; n < 8; ++n, c = c >> 1) { /* LSB first */
    if (c & 0x01) { /* mark (1) */
      send_signal(MARK);
    } else { /* space (0) */
      send_signal(SPACE);
    }
  } 
  send_signal(MARK);  // send stop bit
}

void loop() {
  const char* const str = "HELLO SPRESENSE\n";
  int n = strlen(str);

  for (int i = 0; i < n; ++i) {
    encode(str[i]);
  }
  delay(100);
}
