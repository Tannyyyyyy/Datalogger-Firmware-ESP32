/*
 * console.h
 *
 * Small line-oriented serial console: plain lines, 115200 baud, type "help".
 *
 * This is the only way to change settings in the field, so it deliberately has
 * no dependency on the network being up.
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <Arduino.h>

class Console {
public:
    Console();

    void begin();

    // Feed one received byte. Returns true once a complete line was handled.
    bool processByte(uint8_t c);

    void printMenu();

private:
    static const size_t CMD_BUF_LEN = 128;

    char   _buf[CMD_BUF_LEN];
    size_t _len;

    void handleLine();
    void handleSet(char *args);
};

extern Console console;

#endif /* CONSOLE_H_ */
