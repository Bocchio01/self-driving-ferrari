/**
 * Based on the following code:
 *
 * _kbhit.cpp
 *
 * Linux (POSIX) implementation of _kbhit().
 * Morgan McGuire, morgan@cs.brown.edu
 *
 * @link https://www.flipcode.com/archives/_kbhit_for_Linux.shtml
 *
 */

#pragma once

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>

class KBHit
{
private:
    bool is_initialized;
    struct termios termios_original;

    void init(bool disable_echoing = true);
    void reset();

public:
    KBHit();
    ~KBHit();

    int hit();
};
