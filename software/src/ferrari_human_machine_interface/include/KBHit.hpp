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
public:
    KBHit();
    ~KBHit();

    int hit();

private:
    static const int STDIN = 0;
    bool initialized;
    struct termios orig_termios; // Store the original terminal settings
    void init();                 // Initialize terminal settings
    void reset();                // Restore original terminal settings
};
