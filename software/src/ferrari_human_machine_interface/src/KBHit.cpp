#include "KBHit.hpp"
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <unistd.h>

KBHit::KBHit() : initialized(false)
{
    init();
}

KBHit::~KBHit()
{
    reset();
}

void KBHit::init()
{
    if (!initialized)
    {
        struct termios term;
        tcgetattr(STDIN, &term);          // Get current terminal settings
        orig_termios = term;              // Save original terminal settings
        term.c_lflag &= ~ICANON;          // Disable canonical mode (buffering)
        tcsetattr(STDIN, TCSANOW, &term); // Apply new settings
        setbuf(stdin, NULL);              // Disable stdout buffering
        initialized = true;
    }
}

void KBHit::reset()
{
    if (initialized)
    {
        tcsetattr(STDIN, TCSANOW, &orig_termios); // Restore original settings
        setbuf(stdin, NULL);                      // Ensure no buffering
        initialized = false;
    }
}

int KBHit::hit()
{
    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting); // Check if there are characters in stdin
    return bytesWaiting;
}
