#include "KBHit.hpp"
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <unistd.h>

KBHit::KBHit() : is_initialized(false)
{
    init();
}

KBHit::~KBHit()
{
    reset();
}

void KBHit::init(bool disable_echoing)
{
    if (!is_initialized)
    {
        struct termios term;
        tcgetattr(STDIN_FILENO, &term);
        termios_original = term;
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN_FILENO, TCSANOW, &term);
        setbuf(stdin, NULL);

        if (disable_echoing)
        {
            term.c_lflag &= ~ECHO;
            tcsetattr(STDIN_FILENO, TCSAFLUSH, &term);
        }

        is_initialized = true;
    }
}

void KBHit::reset()
{
    if (is_initialized)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &termios_original);
        setbuf(stdin, NULL);
        is_initialized = false;
    }
}

int KBHit::hit()
{
    int bytesWaiting;
    ioctl(STDIN_FILENO, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
