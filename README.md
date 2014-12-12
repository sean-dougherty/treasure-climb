#Intro#

This is a game for the Atari 2600, written in 6502 assembler. I'll hopefully
finish it in the next few years. As of now, I just have basic player motion
and a simple render kernel. The character can so far do the following:

* walk left/right (joystick left/right)
* run left/right (double-tap joystick left/right)
* jump (fire button)
* high jump (jump within a few milliseconds of landing)
* wall grab (jump into a wall)
* wall jump (jump will grabbing a wall)

The "physics engine" has a concept of maximum acceleration and inertia. The
default settings have the character behave as if it's on ice.

Eventually, I want the character to jump around in a vertically scrolling
platform/wall environment, collecting treasure while doing so. As more
treasure is collected, the player will have more inertia and won't be
able to jump as high and will take longer to get to running speed or
come to a stop. I also want to add the ability for the character to hang
from ledges.

My game already uses 600 bytes of ROM just for the player logic listed
above, which is quite a bit, considering that I wish to use the original
4k ROM size. It will be a challenge to add sprite graphics and maze
definitions in the remaining ROM space.

#Building/Running#

Building requires DASM (I use version 2.20.11) and GNU Make. The Makefile
requires definition of the environment variable `DASM_ATARI_INCLUDE`,
which should provide the necessary command-line flag for DASM to find
the files `vcs.h` and `macro.h`. On my system, I use:
```
export DASM_ATARI_INCLUDE="-I/home/dougher1/atari/dasm/dasm-dillon-code-327-trunk/machines/atari2600/"
```

With that defined, you can just invoke `make`, and the resulting ROM image
will be contained in `treasure-climb.bin`.

The game can then be executed via (with debugger enabled):
```
stella -debug treasure-climb.bin
```

#Useful Links#

I found the following links very helpful in learning how to program the Atari
2600:

* [6502 Programmer Reference](http://homepage.ntlworld.com/cyborgsystems/CS_Main/6502/6502.htm])
* [Atari 2600 Programming for Newbies](http://www.randomterrain.com/atari-2600-memories-tutorial-andrew-davie-01.html)
* [DASM Documentation](http://www.macs.hw.ac.uk/~pjbk/scholar/dasm.html)