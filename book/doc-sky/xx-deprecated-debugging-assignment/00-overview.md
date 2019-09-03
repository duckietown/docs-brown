# Debugging {#part:debugging status=ready}

This unit focuses on methods and strategies for debugging robots.

Most of the time spent writing a program is spent debugging that program. This
issue is particularly challenging for robotics because a robot will not work
unless *everything* else works. During the development of this drone project,
we have had our robots fail because of:

* a bug in our program
* a bug in the library we were calling
* bad electrical wiring
* inadequate cooling
* inadequate circuits

Despite ours and your best efforts, you will encounter bugs when building and
flying your drone. We expect this to happen, and part of the goal for this
assignment is to teach methods and strategies for debugging a robot.
Fundamentally, debugging is about checking your assumptions and localizing the
problem. You need to be systematic and verify each part of the system is
working (or not) when finding a bug.

Often, bugs are present in a sequence: one bug masks a second one. So if you
fix the first bug, it still doesn't work, because now a second problem comes
into play. Don't let this get you down! Expect it. As you work on each project,
you should expect that you did ten things wrong, that you'll have to find and
fix. So if you find one thing and fix it, expect that there are nine more
things you'll have to fix before you can fly.
