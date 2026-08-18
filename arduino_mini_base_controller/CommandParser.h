#pragma once

// Print the boot banner and IMU info line.
void printReady();

// Accumulate serial bytes into an internal buffer and dispatch a command
// whenever a terminator (\n, \r or ':') arrives. Oversized lines are discarded.
void readSerialCommands();
