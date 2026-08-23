#pragma once

// Accumulate serial bytes and dispatch a command on \n, \r or ':' -- the same
// line discipline as the main firmware. Oversized lines are discarded.
void readSerialCommands();

// Print the geometry, the signs, the sweep settings and a run-time estimate.
void printConfig();

// Print the command list.
void printHelp();
