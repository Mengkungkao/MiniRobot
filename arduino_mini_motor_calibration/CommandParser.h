#pragma once

// Accumulate serial bytes and dispatch a command on \n, \r or ':' -- the same
// line discipline as the main firmware. Oversized lines are discarded.
void readSerialCommands();
