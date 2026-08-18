#pragma once

// Emit one 'T,...' CSV telemetry line (field layout in the top-of-sketch
// comment); consumed by the ROS 2 serial bridge.
void publishTelemetry();
