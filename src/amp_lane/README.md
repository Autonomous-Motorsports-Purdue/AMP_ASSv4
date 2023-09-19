## Lane Detection

This is the package for AMP lane detection. At the moment, it directly publishes to `cmd_ackermann`, which controls the simulated kart. It reads from `vehicle/camera`, which is the forward facing RGBA camera mounted on the simulaated kart. 