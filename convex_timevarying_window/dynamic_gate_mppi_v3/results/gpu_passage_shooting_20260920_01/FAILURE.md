# Construction failure, not a flight

2026-09-20: NVRTC compilation failed before controller construction because the generated CUDA used an unresolved integer-power overload (`_Z5powiffi`). No model step or PX4 flight occurred. The source snapshot is retained. The next revision replaces these powers by multiplication and uses a new result directory.
