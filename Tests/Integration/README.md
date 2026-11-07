# Integration Tests

Reserved for tests that exercise more than one module together against real
or emulated hardware (e.g. flash the firmware, drive it over the USART1 UART
protocol described in the root README, and assert on the replies) - as
opposed to `Tests/UnitTest/`, which runs host-native against fakes.

Nothing is implemented here yet; no framework or CMake wiring has been
decided. This file exists so the folder's purpose doesn't have to be guessed
from an empty directory.
