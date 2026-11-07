# Unit Tests

Host-native tests, built and run with the `host-tests` CMake preset (no board,
no ARM toolchain - see the root README's *Building* section). Framework is
[Unity](https://github.com/ThrowTheSwitch/Unity), fetched automatically by
`cmake/unit_tests.cmake`.

## Mocking convention

There is no mocking framework (no CMock) - hardware-touching production code
is tested by linking it against a hand-written fake instead of the real
driver. Two reference examples, at two different layers:

- **`test_swc_controller.c`** fakes the BSP layer: it provides its own
  `USART1_Init()`/`USART1_ReadByte()`/`USART1_TransmitData()`/... so the real
  `swc_controller.c` runs unmodified against scripted RX bytes and captured TX
  bytes, with no MCAL/CMSIS involved at all.
- **`test_debounce.c`** fakes one level lower, at the MCAL layer: it provides
  its own `GPIO_ReadPin()` so the real `debounce.c` runs unmodified against a
  scripted sequence of raw pin levels and timestamps. This needs `gpio.h`'s
  `GPIO_TypeDef` type to exist on the host, which is why `mock/stm32f4xx.h`
  defines a minimal opaque stub for it - the register layout is irrelevant
  since the fake never dereferences the pointer, only the pin-level return
  value matters.

**Adding a new test**: pick the layer directly below the thing you're
testing, fake only that layer's functions, and link the real production
source for everything above it. See the root README's *Extending the
Platform* section for the exact `add_unit_test()` CMake snippet.
