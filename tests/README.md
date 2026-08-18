# Helioduino Tests

Run the host-testable core without Arduino hardware:

```sh
cmake -S tests -B build-host
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

The host suite covers elapsed-time rollover handling, signed tracking direction, motor coast behavior, incremental tracking hold logic, network correction math, binary input stability, and append-only binary record migration helpers.

When Python is available, CTest also runs the source validator. It checks motor persistence, tracking correction, binary sensor behavior, and several framework regressions that are easy to reintroduce during refactors.

Source checks can also be run directly:

```sh
python3 tests/validate_source.py
```

Development Arduino sketches are included for enum conversion and enum decoder export work:

* `EnumConversionTests` checks enum string conversions.
* `EnumTrieExportToCPP` exports the compact enum decoder tree.
