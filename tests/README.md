# Helioduino Tests

Run the host-testable core without Arduino hardware:

```sh
cmake -S tests -B build-host
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

The host suite covers elapsed-time rollover handling, signed tracking direction, motor coast behavior, incremental tracking hold logic, network correction math, binary input stability, and append-only binary record migration helpers. It also builds the full Helioduino source and exercises controller initialization, object registration and reconstruction, factory-created hardware objects, activation and panel attachments, measurement conversion, and user calibration lifecycle.

Development Arduino sketches are included for enum conversion and enum decoder export work:

* `EnumConversionTests` checks enum string conversions.
* `EnumTrieExportToCPP` exports the compact enum decoder tree.
