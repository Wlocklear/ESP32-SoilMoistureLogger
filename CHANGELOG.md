# Changelog

## v1.1

- **Fixed calibration capturing a smoothed value instead of the true raw reading.** Pressing Set DRY/WET point in quick succession could collapse both calibration points to nearly the same value.
- **Disabled WiFi modem sleep.** Power-save mode was cycling the radio on/off, injecting noise bursts into the ADC1 pin shared with the soil sensor.
- **Widened moisture-read averaging to a ~3.6s trimmed mean** (was a single ~192ms sample burst), taken on every reading — boot, Instant Reading, calibration, and the logging interval — to average out sensor drift instead of sampling a single noisy instant.
- **Fixed a server deadlock** caused by a leftover "live ADC feed" that polled the sensor every 2 seconds from the Calibration page. Once readings were slowed down for averaging (~3.6s), that faster polling interval piled up requests and permanently wedged the single-threaded web server.
- **Extended the logging sample interval to 10 minutes** (was 5 minutes) to smooth out readings in the saved history.

## v1.0

- Initial release.
