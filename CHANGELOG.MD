# Changelog

## v0.3.0 - 09.07.2021

### Added
- Added DC motors support for STM32 program
- Added conversion algorithm from joystick coordinates to PWM
- Added emergency stop while Android app is closed or minimized

### Fixed
- Fixed hanging android app after minimize
- Fixed rough motors stop after moving backwards
- Fixed unsuccessful connection to STM32 due to packet data shift

## v0.2.0 - 02.06.2021

### Added
- Added STM32 program
- Added latency test for Android app

## v0.1.0 - 29.04.2021

### Added
- Added Android app
- Added client-server communication between Android app and desktop

### Changed
- Changed Kotlin to Java (I'm not too experienced with Kotlin)
