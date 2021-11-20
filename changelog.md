## 2021-11-19

### Added 
- Children threads now wait for a signal event from the main thread, then exits gracefully from while loop. The state recording thread requires this otherwise file may not be written!
- Added multi wifi card scripts

### Changed
- state csv data now saved with time stamps in file name