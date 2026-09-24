# One asynchronous supervisor operation (unified plan §4.2, §6.2): mode change,
# survey start/returned/save/abort, recovery. Bounded history; query by id
# through /amr/operations/get after a dropped HTTP response.
uint8 PENDING=0
uint8 SUCCEEDED=1
uint8 FAILED=2
uint8 INTERRUPTED=3     # supervisor restarted while it was pending; never replayed
string operation_id
string request_id
string kind                 # mode | survey_start | survey_returned | survey_save | survey_abort | recover
string phase
uint8 status
string map_id               # result identity where applicable
uint32 map_revision
string map_sha256
string message
builtin_interfaces/Time started
builtin_interfaces/Time finished
