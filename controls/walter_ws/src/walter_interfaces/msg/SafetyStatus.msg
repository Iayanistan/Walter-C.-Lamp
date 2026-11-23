# Published by the safety filter at 10 Hz.
builtin_interfaces/Time stamp
string mode              # IDLE | SCRIPTED | VLA | HALTED (as last received from the supervisor)
uint32 chunks_forwarded
uint32 chunks_rejected
uint32 chunks_retimed    # forwarded, but slowed down to respect velocity limits
string last_reason       # why the last chunk was rejected or retimed; empty if none
