# Heartbeat message that supports additional checkpoint information for
# control flow monitoring on the receiver side.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# The unique identifier of the active checkpoint.
uint16 checkpoint_id 0
