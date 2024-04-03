uint8[] background_data  # length always 0 or 1

# TODO: Need to split send-once out, but AFTER the other renaming.
uint8[] send_once_topics


# one-byte codes that map to a topic and message type
uint8[] queue_topics
# What mode to queue data for the corresponding topic
# 0 is RPC (next message transmitted)
# 1-63 is sampled every i-th transmit cycle
# 255 is service call with fixed number of bytes
# TODO: service calls (vs single-shot messages) are NYI
uint8[] queue_periods
