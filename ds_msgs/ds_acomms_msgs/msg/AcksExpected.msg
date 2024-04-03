# Progressive Imagery requires acknowlegement for each
# packet sent, or it tries to resend. This message tells
# the sender that it *should* have received acks and causes
# it to reset the queue.

# TODO: In the name of generality, I've split the
# "reset queue" field from the "please provide data" service,
# but I don't know if any other nodes will care about this,
# or what other information should be attached.
# (Standard header? Time since previous transmit slot?)

# TODO: proper handling of this would require the sender/subsea
# QM to know the TDMA schedule for the topside QM in order to
# infer that it _should_ have heard something. Right now,
# the assumption is that the gap in between two subsea slots
# always corresponds to a topside slot.