
# Maybe our 2-parameter header?  We definitely need to provide loopback detection...

# Time message originated
time stamp

# Used primarily for loopback detection
string source

# Key/value pairs for all 5 principal types
KeyBool[] bools
KeyInt[] ints
KeyFloat[] floats
KeyDouble[] doubles
KeyString[] strings

