# Machine name, type, and team
string    name
string    type
string    state
int8      team_color

# Which zone the machine is in, will be
# 0 during exploration phase (therefore not
# identify the actual zone)
int8      zone

# For RS also denote ring colors
int8[] rs_ring_colors
