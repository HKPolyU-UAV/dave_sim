# This includes instructions on how to draw a single mission element
string role
string label         # An annotation label to add; generally blank
string wellknowntext # Well-known text, ALWAYS in lat/lon;
                     # see https://en.wikipedia.org/wiki/Well-known_text_representation_of_geometry
                     # Currently, only Point, LineString, and Polygon are supported
uint8[16] task_uuid

# Numbers are assigned somewhat randomly
string ROLE_POINT_LAUNCH="launch"
string ROLE_IDLE="idle"
string ROLE_JOYSTICK="joystick"

string ROLE_TRACKLINE="trackline"
string ROLE_TRACKLINE_CONNECTING="connecting_trackline"
string ROLE_TRACKLINE_COMPLETED="completed_trackline"
string ROLE_TRACKLINE_CURRENT="current_trackline"

string ROLE_OPERATING_AREA="operating_area"

string ROLE_GEOFENCE="geofence"
string ROLE_LOITER="loiter"
string ROLE_MANUAL="manual"
