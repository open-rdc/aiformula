uint8 TYPE_UNKNOWN=0
uint8 TYPE_LINE_THIN=1
uint8 TYPE_VIRTUAL_LINE=2
uint8 TYPE_STOP_LINE=3

uint8 SUBTYPE_UNKNOWN=0
uint8 SUBTYPE_SOLID=1
uint8 SUBTYPE_DASHED=2
uint8 SUBTYPE_ROAD_BORDER=3
uint8 SUBTYPE_STOP_LINE=4
uint8 SUBTYPE_VIRTUAL_LINE=5

uint8 MARKING_UNKNOWN=0
uint8 MARKING_SOLID=1
uint8 MARKING_DASHED=2
uint8 MARKING_VIRTUAL=3

uint64 id
uint8 line_type
uint8 line_subtype
uint8 marking_type
bool is_observable
geometry_msgs/Point[] points
