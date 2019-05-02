# Definition of an item model for the itempick component

# Required
string type

# Supported types
string UNKNOWN=UNKNOWN
string BOX=BOX
# string CYLINDER=CYLINDER
# string SPHERE=SPHERE

# Only one of those required depending on the selected type
# In case of unknown type, these values defines an optional bounding box for the item
RangeBox unknown
RangeBox box
