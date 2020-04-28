# Definition of an item model for the ItemPick and BoxPick components

# Required
string type

# Supported types (depending on component)
string UNKNOWN=UNKNOWN
string RECTANGLE=RECTANGLE

# Only one of those required depending on the selected type
# In case of unknown type, these values defines an optional bounding box for the item
RangeBox unknown
RangeRectangle rectangle
