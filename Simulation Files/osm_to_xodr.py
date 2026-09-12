import carla
import io

# Read your downloaded OSM file
with io.open("map.osm", mode="r", encoding="utf-8") as f:
    osm_data = f.read()

# Configure conversion settings
settings = carla.Osm2OdrSettings()
settings.set_osm_way_types(["primary", "secondary", "tertiary", "residential", "unclassified"])
settings.generate_traffic_lights = True  # Pulls traffic lights if mapped in OSM

# Convert to OpenDRIVE format
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# Save the .xodr file
with open("intersection.xodr", "w") as f:
    f.write(xodr_data)