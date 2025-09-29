#!/usr/bin/env python3
import requests
import argparse
from pathlib import Path
import re

def get_location_from_ip():
    """
    Get approximate latitude and longitude from public IP (using ipinfo.io).
    Useful for quick testing without manually providing coordinates.
    """
    try:
        resp = requests.get("https://ipinfo.io/json", timeout=5)
        data = resp.json()
        lat, lon = map(float, data["loc"].split(","))
        return lat, lon
    except Exception as e:
        raise RuntimeError(f"Failed to fetch IP location: {e}")

def update_world_file(world_file, lat, lon, elevation=0.0, heading=0.0):
    """
    Update a Gazebo world (.world) file with new spherical coordinates.
    If the <spherical_coordinates> block exists, it will be replaced.
    Otherwise, a new block will be inserted before </world>.
    """
    world_file = Path(world_file).expanduser()
    if not world_file.exists():
        raise FileNotFoundError(f"World file not found: {world_file}")

    text = world_file.read_text()

    if "<spherical_coordinates>" in text:
        # Replace existing coordinate values
        text = re.sub(r"<latitude_deg>.*?</latitude_deg>", f"<latitude_deg>{lat}</latitude_deg>", text)
        text = re.sub(r"<longitude_deg>.*?</longitude_deg>", f"<longitude_deg>{lon}</longitude_deg>", text)
        text = re.sub(r"<elevation>.*?</elevation>", f"<elevation>{elevation}</elevation>", text)
        text = re.sub(r"<heading_deg>.*?</heading_deg>", f"<heading_deg>{heading}</heading_deg>", text)
    else:
        # Insert a new spherical_coordinates block
        insert_block = f"""
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{elevation}</elevation>
      <heading_deg>{heading}</heading_deg>
    </spherical_coordinates>
    """
        text = text.replace("</world>", insert_block + "\n  </world>")

    world_file.write_text(text)
    print(f"✅ Updated {world_file}")
    print(f"🌍 World origin set to: lat={lat}, lon={lon}, elevation={elevation}, heading={heading}")

def main():
    parser = argparse.ArgumentParser(description="Set origin coordinates in a PX4 Gazebo world file")
    parser.add_argument("world_file", type=str, help="Path to the .world file (e.g. empty.world)")
    parser.add_argument("--lat", type=float, help="Latitude (manual mode)")
    parser.add_argument("--lon", type=float, help="Longitude (manual mode)")
    parser.add_argument("--elev", type=float, default=0.0, help="Elevation in meters (default=0)")
    parser.add_argument("--heading", type=float, default=0.0, help="Heading angle in degrees (default=0)")
    parser.add_argument("--auto", action="store_true", help="Automatically get coordinates from IP")

    args = parser.parse_args()

    if args.auto:
        lat, lon = get_location_from_ip()
        print(f"📡 Auto location detected: lat={lat}, lon={lon}")
    else:
        if args.lat is None or args.lon is None:
            parser.error("Manual mode requires --lat and --lon (or use --auto)")
        lat, lon = args.lat, args.lon

    update_world_file(args.world_file, lat, lon, args.elev, args.heading)

if __name__ == "__main__":
    main()

# python3 set_origin_pose.py /home/lingzp/workspace/code/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world --auto
# python3 set_origin_pose.py /home/lingzp/workspace/code/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world --lat 37.7749 --lon -122.4194 --elev 10 --heading 90

