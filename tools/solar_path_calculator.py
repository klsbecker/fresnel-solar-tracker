#!/usr/bin/env python3

import ephem
import pytz
from datetime import datetime, timedelta
import numpy as np
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
import time

"""
Linear Fresnel Solar Tracker

Universidade do Vale do Rio dos Sinos - UNISINOS

Authors:
- Klaus Becker

Description:
This script calculates and visualizes the solar elevation and azimuth angles
throughout a given day for a specified location. The script includes functions
for both real-time calculation of solar angles and the generation of a daily
graph with sunrise and sunset markers.

"""

# Constants
LONGITUDE = '-51.15282805318066'
LATITUDE = '-29.792892732889147'
TIMEZONE = 'America/Sao_Paulo'
MQTT_BROKER = '4.3.2.1'
MQTT_PORT = 1883
MQTT_TOPIC = 'desired-angle'


# MQTT client setup
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    """Callback for successful MQTT connection"""
    print(f"Connected to MQTT broker with result code {rc}")
    client.subscribe(MQTT_TOPIC)

client.on_connect = on_connect

# Connect to the broker
client.connect(MQTT_BROKER, MQTT_PORT, 60)

def calculate_solar_angles(lat, lon, local_datetime, timezone_str=None):
    """
    Calculate the solar elevation and azimuth angles for a given location and datetime.

    :param lat: Latitude of the location in degrees (string)
    :param lon: Longitude of the location in degrees (string)
    :param local_datetime: Local datetime object
    :param timezone_str: Timezone string (optional; if None, uses system timezone)
    :return: Tuple containing the solar elevation angle and azimuth angle in degrees
    """
    observer = ephem.Observer()
    observer.lat = lat
    observer.lon = lon

    # Handle timezone if not provided
    if timezone_str is None:
        timezone_str = pytz.localzone().zone
    local_tz = pytz.timezone(timezone_str)

    # Convert local time to UTC
    local_datetime = local_tz.localize(local_datetime)
    observer.date = local_datetime.astimezone(pytz.utc)

    # Calculate solar position
    sun = ephem.Sun(observer)
    azimuth_deg = 180 - (np.rad2deg(sun.az) + 180) % 360
    elevation_deg = np.rad2deg(sun.alt)

    return elevation_deg, azimuth_deg

def plot_solar_angles(lat, lon, local_datetime, timezone_str=None):
    """
    Generate a graph of solar elevation and azimuth angles throughout the day.

    :param lat: Latitude of the location in degrees (string)
    :param lon: Longitude of the location in degrees (string)
    :param local_datetime: Local datetime object
    :param timezone_str: Timezone string (optional; if None, uses system timezone)
    """
    hours = np.arange(0, 24, 1/60)  # Calculate angles every minute
    azimuths = []
    elevations = []

    # Calculate angles for each minute of the day
    for hour in hours:
        datetime_current = local_datetime.replace(hour=int(hour), minute=int((hour % 1) * 60), second=0, microsecond=0)
        elevation, azimuth = calculate_solar_angles(lat, lon, datetime_current, timezone_str)
        azimuths.append(azimuth)
        elevations.append(elevation)

    # Convert lists to numpy arrays
    elevations = np.array(elevations)

    # Identify sunrise and sunset times
    sunrise_idx = np.argmin(np.abs(elevations[:len(elevations)//2]))
    sunset_idx = np.argmin(np.abs(elevations[len(elevations)//2:])) + len(elevations)//2

    sunrise_time = hours[sunrise_idx]
    sunset_time = hours[sunset_idx]

    # Convert fractional hours to HH:MM format
    sunrise_str = (datetime.combine(local_datetime.date(), datetime.min.time()) + timedelta(hours=sunrise_time)).strftime('%H:%M')
    sunset_str = (datetime.combine(local_datetime.date(), datetime.min.time()) + timedelta(hours=sunset_time)).strftime('%H:%M')

    # Plot the graph
    plt.figure(figsize=(10, 6))
    plt.plot(hours, elevations, label='Elevation', color='blue', linewidth=2)
    plt.plot(hours, azimuths, label='Azimuth', color='green', linewidth=2)

    # Add points for sunrise and sunset
    plt.scatter([sunrise_time, sunset_time], [0, 0], color='red', zorder=5)

    # Annotate sunrise and sunset times
    plt.text(sunrise_time, 0, f'Sunrise\n{sunrise_str}', color='orange', ha='center', va='bottom', fontsize=10, fontweight='bold')
    plt.text(sunset_time, 0, f'Sunset\n{sunset_str}', color='purple', ha='center', va='bottom', fontsize=10, fontweight='bold')

    plt.xlabel('Hour of the day', fontsize=12)
    plt.ylabel('Angle in degrees', fontsize=12)
    plt.title('Solar Elevation and Azimuth Angles Throughout the Day', fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.6)

    plt.axhline(y=0, color='r', linestyle='--')  # Add a line at y=0
    plt.axvline(x=sunrise_time, color='orange', linestyle='--', label=f'Sunrise ({sunrise_str})')
    plt.axvline(x=sunset_time, color='purple', linestyle='--', label=f'Sunset ({sunset_str})')

    plt.xlim(0, 24)  # Set x-axis limits
    plt.ylim(-180, 180) # Set y-axis limits

    plt.legend(loc='upper left', fontsize=10)
    plt.show()

def get_current_solar_angles(lat, lon, timezone_str=None):
    """
    Get the current solar elevation and azimuth angles.

    :param lat: Latitude of the location in degrees (string)
    :param lon: Longitude of the location in degrees (string)
    :param timezone_str: Timezone string (optional; if None, uses system timezone)
    :return: Tuple containing the current solar elevation and azimuth angles in degrees
    """
    current_datetime = datetime.now()
    elevation, azimuth = calculate_solar_angles(lat, lon, current_datetime, timezone_str)

    return elevation, azimuth

def publish_solar_angles(lat, lon, timezone_str=None):
    """Publish current solar angles to MQTT topic every 1 minute"""
    while True:
        # Get current solar angles
        current_elevation, current_azimuth = get_current_solar_angles(lat, lon, timezone_str)

        # Publish the azimuth as a message
        message = f'{current_azimuth:.2f}'
        print(f'Current Azimuth: {message}', flush=True)

        client.publish(MQTT_TOPIC, message)

        # Wait for 1 minute
        time.sleep(60)

# Main script execution
if __name__ == "__main__":
    # Start MQTT client loop in a separate thread
    client.loop_start()

    # Start publishing solar angles
    publish_solar_angles(LATITUDE, LONGITUDE, TIMEZONE)
