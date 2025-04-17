#!/usr/bin/env python3

import ephem
import pytz
from datetime import datetime, timedelta
import numpy as np
import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt
import time
import argparse
import functools

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

# --- SYSTEM PARAMETERS ---
LONGITUDE = '-51.15282805318066'# Longitude of the site (degrees) used only for plotting
LATITUDE = '-29.792892732889147'# Latitude of the site (degrees) used only for plotting
Qn = 0                          # Horizontal distance between mirror and the absorber (m)
H = 3.11                        # Height of the absorber relative to the mirror plane (m)
phi = -29.7608                  # Site latitude (degrees), negative for southern hemisphere
Lst = 45                        # Standard meridian longitude for time zone (degrees, UTC-3)
Lloc = 51.1522                  # Local longitude (degrees)
N = 365                         # Total number of days in a year
TIMEZONE = 'America/Sao_Paulo'  # Timezone string

# --- MQTT PARAMETERS ---
MQTT_BROKER = '192.168.20.111'         # MQTT broker address
MQTT_PORT = 1883                # MQTT broker port
MQTT_TOPIC = 'desired-angle'    # MQTT topic to publish to

print = functools.partial(print, flush=True)

def on_connect(client, userdata, flags, rc):
    """Callback for successful MQTT connection"""
    print(f"Connected to MQTT broker with result code {rc}")
    client.subscribe(MQTT_TOPIC)

def calculate_elevation_azimuth(lat, lon, local_datetime, timezone_str=None):
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
    desired_angles = []

    # Calculate angles for each minute of the day
    for hour in hours:
        datetime_current = local_datetime.replace(hour=int(hour), minute=int((hour % 1) * 60), second=0, microsecond=0)
        elevation, azimuth = calculate_elevation_azimuth(lat, lon, datetime_current, timezone_str)
        azimuths.append(azimuth)
        elevations.append(elevation)
        desired_angle = calculate_mirror_angle_from_datetime(datetime_current)
        desired_angles.append(desired_angle)

    # Convert lists to numpy arrays
    elevations = np.array(elevations)

    # Identify sunrise and sunset times
    sunrise_idx = np.argmin(np.abs(elevations[:len(elevations)//2]))
    sunset_idx = np.argmin(np.abs(elevations[len(elevations)//2:])) + len(elevations)//2
    solar_noon_idx = np.argmax(elevations)

    sunrise_time = hours[sunrise_idx]
    sunset_time = hours[sunset_idx]
    solar_noon_time = hours[solar_noon_idx]

    # Convert fractional hours to HH:MM format
    sunrise_str = (datetime.combine(local_datetime.date(), datetime.min.time()) + timedelta(hours=sunrise_time)).strftime('%H:%M')
    sunset_str = (datetime.combine(local_datetime.date(), datetime.min.time()) + timedelta(hours=sunset_time)).strftime('%H:%M')
    solar_noon_str = (datetime.combine(local_datetime.date(), datetime.min.time()) + timedelta(hours=solar_noon_time)).strftime('%H:%M')

    # Plot the graph
    plt.figure(figsize=(10, 6))
    plt.plot(hours, elevations, label='Elevation', color='blue', linewidth=2)
    plt.plot(hours, azimuths, label='Azimuth', color='green', linewidth=2)
    plt.plot(hours, desired_angles, label='Desired Angle', color='gray', linewidth=2)

    # Add points for sunrise, sunset, and solar noon
    plt.scatter([sunrise_time, sunset_time, solar_noon_time], [0, 0, 0], color='red', zorder=5)

    plt.xlabel('Hour of the day', fontsize=12)
    plt.ylabel('Angle in degrees', fontsize=12)
    plt.title('Solar Elevation and Azimuth Angles Throughout the Day', fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.6)

    plt.axhline(y=0, color='r', linestyle='--')  # Add a line at y=0
    plt.axvline(x=sunrise_time, color='orange', linestyle='--', label=f'Sunrise ({sunrise_str})')
    plt.axvline(x=sunset_time, color='purple', linestyle='--', label=f'Sunset ({sunset_str})')
    plt.axvline(x=solar_noon_time, color='blue', linestyle='--', label=f'Solar Noon ({solar_noon_str})')

    plt.xlim(0, 24)  # Set x-axis limits
    plt.ylim(-180, 180) # Set y-axis limits

    plt.legend(loc='upper left', fontsize=10)
    plt.show()


def calculate_Gamma(day_of_year: int) -> float:
    """
    Calculate the Earth's position in its orbit (Γ) in degrees.
    """
    return (day_of_year - 1) * (360 / N)

def calculate_solar_declination(Gamma: float) -> float:
    """
    Returns the solar declination δ (degrees) based on Gamma (orbital position).
    """
    G = np.deg2rad(Gamma)
    delta_rad = (
        0.006918 - 0.399912*np.cos(G) + 0.070257*np.sin(G)
        - 0.006758*np.cos(2*G) + 0.000907*np.sin(2*G)
        - 0.002697*np.cos(3*G) + 0.00148*np.sin(3*G)
    )
    return np.rad2deg(delta_rad)

def calculate_equation_of_time(Gamma: float) -> float:
    """
    Returns the equation of time (Et) in minutes.
    Represents the difference between apparent solar time and mean time.
    """
    G = np.deg2rad(Gamma)
    return (
        0.000075 + 0.001868*np.cos(G) - 0.032077*np.sin(G)
        - 0.014615*np.cos(2*G) - 0.04089*np.sin(2*G)
    ) * 229.18

def calculate_true_solar_time(TO: float, Et: float) -> float:
    """
    Returns the True Solar Time (TST) in hours.
    TO = local clock time in minutes.
    """
    return (TO + 4*(Lst - Lloc) + Et) / 60

def calculate_hour_angle(TST: float) -> float:
    """
    Returns the hour angle ω (degrees), where 0° = solar noon.
    """
    return (TST - 12) * 15

def calculate_zenith_angle(phi: float, delta: float, omega: float) -> float:
    """
    Returns the solar zenith angle θz (rad), angle between the Sun and the zenith.
    """
    phi_rad = np.deg2rad(phi)
    delta_rad = np.deg2rad(delta)
    omega_rad = np.deg2rad(omega)
    cos_theta_z = (
        np.cos(phi_rad)*np.cos(delta_rad)*np.cos(omega_rad) +
        np.sin(phi_rad)*np.sin(delta_rad)
    )
    return np.arccos(cos_theta_z)

def calculate_solar_azimuth(phi: float, delta: float, omega: float, theta_z: float) -> float:
    """
    Returns the solar azimuth angle γm (rad), measured from the south (in the southern hemisphere).
    """
    omega_rad = np.deg2rad(omega)
    phi_rad = np.deg2rad(phi)
    delta_rad = np.deg2rad(delta)
    numerator = np.cos(theta_z)*np.sin(phi_rad) - np.sin(delta_rad)
    denominator = np.sin(theta_z)*np.cos(phi_rad)
    return np.sign(omega_rad) * abs(np.arccos(numerator / denominator))

def calculate_lambdaa(theta_z: float, gamma_m: float) -> float:
    """
    Calculates the angle of incidence λ between the solar beam and the mirror plane.
    """
    return np.arctan(np.sin(theta_z) * np.sin(gamma_m) / np.cos(theta_z))

def calculate_mirror_angle(Qn: float, H: float, lambdaa: float) -> float:
    """
    Calculates the mirror tilt angle to reflect sunlight to the absorber.
    """
    return (np.arctan(Qn / H) + lambdaa) / 2

def calculate_mirror_angle_from_datetime(dt: datetime) -> float:
    """
    Main function: returns the optimal mirror angle (in degrees)
    for a given local datetime.
    """
    day_of_year = dt.timetuple().tm_yday
    TO = dt.hour * 60 + dt.minute + dt.second / 60  # local time in minutes

    Gamma = calculate_Gamma(day_of_year)
    delta = calculate_solar_declination(Gamma)
    Et = calculate_equation_of_time(Gamma)
    TST = calculate_true_solar_time(TO, Et)
    omega = calculate_hour_angle(TST)
    theta_z = calculate_zenith_angle(phi, delta, omega)
    gamma_m = calculate_solar_azimuth(phi, delta, omega, theta_z)
    lambdaa = calculate_lambdaa(theta_z, gamma_m)
    mirror_rad = calculate_mirror_angle(Qn, H, lambdaa)

    return np.rad2deg(mirror_rad)

def main():
    # Argument parser
    parser = argparse.ArgumentParser(description='Calculate solar angle for heliostat.')
    parser.add_argument('--mode', choices=['plot', 'mqtt', 'show'], default='mqtt',
                        help='Mode of operation: plot, mqtt, or show')
    args = parser.parse_args()

    if args.mode == 'plot':
        print("Running in plot mode")
        plot_solar_angles(LATITUDE, LONGITUDE, datetime.now(), TIMEZONE)

    elif args.mode == 'mqtt':
        print("Running in MQTT mode")

        def on_connect(client, userdata, flags, rc):
            if rc == mqtt.CONNACK_ACCEPTED:
                print(f"Connected to MQTT broker [{MQTT_BROKER}]")
                client.subscribe(MQTT_TOPIC)
            else:
                print(f"Connection failed to MQTT broker [{MQTT_BROKER}] with code [{rc}]")

        def on_disconnect(client, userdata, flags, rc):
            print(f"Disconnected to MQTT broker [{MQTT_BROKER}] with code [{rc}]")

        client = mqtt.Client(client_id="fresnel-calculator")
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect

        def connect_client(start_loop=False):
            try:
                client.connect(MQTT_BROKER, MQTT_PORT, 5)
                if start_loop: client.start_loop()
                start_time = time.time()
                while time.time() - start_time < 5 and not client.is_connected():
                    time.sleep(0.1)
            except:
                pass

        connect_client(start_loop=True)

        while True:
            start_loop_time = time.time()

            if not client.is_connected():
                connect_client()

            angle = calculate_mirror_angle_from_datetime(datetime.now())

            try:
                result = client.publish(MQTT_TOPIC, f"{angle:.2f}")
                result.wait_for_publish(timeout=1)
                publish_status = "Published" if result.is_published() else "Not Published"
            except Exception as e:
                publish_status = e

            print(f"Desired Angle [{angle:.2f}] - MQTT [{publish_status}]")

            while time.time() - start_loop_time < 5:
                time.sleep(0.1)

        client.loop_stop()

    elif args.mode == 'show':
        print("Running in show mode")
        while True:
            desired_angle = calculate_mirror_angle_from_datetime(datetime.now())
            print(f"Desired angle: {desired_angle:.2f} degrees")
            time.sleep(5)

if __name__ == '__main__':
    main()
