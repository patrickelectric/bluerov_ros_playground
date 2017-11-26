# Topics and Data
- List of all topics
    - **/diagnostics** - `diagnostic_msgs/DiagnosticArray`
    - **/mavlink/from** - `mavros_msgs/Mavlink`
    - **/mavlink/to** - `mavros_msgs/Mavlink`
    - **/mavros/adsb/send** - `mavros_msgs/ADSBVehicle`
    - **/mavros/adsb/vehicle** - `mavros_msgs/ADSBVehicle`
    - **/mavros/battery** - `sensor_msgs/BatteryState`
    - **/mavros/cam_imu_sync/cam_imu_stamp** - `mavros_msgs/CamIMUStamp`
    - **/mavros/distance_sensor/rangefinder_pub** - `sensor_msgs/Range`
    - **/mavros/distance_sensor/rangefinder_sub** - `sensor_msgs/Range`
    - **/mavros/extended_state** - `mavros_msgs/ExtendedState`
    - **/mavros/fake_gps/mocap/tf** - `geometry_msgs/TransformStamped`
    - **/mavros/global_position/compass_hdg** - `std_msgs/Float64`
    - **/mavros/global_position/global** - `sensor_msgs/NavSatFix`
    - **/mavros/global_position/gp_lp_offset** - `geometry_msgs/PoseStamped`
    - **/mavros/global_position/gp_origin** - `geographic_msgs/GeoPointStamped`
    - **/mavros/global_position/home** - `mavros_msgs/HomePosition`
    - **/mavros/global_position/local** - `nav_msgs/Odometry`
    - **/mavros/global_position/raw/fix** - `sensor_msgs/NavSatFix`
    - **/mavros/global_position/raw/gps_vel** - `geometry_msgs/TwistStamped`
    - **/mavros/global_position/rel_alt** - `std_msgs/Float64`
    - **/mavros/global_position/set_gp_origin** - `geographic_msgs/GeoPointStamped`
    - **/mavros/hil/actuator_controls** - `mavros_msgs/HilActuatorControls`
    - **/mavros/hil/controls** - `mavros_msgs/HilControls`
    - **/mavros/hil/gps** - `mavros_msgs/HilGPS`
    - **/mavros/hil/imu_ned** - `mavros_msgs/HilSensor`
    - **/mavros/hil/optical_flow** - `mavros_msgs/OpticalFlowRad`
    - **/mavros/hil/rc_inputs** - `mavros_msgs/RCIn`
    - **/mavros/hil/state** - `mavros_msgs/HilStateQuaternion`
    - **/mavros/home_position/home** - `mavros_msgs/HomePosition`
    - **/mavros/home_position/set** - `mavros_msgs/HomePosition`
    - **/mavros/imu/atm_pressure** - `sensor_msgs/FluidPressure`
    - **/mavros/imu/data** - `sensor_msgs/Imu`
    - **/mavros/imu/data_raw** - `sensor_msgs/Imu`
    - **/mavros/imu/mag** - `sensor_msgs/MagneticField`
    - **/mavros/imu/temperature** - `sensor_msgs/Temperature`
    - **/mavros/local_position/odom** - `nav_msgs/Odometry`
    - **/mavros/local_position/pose** - `geometry_msgs/PoseStamped`
    - **/mavros/local_position/velocity** - `geometry_msgs/TwistStamped`
    - **/mavros/manual_control/control** - `mavros_msgs/ManualControl`
    - **/mavros/mission/reached** - `mavros_msgs/WaypointReached`
    - **/mavros/mission/waypoints** - `mavros_msgs/WaypointList`
    - **/mavros/odometry/odom** - `nav_msgs/Odometry`
    - **/mavros/radio_status** - `mavros_msgs/RadioStatus`
    - **/mavros/rangefinder/rangefinder** - `sensor_msgs/Range`
    - **/mavros/rc/in** - `mavros_msgs/RCIn`
    - **/mavros/rc/out** - `mavros_msgs/RCOut`
    - **/mavros/rc/override** - `mavros_msgs/OverrideRCIn`
    - **/mavros/setpoint_accel/accel** - `geometry_msgs/Vector3Stamped`
    - **/mavros/setpoint_attitude/cmd_vel** - `geometry_msgs/TwistStamped`
    - **/mavros/setpoint_attitude/thrust** - `mavros_msgs/Thrust`
    - **/mavros/setpoint_position/local** - `geometry_msgs/PoseStamped`
    - **/mavros/setpoint_raw/attitude** - `mavros_msgs/AttitudeTarget`
    - **/mavros/setpoint_raw/global** - `mavros_msgs/GlobalPositionTarget`
    - **/mavros/setpoint_raw/local** - `mavros_msgs/PositionTarget`
    - **/mavros/setpoint_raw/target_attitude** - `mavros_msgs/AttitudeTarget`
    - **/mavros/setpoint_raw/target_global** - `mavros_msgs/GlobalPositionTarget`
    - **/mavros/setpoint_raw/target_local** - `mavros_msgs/PositionTarget`
    - **/mavros/setpoint_velocity/cmd_vel** - `geometry_msgs/TwistStamped`
    - **/mavros/setpoint_velocity/cmd_vel_unstamped** - `geometry_msgs/Twist`
    - **/mavros/state** - `mavros_msgs/State`
    - **/mavros/time_reference** - `sensor_msgs/TimeReference`
    - **/mavros/vfr_hud** - `mavros_msgs/VFR_HUD`
    - **/mavros/wind_estimation** - `geometry_msgs/TwistStamped`
    - **/rosout** - `rosgraph_msgs/Log`
    - **/rosout_agg** - `rosgraph_msgs/Log`
    - **/tf** - `tf2_msgs/TFMessage`
    - **/tf_static** - `tf2_msgs/TFMessage`
- YAML:
```yaml
{
  "mavros": {
    "manual_control": {
      "control": {}
    },
    "hil_actuator_controls": {},
    "radio_status": {},
    "setpoint_raw": {
      "target_global": {},
      "target_attitude": {},
      "target_local": {}
    },
    "battery": {
      "power_supply_status": 2,
      "capacity": "nan",
      "power_supply_technology": 0,
      "design_capacity": "nan",
      "power_supply_health": 0,
      "current": -25.0,
      "header": {
        "stamp": {
          "secs": 1509312230,
          "nsecs": 514791438
        },
        "frame_id": "",
        "seq": 71
      },
      "charge": "nan",
      "voltage": 12.241999626159668,
      "cell_voltage": [],
      "serial_number": "",
      "percentage": 0.40999999642372131,
      "present": true,
      "location": "id0"
    },
    "altitude": {},
    "local_position": {
      "velocity": {
        "twist": {
          "linear": {
            "y": -0.051705911755561842,
            "x": 0.067819267511367784,
            "z": 0.030913775786757476
          },
          "angular": {
            "y": -0.0025659149978309874,
            "x": 0.0025783136952668428,
            "z": -0.0026175875682383771
          }
        },
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 512740420
          },
          "frame_id": "map",
          "seq": 69
        }
      },
      "pose": {
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 512740420
          },
          "frame_id": "map",
          "seq": 69
        },
        "pose": {
          "position": {
            "y": -0.048640180379152312,
            "x": 0.063255816698074216,
            "z": 0.96192848682403564
          },
          "orientation": {
            "y": -0.00015603664820251572,
            "x": -0.0025468758022611153,
            "z": -0.76219315864435044,
            "w": -0.64734463618072902
          }
        }
      }
    },
    "mission": {
      "waypoints": {}
    },
    "state": {
      "system_status": 4,
      "connected": true,
      "mode": "MANUAL",
      "header": {
        "stamp": {
          "secs": 1509312230,
          "nsecs": 271053284
        },
        "frame_id": "",
        "seq": 10
      },
      "guided": false,
      "armed": false
    },
    "time_reference": {},
    "imu": {
      "atm_pressure": {
        "fluid_pressure": 94505.56640625,
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 510414430
          },
          "frame_id": "base_link",
          "seq": 69
        },
        "variance": 0.0
      },
      "data_raw": {
        "linear_acceleration_covariance": [
          8.9999999999999985e-08,
          0.0,
          0.0,
          0.0,
          8.9999999999999985e-08,
          0.0,
          0.0,
          0.0,
          8.9999999999999985e-08
        ],
        "orientation_covariance": [
          -1.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0
        ],
        "orientation": {
          "y": 0.0,
          "x": 0.0,
          "z": 0.0,
          "w": 0.0
        },
        "angular_velocity_covariance": [
          1.2184696791468346e-07,
          0.0,
          0.0,
          0.0,
          1.2184696791468346e-07,
          0.0,
          0.0,
          0.0,
          1.2184696791468346e-07
        ],
        "angular_velocity": {
          "y": -0.0020000000000000005,
          "x": 0.002,
          "z": -0.0019999999999999996
        },
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 509825341
          },
          "frame_id": "base_link",
          "seq": 69
        },
        "linear_acceleration": {
          "y": 1.3234670151209948e-15,
          "x": 0.0,
          "z": 10.806928300000001
        }
      },
      "data": {
        "linear_acceleration_covariance": [
          8.9999999999999985e-08,
          0.0,
          0.0,
          0.0,
          8.9999999999999985e-08,
          0.0,
          0.0,
          0.0,
          8.9999999999999985e-08
        ],
        "orientation_covariance": [
          1.0,
          0.0,
          0.0,
          0.0,
          1.0,
          0.0,
          0.0,
          0.0,
          1.0
        ],
        "orientation": {
          "y": -0.00015397150311567248,
          "x": -0.0025500019842874164,
          "z": -0.76217664309413502,
          "w": -0.64736406951916303
        },
        "angular_velocity_covariance": [
          1.2184696791468346e-07,
          0.0,
          0.0,
          0.0,
          1.2184696791468346e-07,
          0.0,
          0.0,
          0.0,
          1.2184696791468346e-07
        ],
        "angular_velocity": {
          "y": -0.0025882264599204068,
          "x": 0.002464118879288435,
          "z": -0.0025810054503381248
        },
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 513459437
          },
          "frame_id": "base_link",
          "seq": 69
        },
        "linear_acceleration": {
          "y": 1.3234670151209948e-15,
          "x": 0.0,
          "z": 10.806928300000001
        }
      },
      "temperature": {
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 510414430
          },
          "frame_id": "base_link",
          "seq": 69
        },
        "temperature": 35.0,
        "variance": 0.0
      },
      "mag": {
        "magnetic_field_covariance": [
          -1.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0,
          0.0
        ],
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 509825341
          },
          "frame_id": "base_link",
          "seq": 69
        },
        "magnetic_field": {
          "y": -85999.999999999927,
          "x": 220000.0,
          "z": 557000.0
        }
      }
    },
    "global_position": {
      "raw": {
        "fix": {},
        "gps_vel": {}
      },
      "odom": {},
      "global": {},
      "local": {},
      "compass_hdg": {},
      "rel_alt": {}
    },
    "rc": {
      "in": {
        "channels": [
          1200,
          1200,
          1200,
          1200,
          1200,
          1200,
          1200,
          1200,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0
        ],
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 513286172
          },
          "frame_id": "",
          "seq": 71
        },
        "rssi": 0
      },
      "out": {
        "channels": [
          1500,
          1500,
          1500,
          1500,
          1500,
          1500,
          0,
          0
        ],
        "header": {
          "stamp": {
            "secs": 1509312230,
            "nsecs": 512909758
          },
          "frame_id": "",
          "seq": 69
        }
      }
    },
    "extended_state": {},
    "wind_estimation": {}
  },
  "mavlink": {
    "from": {
      "incompat_flags": 0,
      "magic": 254,
      "seq": 6,
      "sysid": 1,
      "compid": 1,
      "msgid": 24,
      "header": {
        "stamp": {
          "secs": 1509312228,
          "nsecs": 601764769
        },
        "frame_id": "",
        "seq": 1636
      },
      "len": 30,
      "framing_status": 1,
      "payload64": [
        30776214000,
        6406598068487847584,
        56295515033758006,
        71787139947298816
      ],
      "compat_flags": 0,
      "signature": "b''",
      "checksum": 40755
    }
  },
  "diagnostics": {
    "status": [
      {
        "message": "connected",
        "hardware_id": "udp://:14551@127.0.0.1:14552",
        "values": [
          {
            "key": "Received packets:",
            "value": "2263"
          },
          {
            "key": "Dropped packets:",
            "value": "0"
          },
          {
            "key": "Buffer overruns:",
            "value": "0"
          },
          {
            "key": "Parse errors:",
            "value": "0"
          },
          {
            "key": "Rx sequence number:",
            "value": "20"
          },
          {
            "key": "Tx sequence number:",
            "value": "0"
          },
          {
            "key": "Rx total bytes:",
            "value": "68167"
          },
          {
            "key": "Tx total bytes:",
            "value": "66100"
          },
          {
            "key": "Rx speed:",
            "value": "14184.000000"
          },
          {
            "key": "Tx speed:",
            "value": "14271.000000"
          }
        ],
        "name": "mavros: GCS bridge",
        "level": 0
      }
    ],
    "header": {
      "stamp": {
        "secs": 1509312230,
        "nsecs": 299590422
      },
      "frame_id": "",
      "seq": 20
    }
  }
}
```
