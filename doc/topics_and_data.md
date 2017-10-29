# Topics and Data
- List of all topics
    ```py
    ['mavros']['battery']
    ['mavros']['global_position']['global']
    ['mavros']['global_position']['raw']['fix']
    ['mavros']['imu']['atm_pressure']
    ['mavros']['imu']['data']
    ['mavros']['imu']['data_raw']
    ['mavros']['imu']['mag']
    ['mavros']['imu']['temperature']
    ['mavros']['time_reference']
    ['mavlink']['from']
    ['mavros']['altitude']
    ['mavros']['extended_state']
    ['mavros']['global_position']['compass_hdg']
    ['mavros']['global_position']['rel_alt']
    ['mavros']['hil_actuator_controls']
    ['mavros']['manual_control']['control']
    ['mavros']['mission']['waypoints']
    ['mavros']['radio_status']
    ['mavros']['rc']['in']
    ['mavros']['rc']['out']
    ['mavros']['setpoint_raw']['target_attitude']
    ['mavros']['setpoint_raw']['target_global']
    ['mavros']['setpoint_raw']['target_local']
    ['mavros']['state']
    ['mavros']['local_position']['pose']
    ['mavros']['wind_estimation']
    ['mavros']['local_position']['velocity']
    ['mavros']['global_position']['local']
    ['mavros']['global_position']['odom']
    ['mavros']['global_position']['raw']['gps_vel']
    ['diagnostics']
    ```
- YAML of what is possible to get:
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