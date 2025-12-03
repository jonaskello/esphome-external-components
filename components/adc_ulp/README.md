# example configuration:

```yaml
esphome:
  name: mysensor
  friendly_name: mysensor

esp32:
  board: esp32-s3-devkitc-1
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_ULP_COPROC_ENABLED: "y"
      CONFIG_ULP_COPROC_TYPE_FSM: "y"
      CONFIG_ULP_COPROC_RESERVE_MEM: "1024"

logger:
  level: VERBOSE

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  output_power: 11dB
  fast_connect: true
  manual_ip:
    static_ip: 192.168.0.31
    gateway: 192.168.0.1
    subnet: 255.255.255.0

mqtt:
  broker: 192.168.0.22
  username: !secret mqtt_user
  password: !secret mqtt_pass
  discovery: true
  # https://community.home-assistant.io/t/how-to-get-last-known-value-while-sensor-is-in-deep-sleep/119665/18
  birth_message:
  will_message:

external_components:
  - source: github://jonaskello/esphome-external-components@main

sensor:
  - platform: adc_ulp
    pin: GPIO6
    threshold: 0.2
    update_interval: 100ms
    name: "Moisture Sensor #3"
    attenuation: 12db
    qos: 1
```
