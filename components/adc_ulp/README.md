# adc_ulp

This component uses the FSM ULP on ESP32 or ESP32-S3 to read ADC and wake the CPU when a threshold is reached. It works similar to the regular `adc` component but the ULP reads at `update_interval` and only wakes the CPU if `threshold` is reached, and only then is the value sent.

Note that this component does not put the CPU to sleep, it only starts the ULP program on shutdown which wakes the CPU up. You need to use the `deep_sleep` component with the action `deep_sleep.enter` to do put the CPU to sleep. If you are using MQTT then a good place to do this is after the `mqtt` component has connected and sent the values. See the example yaml below how to do that.

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

deep_sleep:

mqtt:
  broker: 192.168.0.22
  username: !secret mqtt_user
  password: !secret mqtt_pass
  discovery: true
  discovery_unique_id_generator: mac
  # https://community.home-assistant.io/t/how-to-get-last-known-value-while-sensor-is-in-deep-sleep/119665/18
  birth_message:
  will_message:
  on_connect: 
    then:
      - delay: 
          1000ms
      - mqtt.disable
      - delay:
          1000ms   
      - deep_sleep.enter

external_components:
  - source: github://jonaskello/esphome-external-components@main

sensor:
  - platform: adc_ulp
    pin: GPIO6
    threshold: 0.2
    update_interval: 100ms
    name: "Moisture"
    attenuation: 12db
    qos: 1
```
