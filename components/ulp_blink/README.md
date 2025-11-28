# example configuration:

esp32:
  board: esp32dev
  framework:
    type: esp-idf
    sdkconfig_options:
      # NOTE: Without this option it will not compile
      CONFIG_ULP_COPROC_ENABLED: "y"


external_components:
  - source: github://jonaskello/esphome-external-components@main
    components: [ ulp_blink ]

ulp_blink:
  pin: GPIO2
  interval: 1000ms

# Use deep sleep to prove that ulp runs even when in deep sleep
# The led will blink very faintly and quickly in sleep mode, watch closely to see it
deep_sleep:
  run_duration: 10s
  sleep_duration: 10s
