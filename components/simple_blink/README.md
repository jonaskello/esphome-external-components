# example configuration:

external_components:
  - source: github://jonaskello/esphome-external-components@main
    components: [ simple_blink ]

simple_blink:
  pin: GPIO2
  interval: 200ms
