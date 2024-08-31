# AfterAI Flaps AVR Chip

AVR chip is on both Leader PCBs and Follower PCBs.

We developed this software based on `unit` directory of [davidkingsman/split-flap](https://github.com/davidkingsman/split-flap). Here are some of the major changes we have made:

- Store offset in AVR chip
- Expanded the I2C address space for AVR chips
  - More AVR chips means more current, so be careful if you connect more than 16 units.
