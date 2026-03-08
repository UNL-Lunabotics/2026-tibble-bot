# 2026-tibble-bot

## Motor Info
Drivetrain (Kraken x60 powered by TalonFX)
- https://store.ctr-electronics.com/products/kraken-x60
- Two of them
- CAN via TalonFX
- CANh HIGH on yellow, CANh LOW on green

Excavation Motor (Yellow Jacket)
- https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-50-9-1-ratio-24mm-length-8mm-rex-shaft-117-rpm-3-3-5v-encoder/
- 50:9:1 ratio
- 24mm length 8mm REX shaft
- 117 rpm at max
- will go at 100 rpm when excavating

Vibration Motor (NFP Motor)
- https://nfpmotor.com/775-performance-motor-12v-24v-dc-double-heads-electric-vibrating-motor
- No encoders
- 44-60mm
- 8,000 rpm max
- 4,000 rpm during excavation and dumping
- CW and CWW rotation

Hopper Actuators (Super Duty)
- https://www.firgelliauto.com/products/super-duty-actuators?variant=39956754890823
- Two of them
- 450 lb
- 10 inch stroke
- encoders built in
- Sync control board = Teensy

Hopper Servo (GoBuilda 2000 Dual Mode)
- https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque/ 

2 roboclaws, the excav motor, 

## State Machine Design

The linear actuators will have their own states, REST, EXCAV, and DUMP to correspond to the encoder positions for where it should be.

For every state, it needs to clarify what drivetrain should be doing, what the vibration motor should be doing, what the excavation motor should be doing, where the actuators should be, and if the hopper servo should be latched or not

Using just a bullet point means the order does not matter, using a numbered list means the actions must happen sequentially

IDLE:
The robot is just sitting there not doing anything, but is turned on and could do something.

- drivetrain off
- vibe off
- hop servo latched
- Linear Actuator's (LA's) at REST position

TRAVELING:
The robot is not digging or excavating but is traveling around the arena

- drivetrain on
- vibe off
- hop servo latched
- LA's at REST

EXCAVATING:
The robot is moving forward slowly with the hopper tilted forward, paddles on, vibe on, to dig up the sand and store it

- drivetrain on the whole time slowly going forward
- hopper latched
1. start vibration motor
2. retract LA's to EXCAV
3. start paddles rotating clockwise

DUMPING:
The robot is moving forward slowly with the opper tilted backwards to slowly dump in the berm

- drivetrain optionally off or moving forward slowly
1. unlatch hopper
2. extend LA's to DUMP
3. start the vibration motor

## Procedure for Fixing URDFs
1. It uses the wrong import mesh thing, fix that
2. Need to manually add a base link
3. Xacro cleanup stuff
4. It gets the axes wrong every time