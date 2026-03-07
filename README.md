# Game AI Project

I made flocking system in Unreal Engine. To turn on or off, spatial partitioning, the user must toggle the #define in code in Flock.h.

An addiitonal agent is spawned in the middle. User controlls it by clicking in the field to make this agent seek the click position.
Flock evades this agent at a constant radius (can be tweaked in code);

Boid behaviour features following behaviours:
- Cohesion (move as a group towards a common target)
- Separation (keep a certain distance from neighbors to avoid collision)
- Velocity Match (move at a relatively same speed as neighbors)
- Seek (simply go towards the target)
- Wander (wander around, go where-ever may please)
- Evade (evade the agent controlled by user)

Flock behaviour weigths can be tweaked using imgui

IMPORTANT - MAKE SURE TO OPEN FLOCKING LEVEL TO SEE FLOCKING IMPLEMENTATION
