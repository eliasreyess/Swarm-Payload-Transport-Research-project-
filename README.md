# Swarm-Payload-Transport-Research-project-
![GUI](model_ss.png)

This project presents an agent-based model that investigates collective payload transport in a toroidal arena. Each Agent follows a Vicsek alignment rule augmented by weak chemotactic signals emitted by payload and target. Motility-induced phase separation emerges as the driving mechanism.
The model is designed as a 20 × 20 patch
arena with top and bottom edges wrapped
such that an agent leaves through the top
and renters immediately the bottom.
This configuration was chosen due to the
similarity of physical experiments done with
Janus colloids in toroidal enclosures with the
distinction that this model expresses a
surface instead of a volume .
Agents are initially placed at random and
respond to user defined inputs (interaction
radius, speed, and noise, etc).

Each run assigns obstacles, the payload, and
the target locations. The payload and target
emit a weak chemotactic signal that slightly
biases agent headings toward them, but
motion is overall dominated by simple
Vicsek alignment.

To explore parameter space, a coarse sweep
of approximately 435,000 simulations was
ran in BehaviorSpace to identify effective
parameter sets. Then a narrower sweep of
1,000 simulations was performed focused on
the most promising region to isolate the best
parameter combinations.

Comparing runs with active versus suppressed MIPS suggests that motility
induced phase separation plays a critical role in successful payload delivery.
Vicsek alignment by itself can deliver the payload, but it does so by a
stochastic clustering. When MIPS works together with weak chemotactic signals
the agents form polarized flocs almost immediately, yielding higher success
rates and shorter completion times.
Moderate noise was observed to be necessary for Vicsek alignment and MIPS,
yet at the same time noise had critical boundaries which once surpassed
generated instability and did not allow for flocking, resembling an isotropic
gas. The effective set of parameters contains a narrow, noise dependent
“sweet-spot” where Vicsek alignment, MIPS and chemotaxis act effectively
