# Single original-size moving gate, mechanism study

Use original W1 rectangular aperture and x500. Start remains (-16,4,3.2); goal is 6 m
past the fixed gate plane. Only W1 exists in the Gazebo world. The plane orientation
is fixed; its center oscillates horizontally inside the plane. Both variants share
the same future motion, gate-crossing time optimization, free crossing V/A/J,
soft safety/dynamics costs, L-BFGS-B and 1 s x500 CTBR tracking MPC. Only the
two crossing-position coordinates are fixed to zero for center, free for ours.

Frozen pilot cases: static f=1 Hz,A=0 m; f=1 Hz,A=2 m; f=2 Hz,A=2 m;
global phase zero once per pair. Run both variants in each case. No adaptive
selection of cases or tuning after outcome. Each flight has a 40 s post-takeoff
limit. Completion requires one forward crossing, full-body exit, reaching the
forward goal and no physical frame contact. Collision, reverse crossing, timeout,
controller error and missing truth are recorded separately. Times from start to
forward goal; no return in this isolated mechanism experiment.

True 4 ms body-center data provide crossing time/point, entry-to-exit minimum
sphere-proxy clearance, actual relative lateral velocity and tracking position
error. The proxy does not replace Gazebo contacts as the physical collision verdict.
The main seven-gate benchmark retains the full return metric separately.
