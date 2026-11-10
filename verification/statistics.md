# FLASK-RRTConnect first / final / budget measurements

Seeds: [20]; repeats per seed: 1; total planning budget: 1.0 s.
J = rho*T + integral(w^T R w dt); execution duration is not planning time.
Means +/- sample SD are over successful runs only. Before a first solution, cost and length are missing.
First metrics belong to the saved raw RRTConnect path; final metrics belong to the shortcut path.
Search and shortcut share the budget. Export, quadrature and native checks are outside the measured planning clock.

## First solution

| Environment | Success | First time (ms) | First J | First length (m) | First execution duration (s) |
| --- | --- | --- | --- | --- | --- |
| uni_bugtrap | 0/1 | NA | NA | NA | NA |
| uni_wall | 1/1 | 100.332 +/- NA | 44.939 +/- NA | 25.528 +/- NA | 34.778 +/- NA |
| quad2d_bugtrap | 1/1 | 3.746 +/- NA | 38.371 +/- NA | 16.411 +/- NA | 22.631 +/- NA |
| quad2d_hole | 0/1 | NA | NA | NA | NA |
| quad3d_block | 1/1 | 0.476 +/- NA | 13.254 +/- NA | 9.423 +/- NA | 9.164 +/- NA |
| quad3d_window | 1/1 | 4.167 +/- NA | 26.400 +/- NA | 11.141 +/- NA | 16.761 +/- NA |

## Final solution

| Environment | Success | Final J | Final length (m) | Execution duration (s) | Planning incl. shortcut (ms) | Last improvement (ms) |
| --- | --- | --- | --- | --- | --- | --- |
| uni_bugtrap | 0/1 | NA | NA | NA | NA | NA |
| uni_wall | 1/1 | 17.643 +/- NA | 9.880 +/- NA | 13.419 +/- NA | 105.119 +/- NA | 104.757 +/- NA |
| quad2d_bugtrap | 1/1 | 26.295 +/- NA | 14.780 +/- NA | 18.685 +/- NA | 4.424 +/- NA | 4.323 +/- NA |
| quad2d_hole | 0/1 | NA | NA | NA | NA | NA |
| quad3d_block | 1/1 | 12.711 +/- NA | 9.278 +/- NA | 9.352 +/- NA | 0.688 +/- NA | 0.686 +/- NA |
| quad3d_window | 1/1 | 9.737 +/- NA | 5.764 +/- NA | 7.100 +/- NA | 4.370 +/- NA | 4.369 +/- NA |

## Checkpoints

| Environment | Budget (s) | Success | Mean J (available runs) | Mean length (m) |
| --- | --- | --- | --- | --- |
| dynobench_uni_bugtrap | 0.001 | 0/1 | NA | NA |
| dynobench_uni_bugtrap | 0.005 | 0/1 | NA | NA |
| dynobench_uni_bugtrap | 0.01 | 0/1 | NA | NA |
| dynobench_uni_bugtrap | 0.05 | 0/1 | NA | NA |
| dynobench_uni_bugtrap | 0.1 | 0/1 | NA | NA |
| dynobench_uni_bugtrap | 0.5 | 0/1 | NA | NA |
| dynobench_uni_bugtrap | 1 | 0/1 | NA | NA |
| dynobench_uni_wall | 0.001 | 0/1 | NA | NA |
| dynobench_uni_wall | 0.005 | 0/1 | NA | NA |
| dynobench_uni_wall | 0.01 | 0/1 | NA | NA |
| dynobench_uni_wall | 0.05 | 0/1 | NA | NA |
| dynobench_uni_wall | 0.1 | 0/1 | NA | NA |
| dynobench_uni_wall | 0.5 | 1/1 | 17.6429 | 9.88008 |
| dynobench_uni_wall | 1 | 1/1 | 17.6429 | 9.88008 |
| dynobench_quad2d_bugtrap | 0.001 | 0/1 | NA | NA |
| dynobench_quad2d_bugtrap | 0.005 | 1/1 | 26.2945 | 14.7797 |
| dynobench_quad2d_bugtrap | 0.01 | 1/1 | 26.2945 | 14.7797 |
| dynobench_quad2d_bugtrap | 0.05 | 1/1 | 26.2945 | 14.7797 |
| dynobench_quad2d_bugtrap | 0.1 | 1/1 | 26.2945 | 14.7797 |
| dynobench_quad2d_bugtrap | 0.5 | 1/1 | 26.2945 | 14.7797 |
| dynobench_quad2d_bugtrap | 1 | 1/1 | 26.2945 | 14.7797 |
| dynobench_quad2d_hole | 0.001 | 0/1 | NA | NA |
| dynobench_quad2d_hole | 0.005 | 0/1 | NA | NA |
| dynobench_quad2d_hole | 0.01 | 0/1 | NA | NA |
| dynobench_quad2d_hole | 0.05 | 0/1 | NA | NA |
| dynobench_quad2d_hole | 0.1 | 0/1 | NA | NA |
| dynobench_quad2d_hole | 0.5 | 0/1 | NA | NA |
| dynobench_quad2d_hole | 1 | 0/1 | NA | NA |
| dynobench_quad3d_block | 0.001 | 1/1 | 12.7111 | 9.2781 |
| dynobench_quad3d_block | 0.005 | 1/1 | 12.7111 | 9.2781 |
| dynobench_quad3d_block | 0.01 | 1/1 | 12.7111 | 9.2781 |
| dynobench_quad3d_block | 0.05 | 1/1 | 12.7111 | 9.2781 |
| dynobench_quad3d_block | 0.1 | 1/1 | 12.7111 | 9.2781 |
| dynobench_quad3d_block | 0.5 | 1/1 | 12.7111 | 9.2781 |
| dynobench_quad3d_block | 1 | 1/1 | 12.7111 | 9.2781 |
| dynobench_quad3d_window | 0.001 | 0/1 | NA | NA |
| dynobench_quad3d_window | 0.005 | 1/1 | 9.7369 | 5.76362 |
| dynobench_quad3d_window | 0.01 | 1/1 | 9.7369 | 5.76362 |
| dynobench_quad3d_window | 0.05 | 1/1 | 9.7369 | 5.76362 |
| dynobench_quad3d_window | 0.1 | 1/1 | 9.7369 | 5.76362 |
| dynobench_quad3d_window | 0.5 | 1/1 | 9.7369 | 5.76362 |
| dynobench_quad3d_window | 1 | 1/1 | 9.7369 | 5.76362 |

A changing available-run cohort can change the conditional mean; it is not an unconditional optimality claim.
RRTConnect is not restarted after termination. Later checkpoints hold its final incumbent.
Paper dynamics are retained; model_differences.json records differences from original DynoBench models.
Reduced-r2 success is separate from native dynamics certification. See native_validation.json.

- dynobench_uni_bugtrap: {'timeout': 1}; failed planning time: 1000.014 +/- NA ms
- dynobench_quad2d_hole: {'timeout': 1}; failed planning time: 1000.036 +/- NA ms
