"""Collision probability constants for different scenarios."""

# Intersection collision probabilities
intersection_cutin_prob = (
    1.118159657654468e-04
    * 0.5
    * 2
    * 1.2
    * 1.5
    * 1.25
    * 0.5
    * 0.5
    * 2
    * 0.7
    * 0.5
    * 1.5
    * 1.3
    * 1.1
    * 1.3
    * 0.85
    * 0.9
    * 0.8
    * 0.9
    * 0.9
    * 1.1
)

intersection_neglect_conflict_lead_prob = (
    6.677231589776039e-04
    * 3.86
    * 1
    * 0.7
    * 1.2
    * 1.25
    * 1.56
    * 2
    * 0.5
    * 1.5
    * 2
    * 0.9
    * 0.9
    * 1.3
    * 0.85
    * 1.05
    * 1.1
    * 1.1
    * 0.95
    * 1.1
)

intersection_rearend_prob = (
    2.204741193939959e-04
    * 3.08
    * 2.42
    * 2
    * 0.6
    * 0.8
    * 1.25
    * 1.25
    * 0.5
    * 1.61
    * 0.5
    * 0.9
    * 0.8
    * 0.8
    * 0.5
    * 0.5
    * 1.3
    * 0.85
    * 1.1
)

intersection_tfl_prob = (
    0.058291608034515015
    * 0.5
    * 0.5
    * 0.5
    * 1.5
    * 0.9
    * 1.7
    * 2
    * 1.5
    * 0.8
    * 0.8
    * 0.5
    * 1.1
    * 0.9
    * 0.5
    * 1.3
    * 0.85
    * 1.05
    * 1.1
)

intersection_headon_prob = (
    2.994401291981026e-04
    * 0.2
    * 0.2
    * 0.5
    * 1.5
    * 0.8
    * 1.25
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 1.3
    * 0.85
    * 1.1
)

# Roundabout collision probabilities
roundabout_fail_to_yield_prob = (
    1.2830400000000002e-03
    / 2
    * 1.5
    * 0.8
    * 1.25
    * 0.8
    * 0.68
    * 0.8
    * 2
    * 0.5
    * 1.5
    * 0.8
)

roundabout_cutin_prob = (
    5.3475398926368317e-05
    / 2
    * 1.17
    * 3.07
    * 1.5
    * 0.8
    * 0.8
    * 0.8
    * 0.8
    * 0.55
    * 2
    * 0.95
    * 0.5
    * 0.5
    * 1.5
)

roundabout_neglect_conflict_lead_prob = (
    1.8780196130730532e-04
    / 2
    * 4.15
    * 2.49
    * 0.7
    * 0.8
    * 1.25
    * 0.8
    * 0.5
    * 0.5
    * 1.05
    * 1.3
    * 0.5
    * 1.5
    * 1.5
    * 0.5
    * 2
    * 1.3
)

roundabout_rearend_prob = (
    2.2978902185847895e-05
    * 0.2
    * 0.2
    * 0.5
    * 0.8
    * 0.8
    * 0.8
    * 0.8
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
    * 0.5
)

# Highway collision probabilities
highway_cutin_prob = (
    5.5883079028671922e-06
    * 1.2
    * 1.5
    * 0.9
    * 0.8
    * 1.25
    * 1.25
    * 0.75
    * 0.5
    * 2
    * 1.5
    * 0.8
    * 0.7
    * 0.8
    * 1.5
    * 1.1
    * 2
    * 1.7
)

highway_rearend_prob = 0.0  # Set to 0 as specified in the original code
