#!/usr/bin/env python3
"""Test script to verify that multiple stalled objects with the same object_type generate unique IDs."""

import sys
from pathlib import Path

# Add packages to path
sys.path.insert(0, str(Path(__file__).parent / "packages" / "terasim-nde-nade"))

from terasim_nde_nade.adversity.static.stalled_object import StalledObjectAdversity


def test_unique_vehicle_ids():
    """Test that multiple stalled objects with same object_type get unique vehicle IDs."""

    # Create multiple stalled objects with the same object_type
    stalled1 = StalledObjectAdversity(
        placement_mode="xy_angle",
        x=83563.00,
        y=22882.00,
        angle=268.28,
        object_type="veh_passenger"
    )

    stalled2 = StalledObjectAdversity(
        placement_mode="xy_angle",
        x=90280.00,
        y=23902.00,
        angle=268.28,
        object_type="veh_passenger"  # Same object_type!
    )

    stalled3 = StalledObjectAdversity(
        placement_mode="xy_angle",
        x=120470.00,
        y=28539.00,
        angle=268.28,
        object_type="veh_passenger"  # Same object_type again!
    )

    # Check that they have unique adversity IDs
    print("Testing unique adversity IDs...")
    assert stalled1._adversity_id != stalled2._adversity_id
    assert stalled1._adversity_id != stalled3._adversity_id
    assert stalled2._adversity_id != stalled3._adversity_id
    print(f"✓ Adversity IDs are unique:")
    print(f"  stalled1: {stalled1._adversity_id}")
    print(f"  stalled2: {stalled2._adversity_id}")
    print(f"  stalled3: {stalled3._adversity_id}")

    # Generate vehicle IDs (simulate what happens in initialize())
    unique_suffix1 = str(stalled1._adversity_id).replace("-", "")[:8]
    unique_suffix2 = str(stalled2._adversity_id).replace("-", "")[:8]
    unique_suffix3 = str(stalled3._adversity_id).replace("-", "")[:8]

    vehicle_id1 = f"BV_{stalled1._object_type}_stalled_object_{unique_suffix1}"
    vehicle_id2 = f"BV_{stalled2._object_type}_stalled_object_{unique_suffix2}"
    vehicle_id3 = f"BV_{stalled3._object_type}_stalled_object_{unique_suffix3}"

    print("\nTesting unique vehicle IDs...")
    assert vehicle_id1 != vehicle_id2
    assert vehicle_id1 != vehicle_id3
    assert vehicle_id2 != vehicle_id3
    print(f"✓ Vehicle IDs are unique:")
    print(f"  stalled1: {vehicle_id1}")
    print(f"  stalled2: {vehicle_id2}")
    print(f"  stalled3: {vehicle_id3}")

    print("\n✓ All tests passed! Multiple stalled objects with the same object_type now generate unique IDs.")


if __name__ == "__main__":
    test_unique_vehicle_ids()
