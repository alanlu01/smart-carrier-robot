"""Intermittent detections must survive without summing people across frames."""

import math

import pytest

from smart_delivery_core.optional_idle import DirectionalPeople, map_alignment_correction
from test_optional_idle import configuration


def observed(positive_indices=(), positive=1):
    config = configuration()['people']
    people = DirectionalPeople(config)
    # A fixed east bearing intermittently visible across the camera sweep.
    stamp = 0
    for heading_bin in range(24):
        for frame in range(10):
            heading = math.radians(heading_bin*15+7.5)
            angle = math.degrees(math.atan2(math.sin(heading-math.radians(7.5)),
                                            math.cos(heading-math.radians(7.5))))
            detections = ([{'class': 'Person', 'angle': 0., 'score': .9}]*positive
                          if abs(angle) <= 34 and frame in positive_indices else [])
            for target in detections:
                target['angle'] = angle
            stamp += 1
            people.observe(detections, heading, stamp)
    return people, config


def test_three_intermittent_positives_choose_supported_direction_even_with_zero_median():
    people, config = observed((1, 4, 7))
    assert people.select(config['observation_pose'])['name'] == 'E'
    report = people.summary(config['observation_pose'])
    assert report['qualified_bins'] >= 1 and report['person_frames'] >= 3
    assert report['reason'] == 'direction_selected'


@pytest.mark.parametrize('positive_count', [0, 1, 2])
def test_empty_or_one_two_frame_false_positives_never_choose_a_point(positive_count):
    people, config = observed(())
    people.frames[0] = [1]*positive_count+[0]*(10-positive_count)
    assert people.select(config['observation_pose']) is None


def test_low_positive_fraction_is_not_promoted_by_neighbor_smoothing():
    people, config = observed((1, 4, 7))
    people.frames[0] = [1]*3+[0]*57
    assert people.select(config['observation_pose']) is None


def test_repeated_frames_do_not_meet_three_positive_requirement():
    people, config = observed(())
    for _ in range(20):
        people.observe([{'class': 'Person', 'angle': 0., 'score': .99}], 0., 241)
    assert people.person_frames == 1
    assert people.select(config['observation_pose']) is None


def test_more_people_in_same_frames_increases_score_not_number_of_sightings():
    one, config = observed((1, 4, 7), 1)
    two, _ = observed((1, 4, 7), 2)
    a = one.summary(config['observation_pose'])['sector_scores']['E']
    b = two.summary(config['observation_pose'])['sector_scores']['E']
    assert b == pytest.approx(a*2, abs=1e-5)
    assert b < 2  # not six sightings reported as six unique people


def test_diagnostic_distinguishes_no_people_evidence_and_partial_coverage():
    people, config = observed(())
    assert people.summary(config['observation_pose'])['reason'] == 'no_person_detected'
    people, config = observed((1,))
    assert people.summary(config['observation_pose'])['reason'] == 'insufficient_positive_evidence'
    people.headings = {0}
    assert people.summary(config['observation_pose'])['reason'] == 'insufficient_heading_coverage'


def test_map_origin_rotation_does_not_invent_vehicle_translation_far_from_origin():
    angle = .01
    current = (20.-20.*math.cos(angle), -20.*math.sin(angle), angle)
    assert math.hypot(*current[:2]) > .1
    distance, turn = map_alignment_correction((0.,0.,0.), current, 20., 0.)
    assert distance == pytest.approx(0.) and turn == pytest.approx(.01)
    shifted = (current[0]+.15, current[1], current[2])
    assert map_alignment_correction((0.,0.,0.), shifted, 20., 0.)[0] == pytest.approx(.15)
