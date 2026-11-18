"""Geometry reuse is exact, bounded, defensive and scoped to one evaluation."""
from types import SimpleNamespace as NS

import pytest
from smart_delivery_core.station_geometry import StationGrid, geometry_cycle


def grid():
    return StationGrid(NS(info=NS(resolution=.05, width=40, height=40,
        origin=NS(position=NS(x=-1., y=-1.), orientation=NS(x=0., y=0., z=0., w=1.))),
        data=[0]*1600))


def test_obstacle_and_new_grid_are_never_hidden_by_a_previous_cycle():
    g = grid()
    with g.evaluation():
        assert g.clear((0., 0., 0.)) and g.clear((0., 0., 0.))
        assert g.geometry_cache_hits == 1
    assert g._blocker_cache is None
    g.data[20*40+20] = 100
    with g.evaluation():
        assert not g.clear((0., 0., 0.))
        blocker = g.blocker((0., 0., 0.))
        blocker['position'][0] = 999.
        assert g.blocker((0., 0., 0.))['position'][0] != 999.
    with grid().evaluation():
        assert not g.clear((0., 0., 0.))


def test_cache_never_quantizes_position_or_grows_without_bound():
    g = grid()
    with g.evaluation():
        for i in range(600):
            g.clear((i/1_000_000., 0., 0.))
        assert g.geometry_cache_hits == 0 and len(g._blocker_cache) == 512


def test_decorator_cleans_up_after_exception_and_rechecks_replaced_snapshot():
    g = grid()
    class Owner:
        grid = g
        static_grid = None

        @geometry_cycle
        def evaluate(self, fail=False):
            assert self.grid.clear((0., 0., 0.))
            assert self.grid.clear((0., 0., 0.))
            if fail:
                raise ValueError('cancel evaluation')
    owner = Owner()
    with pytest.raises(ValueError):
        owner.evaluate(True)
    assert g._blocker_cache is None
    owner.grid = grid()
    owner.evaluate()
    assert owner.grid.geometry_cache_hits == 1
