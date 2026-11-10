"""Reuse the same geometry screen with the whole-course motion envelope."""
from ..togt_course import layout as original
from . import scenario

foreign_diagnostics=original.foreign_diagnostics


def _call(name,*args):
    previous=original.scenario
    original.scenario=scenario
    try:return getattr(original,name)(*args)
    finally:original.scenario=previous


def audit():return _call('audit')
def motion_audit():return _call('motion_audit')
def figure(out,geometry):return _call('figure',out,geometry)
