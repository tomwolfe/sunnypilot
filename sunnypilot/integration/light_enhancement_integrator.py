"""
Light Enhancement Integrator Module
This module provides the integration point for lightweight enhancements,
as expected by the test files.
"""

from sunnypilot.lightweight.integration import LightweightIntegrator, create_light_integrator


class LightEnhancementIntegrator(LightweightIntegrator):
    """
    Wrapper class for the lightweight integrator to maintain API compatibility
    with the expected interface in tests.
    """
    pass


def create_light_enhancement_integrator():
    """
    Factory function to create the light enhancement integrator.
    This is a wrapper that delegates to the actual implementation
    in the lightweight module to maintain compatibility with tests.
    """
    return create_light_integrator()