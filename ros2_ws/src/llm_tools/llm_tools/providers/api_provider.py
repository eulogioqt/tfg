"""TODO: Add module documentation."""
from .base_provider import BaseProvider


class APIProvider(BaseProvider):

"""TODO: Describe class."""
    def load(self, *args, **kwargs):
    """TODO: Describe load.
Args:
    *args (:obj:`Any`): TODO.
    **kwargs (:obj:`Any`): TODO.
"""
        print(f"[INFO] Skipping load: API-based provider, nothing to load.")

    def unload(self, *args, **kwargs):
    """TODO: Describe unload.
Args:
    *args (:obj:`Any`): TODO.
    **kwargs (:obj:`Any`): TODO.
"""
        print(f"[INFO] Skipping unload: API-based provider, nothing to unload.")
