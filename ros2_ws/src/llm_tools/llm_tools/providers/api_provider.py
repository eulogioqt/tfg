from .base_provider import BaseProvider

class APIProvider(BaseProvider):

    def load(self, models=None):
        print(f"[INFO] Skipping load: API-based provider, nothing to load.")

    def unload(self, models=None):
        print(f"[INFO] Skipping unload: API-based provider, nothing to unload.")
