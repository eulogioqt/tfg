"""TODO: Add module documentation."""
from .service_engine import ServiceEngine

from dotenv import load_dotenv

from llm_msgs.srv import Embedding


class EmbeddingEngine(ServiceEngine):
"""TODO: Describe class."""
    def __init__(self, node):
    """TODO: Describe __init__.
Args:
    node (:obj:`Any`): TODO.
"""
        super().__init__(node)

        load_dotenv()

        self.embeddings_cli = self.create_client(Embedding, "llm_tools/embedding")

        self.node.get_logger().info("Embedding Engine initializated succesfully")
    
    def embedding_request(self, provider="", model="", user_input=""):
    """TODO: Describe embedding_request.
Args:
    provider (:obj:`Any`): TODO.
    model (:obj:`Any`): TODO.
    user_input (:obj:`Any`): TODO.
"""
        req = Embedding.Request()
        req.provider = provider
        req.model = model
        req.user_input = user_input

        result = self.call_service(self.embeddings_cli, req)

        return result.embedding, result.provider_used, result.model_used, result.message, result.success
