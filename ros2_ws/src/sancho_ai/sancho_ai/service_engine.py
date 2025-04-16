import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread, Event
from llm_msgs.srv import Prompt
from hri_msgs.srv import GetString


class ServiceEngine:
    def __init__(self):
        self.node = rclpy.create_node("service_engine_client")

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.thread = Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

        self.prompt_cli = self._create_client(Prompt, "llm_tools/prompt"),
        self.actual_people_cli = self._create_client(GetString, "logic/get/actual_people"),

        self.node.get_logger().info("ServiceEngine inicializado correctamente")

    def prompt_request(self, provider="openai", model="gpt-3.5-turbo", prompt_system="", messages_json="", user_input="", parameters_json=""):
        req = Prompt.Request()
        req.provider = provider
        req.model = model
        req.prompt_system = prompt_system
        req.messages_json = messages_json
        req.user_input = user_input
        req.parameters_json = parameters_json

        result = self._call_service(self.prompt_cli, req)

        return result.response

    def get_actual_people_request(self):
        req = GetString.Request()

        result = self._call_service(self.actual_people_cli, req)

        return result.text

    def _create_client(self, srv_type, srv_name):
        client = self.node.create_client(srv_type, srv_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(f'{srv_name} service not available, waiting again...')

        return client

    def _call_service(self, client, request):
        done_event = Event()
        result_holder = {}

        def done_cb(fut):
            try:
                result_holder["value"] = fut.result()
            except Exception as e:
                self._log(f"Error al recibir respuesta: {e}", level="error")
                result_holder["value"] = None
            finally:
                done_event.set()

        future = client.call_async(request)
        future.add_done_callback(done_cb)
        done_event.wait()

        return result_holder["value"]
