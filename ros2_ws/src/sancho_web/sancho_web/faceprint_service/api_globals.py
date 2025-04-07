api_node = None

def get_api_node():
    global api_node
    return api_node

def set_api_node(n):
    global api_node
    api_node = n