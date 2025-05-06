import requests

def translate(texto, idioma_origen="es", idioma_destino="en"):
    url = "https://translate.googleapis.com/translate_a/single?client=gtx&sl={}&tl={}&dt=t&q={}".format(idioma_origen, idioma_destino, texto)
    response = requests.get(url)
    traduccion = response.json()[0][0][0]
    
    return traduccion