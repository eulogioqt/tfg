# meter el pauqete de video para todo el pipeline de detectar y reconocer etc (en el paper referenciar al otro paper)
# una vez este el pipeline (a parte de simplifcar todo quitar cosas de movimiento y demas, y lo de postear la imagen de
# deteccion y reconocimiento de otra forma) pues hacer el intent este para que si le preguntas que a quien esta viendo
# pues te diga a quien esta viendo
# ver si es mejor que publique actualpeople todo el rato o un service para gettearlo (en el caso 2 pues hacer una clase getteablenode o algo asi)
# lo de la base de datos intentar hacer una mongodb con endpoints y demas o algo asi que sea interesante, en vez
# de pasar toda la infor por websocket y ya. Quiza por websocket mandar señales o la info nueva en tiempo real,
# pero por detras toda la api con mongodb y fastapi o algo asi
# en la visualizacion de la bd, si ers admin puedes borrar y editar y si eres user solo ver
# HABLAR CON CHATGPT LA MEJOR FORMA DE HACERLO, SI TODO POR WEBSOCKET, SI WEBSOCKET Y FASTAPI, SI MONGODB ACCESIBLE, SI SQLITE

# comandos: hacerlos todos relacionados con el robot, las demas respuestas, entrenar con un buen prompt y un rag las respuestas
# para el ejemplo este finde, borrar usuario, quien ves en la pantalla, 

# hacer en la web una parte para visualizar los topics todos en general. Hacer un tipo de mensaje "INFO" en el que se
# pueda pasar info tipo se ha subscrito ahora a este nodo (y salga un toast o algo asi, COGER EL DE LA WIKI), ahora ha pasao esto
# ahora lo otro... que al conectarse pase todos los de estos a los que esta subscrito para en un apartado de la
# web que sea ROS o algo asi puedas ver los topics que esta subscrito el tipo de mensaje, el formato en forma de diccionario del mensaje
# algo asi guay

# despues de hacer estas cosas para el paper, parar, hacer refactor del ros2web y dejarlo perfecto con la docu y todo
# luego hacer lo del video que funcione como el de youube se pueda cerrar y poner en grande y demas la interfaz ponerla perita
# y ya seguir... el tema de mejorar la interaccion la velocidad los modelos de voz y deteccion y todo eso
# luego ver tambien la comunicacion con csar que era un follon
# los logs en la web de todo lo que ha hecho x usuario
# firebase pa iniciar sesion y eso... reutilizar de la wiki
# lo de poner la cabeza tambien en la interfaz y que sepas si esta hablando o no, los ojos la boca....
# pensar bien como organizar la interfaz web. Sobre lo de que usen varios la interfaz a la vez,
# pues seria como si en la vida real hubiese varios robots hablandole a la vez, se va a agobiar

# IDEA CLAVE: Hacer el chat general, que si un usuario escribe o habla a traves de la interfaz, pues se ponga que esa
# persona ha puesto ese mensaje y sancho responda a eso. Que si habla alguien en persona y sabe quien es (lo tiene como
# interlocutor), pues tambien ponga quien es, si no que ponga Unknown. Creo que esto es puntazo para solucionar esto
# Y que los que hablen usando la interfaz web mande el audio por websocket o incluso se haga el TTS en el frontend

# idea a futuro: asociar la cara de una persona con la cuenta de firebase o lo que sea y al iniciar sesion
# sepa quien eres a nivel de cara y usando los registros de la base de datos de caras pues sepa la ultima vez
# que te vio si habia mas gente cosas asi
# En la bd de caras tambien poner logs de caras y salga la fecha en la que esta y mucha info en plan asi

# https://chatgpt.com/c/67dac8c1-affc-800f-91d3-02cb7a121d57






# quiza el articulo se podria hacer sobre esto, al menos en el tfg estara:

# opciones sobre llms
# 0. Procesar con palabras clave (simple ai)

# A. Embeddings para detectar la intencion (embeddings ai)
    # A.1 Con una frase que explique todo (json con command: description)
    # A.2 Con ejemplos (json con command: [examples])

# B. Prompt para detectar la intencion, y prompt para generar la respuesta (classification hri ai)
# (json con {command, description, arguments}, prompt system chetao para intencion y otro para generar respuesta)

# C. Prompt para detectar la intencion, y lista de templates para generar la respuesta (classification templates ai)
# (json con {command, description, arguments}, prompt sstem chetoa para intencion y coger uno random de la lista de templates)

# D. Prompt para detectar la intencion y generar el template (unified ai)
# (json con {command, description, arguments}, prompt system unico con todo lo de B en uno, mas lo que se quiera)

# medir gasto de tokens, tiempo, eficiencia...

# Extra: implementacion con ros y ros2web
# llm_tools
# como hacer pruebas a tu mierdon este
# sistema para hacer todo esto solo definiendo los comandos argumentso y demas