import cv2

# Abre la cámara (0 es la cámara predeterminada)
cap = cv2.VideoCapture(0)

# Verifica si la cámara se abrió correctamente
if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

while True:
    # Lee un fotograma de la cámara
    ret, frame = cap.read()

    # Si no se pudo leer el fotograma, salimos del bucle
    if not ret:
        print("No se pudo recibir un fotograma")
        break

    # Muestra el fotograma en una ventana
    cv2.imshow('Camera', frame)

    # Sale si presionas la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera la cámara y cierra las ventanas abiertas
cap.release()
cv2.destroyAllWindows()
