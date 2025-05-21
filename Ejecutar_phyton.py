import cv2
import numpy as np
import urllib.request
import requests
import time

# Dirección IP del ESP32 (modifica según corresponda)
ESP32_IP = "http://192.168.3.55"
motor_url = ESP32_IP + "/setMotor"

# Parámetros del PID (ajusta según sea necesario)
Kp = 0.6
Ki = 0.0000001
Kd = 0.06

integral = 0.0
last_error = 0.0
last_time = time.time()

base_speed = 255  # Valor base PWM (0-255)

def send_motor_speeds(left, right):
    url = f"{motor_url}?left={left}&right={right}"
    try:
        response = requests.get(url, timeout=1)
        print(f"Enviado: Izquierda={left}, Derecha={right} -> {response.text}")
    except Exception as e:
        print("Error enviando velocidades:", e)

def process_frame(frame):
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape

    # Definir una región de interés (ROI) en la parte inferior de la imagen
    roi = gray[int(height * 0.7):, :]
    
    # Aplicar umbralización para obtener una imagen binaria
    ret, thresh = cv2.threshold(roi, 100, 255, cv2.THRESH_BINARY_INV)
    
    # Mostrar la imagen binaria en una ventana separada
    cv2.imshow("Imagen Binaria", thresh)
    
    # Encontrar contornos en la imagen binaria
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    error = 0
    if contours:
        # Se selecciona el contorno de mayor área
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            # Calcular el centroide del contorno encontrado
            cx = int(M["m10"] / M["m00"])
            # Dibujar el centroide en la imagen original (ajustando la posición en función de la ROI)
            cv2.circle(frame, (cx, int(height * 0.7) + 10), 5, (0, 0, 255), -1)
            # Error definido como la diferencia entre el centroide y el centro de la imagen completa
            error = cx - (width / 2)
    else:
        # Si no se detecta contorno, se puede definir el error como cero o mantener el último valor
        error = 0

    return frame, error

def compute_pid(error):
    global integral, last_error, last_time
    current_time = time.time()
    dt = current_time - last_time
    if dt == 0:
        dt = 1e-3
    integral += error * dt
    derivative = (error - last_error) / dt
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    last_time = current_time
    return output

def main():
    stream_url = ESP32_IP + ":81/"
    stream = urllib.request.urlopen(stream_url)
    bytes_data = b""
    
    while True:
        bytes_data += stream.read(1024)
        a = bytes_data.find(b'\xff\xd8')
        b = bytes_data.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                processed_frame, error = process_frame(frame)
                correction = compute_pid(error)
                # Calcula las velocidades de los motores a partir de la corrección
                left_speed = int(base_speed + correction)
                right_speed = int(base_speed - correction)
                left_speed = max(0, min(255, left_speed))
                right_speed = max(0, min(255, right_speed))
                
                send_motor_speeds(left_speed, right_speed)
                cv2.imshow("Frame", processed_frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break

if __name__ == "__main__":
    main()
    
