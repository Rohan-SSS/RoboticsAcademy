import json
import cv2
import base64
import threading
import time

from gui_interfaces.general.measuring_threading_gui import MeasuringThreadingGUI
from src.manager.ram_logging.log_manager import LogManager
from console import start_console
import numpy as np


# Graphical User Interface Class

class GUI(MeasuringThreadingGUI):

    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        # Execution control vars
        self.image_to_be_shown = None
        self.image_to_be_shown_updated = False
        self.image_show_lock = threading.Lock()
        #self.ack = True
        #self.ack_lock = threading.Lock()
        #self.running = True

        self.host = host
        self.payload = {"image": ""}
        self.frame_rgb = None

        self.start()
        
    # Process incoming messages to the GUI
    def gui_in_thread(self, ws, message):
        # In this case, incoming msgs can only be acks
        if "ack" in message:
            with self.ack_lock:
                self.ack = True
                self.ack_frontend = True
        elif "pick" in message:
            print ("pick")
            message_array=np.fromstring(message, dtype=int, sep=',')
            message_array.resize(240,320,3)
            frame_int64 = message_array
            frame_bgr = np.uint8(frame_int64)
            self.frame_rgb = cv2.cvtColor(frame_bgr ,cv2.COLOR_BGR2RGB)

    # Prepares and sends a map to the websocket server
    def update_gui(self):

        payload = self.payloadImage()
        self.payload["image"] = json.dumps(payload)  
       
        message = json.dumps(self.payload)
        self.send_to_client(message)

    # Function to prepare image payload
    # Encodes the image as a JSON string and sends through the WS
    def payloadImage(self):
        with self.image_show_lock:
            image_to_be_shown_updated = self.image_to_be_shown_updated
            image_to_be_shown = self.image_to_be_shown

        image = image_to_be_shown
        payload = {'image': '', 'shape': ''}

        if not image_to_be_shown_updated:
            return payload

        shape = image.shape
        
        frame = cv2.imencode('.JPEG', image)[1]
        encoded_image = base64.b64encode(frame)

        payload['image'] = encoded_image.decode('utf-8')
        payload['shape'] = shape

        with self.image_show_lock:
            self.image_to_be_shown_updated = False

        return payload
    
    # Function for student to call
    def showImage(self, image):
        with self.image_show_lock:
            self.image_to_be_shown = image
            self.image_to_be_shown_updated = True


    def getImage(self):
        # TEMPORAL PARA PROBAR EL ENVIO DE IMAGENES   
        self.frame_rgb = np.ones((240, 320, 3), dtype="uint8") * 0  # Blanco

        # Establecer el texto a mostrar
        texto = "Hola"

        # Definir la fuente, tamaño, color y grosor del texto
        fuente = cv2.FONT_HERSHEY_SIMPLEX
        tamaño_fuente = 1
        color = (255, 0, 0)  # Negro
        grosor = 2

        # Obtener el tamaño del texto para centrarlo
        (tamaño_texto, _) = cv2.getTextSize(texto, fuente, tamaño_fuente, grosor)
        ancho_texto, alto_texto = tamaño_texto

        # Calcular las coordenadas para centrar el texto
        pos_x = (self.frame_rgb.shape[1] - ancho_texto) // 2
        pos_y = (self.frame_rgb.shape[0] + alto_texto) // 2

        # Poner el texto sobre la imagen
        cv2.putText(self.frame_rgb, texto, (pos_x, pos_y), fuente, tamaño_fuente, color, grosor)
        return self.frame_rgb
            
host = "ws://127.0.0.1:2303"
gui = GUI(host)

# Redirect the console
start_console()


# Expose the user functions
def showImage(image):
    gui.showImage(image)


def getImage():
    return gui.getImage()
