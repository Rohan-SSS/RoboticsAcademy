import React, { useEffect, useRef, useState} from 'react';


function Camera() {
  const videoRef = useRef(null);
  const streamRef = useRef(null);  // Usamos useRef para manejar el stream
  const [stream, setStream] = useState(null);
  // Obtener el stream de la cámara
  useEffect(() => {
    const startCamera = () => {
	    if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
	      navigator.mediaDevices.getUserMedia({ video: true })
		.then(stream => {
		  // Establecer el stream y asignarlo al video
		  setStream(stream);
		  if (videoRef.current) {
		    videoRef.current.srcObject = stream;
		    streamRef.current = stream; // Guardamos el stream en la referencia
		  }
		})
		.catch(err => console.log(err));
	    }
    };

    startCamera();
    
        // Limpiar el stream cuando el componente se desmonte
    return () => {
      if (streamRef,current) {
        streamRef.getTracks().forEach(track => track.stop());
      }
    };
  }, []);

  // Función para capturar un fotograma del video y convertirlo en una matriz CV_8UC4
  const captureFrame = () => {
    const video = videoRef.current;
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');

    if (video && canvas && ctx) {
      // Establecer el tamaño del canvas igual al tamaño del video
      canvas.width = 320;
      canvas.height = 240;
      
      // Dibujar el frame del video en el canvas
      ctx.drawImage(video, 0, 0, canvas.width, canvas.height);

      // Obtener los datos de la imagen (array de píxeles RGBA)
      const imageDataURL = canvas.toDataURL('image/jpeg');
      // Codificamos en base64
      // Enviar la matriz por WebSocket
      window.RoboticsExerciseComponents.commsManager.send("gui", `pick${imageDataURL}`);


    }
  };

  // Llamar a captureFrame cada 100 ms para enviar imágenes de la cámara
  useEffect(() => {
    const interval = setInterval(() => {
      captureFrame();
    }, 2000); // 10 fotogramas por segundo (100 ms)

    return () => clearInterval(interval);
  }, []);
  

  
  return (
    <div style={{display: "flex", width: "100%", height: "100%"}}>
      <video ref={videoRef} autoPlay></video>
    </div>
  );
}

export default Camera;
