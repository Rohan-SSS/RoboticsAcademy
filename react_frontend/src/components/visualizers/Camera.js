import React, { useEffect, useRef, useState} from 'react';

function Camera() {
  const videoRef = useRef(null);
  const [stream] = useState(null);
  
  useEffect(() => {
    if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
      navigator.mediaDevices.getUserMedia({ video: true })
        .then(stream => {
          // Establecer el stream y asignarlo al video
          //setStream(stream);
          if (videoRef.current) {
            videoRef.current.srcObject = stream;
            try {             
            	window.RoboticsExerciseComponents.commsManager.send("gui", `pick${videoRef.current.srcObject}`)
	    } catch (error) 
	    {
	    }
          }
        })
        .catch(err => console.log(err));
    }
  }, []);
  

     
  return (
    <div style={{display: "flex", width: "100%", height: "100%"}}>
      <video ref={videoRef} autoPlay></video>
    </div>
  );
}

export default Camera;
