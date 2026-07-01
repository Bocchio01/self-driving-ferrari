#!/usr/bin/env python3

import cv2
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

gst_pipeline = (
    "libcamerasrc ! "
    "video/x-raw, width=640, height=480 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true"
)

cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)


class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(b"<html><head><title>Freenove Stream</title></head><body>")
            self.wfile.write(b"<h1>Freenove IMX219 Live Stream</h1>")
            self.wfile.write(b"<img src='/stream.mjpg' />")
            self.wfile.write(b"</body></html>")
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            try:
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        continue

                    # Codifica il frame in JPEG in memoria
                    _, jpeg = cv2.imencode(".jpg", frame)
                    frame_bytes = jpeg.tobytes()

                    # Invia il frame nel flusso MJPEG
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame_bytes)))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.03)  # Limita a ~30 FPS per non sovraccaricare la CPU
            except Exception as e:
                print(f"Connessione chiusa dal browser: {e}")


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Gestisce le richieste in thread separati per non bloccare il video"""

    pass


if __name__ == "__main__":
    if not cap.isOpened():
        print("Errore: Impossibile aprire la telecamera.")
        exit()

    # Avvia il server sulla porta 8080 (puoi cambiarla se già occupata)
    server = ThreadedHTTPServer(("0.0.0.0", 8080), CamHandler)
    print("Server di streaming avviato!")
    print("Apri il browser sul tuo PC e vai su -> http://<IP_DEL_RASPBERRY>:8080")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nChiusura del server...")
    finally:
        cap.release()
        server.server_close()
