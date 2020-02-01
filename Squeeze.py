import picamera
from picamera.array import PiRGBArray
import numpy as np
import time
from time import sleep
import datetime as dt
import cv2
import Rpi.Gpio as GPIO
pin=18
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)

# загружаем параметры сети
prototxt = 'deploy.prototxt'
model = 'squeezenet_v1.1.caffemodel'
labels = 'synset_words1.txt'

# загружаем распознаваемые классы
rows = open(labels).read().strip().split("\n")
classes = [r[r.find(" ") + 1:].split(",")[0] for r in rows]

# загружаем модель сети
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(prototxt, model)

print("[INFO] starting video stream...")
# инициализируем камеру
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25

# прогреваем камеру
camera.start_preview()
sleep(1)
camera.stop_preview()

# инициализируем кадр в формате raw 
rawCapture = PiRGBArray(camera)
# сбрасываем счетчик FPS
t0 = time.time()

# цикл обработки видео потока
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # захватываем кадр как blob
    frame = rawCapture.array
    blob = cv2.dnn.blobFromImage(frame, 1, (224, 224), (104, 117, 124))

    # загружаем в сеть blob, получаем класс и вероятность
    net.setInput(blob)
    preds = net.forward()
    preds = preds.reshape((1, len(classes)))
    idxs = int(np.argsort(preds[0])[::-1][:1])
    
    # вычисляем FPS
    FPS = 1/(time.time() - t0)
    t0 = time.time()

    # помещаем на кадр класс, вероятность и FPS, выводим в консоль
    if idxs<=8 :
      state_gpio18=1
      text = "Label: {}, p = {:.2f}%, fps = {:.2f}".format(classes[idxs], preds[0][idxs] * 100, FPS)
    #cv2.putText(frame, text, (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
      print(text)
    elif state_gpio18=1 :
      GPIO.cleanup()
      state_gpio18=0

    #cv2.imshow("Frame", frame)     # выводим кадр на дисплее Raspberry 
    #fname = 'pic_' + dt.datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + '.jpg'
    #cv2.imwrite(fname, frame)      # сохраняем кадр на SD диске
    key = cv2.waitKey(1) & 0xFF

    # если нажата кнопка `q` выходим из цикла
    if key == ord("q"): break

    # очищаем поток raw данных с камеры перед следующим циклом
    rawCapture.truncate(0)

print("[INFO] video stream is terminated")

# прибираем за собой
cv2.destroyAllWindows()
camera.close()
