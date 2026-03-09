# Camera Calibration

ใช้สคริปต์นี้หาค่า `APRILTAG_FX`, `APRILTAG_FY`, `APRILTAG_CX`, `APRILTAG_CY` ของกล้องหน้า
เพื่อให้ `AprilTag pose x/z/yaw` ใน `AmazingRobot` แม่นขึ้น

## ใช้อะไร

- กระดาน chessboard
- ค่า default ของสคริปต์คือ `9x6` inner corners
- ค่า default `square-size-mm=25.0`

ถ้ากระดานของคุณไม่ใช่ 9x6 หรือขนาดช่องไม่ใช่ 25 mm ต้องใส่ค่าให้ตรงจริง

## สร้างกระดานมาตรฐานเอง

ถ้ายังไม่แน่ใจว่ากระดานที่ใช้อยู่เป็น chessboard มาตรฐานหรือไม่ ให้สร้าง PNG จากโปรเจกต์นี้:

```bash
cd /home/besstsu/Documents/AmazingRobot
python experiments/camera_calibration/generate_chessboard.py
```

ไฟล์จะถูกสร้างที่:

```text
experiments/camera_calibration/chessboard_9x6.png
```

นำไปพิมพ์ออกกระดาษแบบไม่ยืดสัดส่วน และวัดขนาดช่องจริงก่อนใส่ `--square-size-mm`

## วิธีรันแบบถ่ายสด

```bash
cd /home/besstsu/Documents/AmazingRobot
source venv/bin/activate
python experiments/camera_calibration/calibrate_camera.py --camera-index 0
```

ปุ่ม:
- `SPACE` บันทึกเฟรมที่เจอ chessboard
- `C` เริ่ม calibrate เมื่อมีภาพพอ
- `Q` ออก

ควรถ่ายอย่างน้อย 10-20 ภาพ
และให้มุม/ระยะของกระดานหลากหลาย

## วิธีรันบน Pi แบบไม่มีจอ (headless / SSH)

ถ้ารันผ่าน SSH แล้วไม่มี X display ให้ใช้โหมดนี้:

```bash
cd /home/besstsu/Documents/AmazingRobot
source venv/bin/activate
python experiments/camera_calibration/calibrate_camera.py \
  --device /dev/video0 \
  --headless \
  --capture-count 12 \
  --capture-interval-sec 1.2
```

สคริปต์จะเซฟภาพอัตโนมัติเมื่อเจอ chessboard
คุณต้องถือกระดานในมุม/ระยะต่างๆ เองระหว่างรัน

ถ้า OpenCV เปิดกล้องผ่าน SSH แล้วเจอปัญหา ให้บังคับใช้ `ffmpeg`:

```bash
python experiments/camera_calibration/calibrate_camera.py \
  --device /dev/v4l/by-id/<camera>-video-index0 \
  --backend v4l2 \
  --fourcc MJPG \
  --width 640 --height 480 \
  --headless \
  --headless-method ffmpeg \
  --capture-count 12 \
  --capture-interval-sec 1.2
```

## วิธีรันจากภาพที่มีอยู่แล้ว

```bash
cd /home/besstsu/Documents/AmazingRobot
source venv/bin/activate
python experiments/camera_calibration/calibrate_camera.py \
  --images-glob 'experiments/camera_calibration/captures/*.jpg'
```

ถ้าต้องการเช็คว่ารูปไหนเจอ pattern บ้าง:

```bash
python experiments/camera_calibration/calibrate_camera.py \
  --images-glob 'experiments/camera_calibration/captures/*.jpg' \
  --debug-detect
```

## ถ้ากระดานไม่ใช่ 9x6

ตัวอย่าง 8x6 ช่องใน, ช่องละ 20 mm

```bash
python experiments/camera_calibration/calibrate_camera.py \
  --camera-index 0 \
  --cols 8 --rows 6 \
  --square-size-mm 20
```

## ผลลัพธ์

สคริปต์จะสร้างไฟล์:

```text
experiments/camera_calibration/calibration_result.json
```

และพิมพ์ค่าที่เอาไปใส่ `.env` ได้ตรงๆ เช่น:

```env
APRILTAG_FX=712.400000
APRILTAG_FY=709.800000
APRILTAG_CX=318.600000
APRILTAG_CY=241.200000
```

## เกณฑ์ดูว่าค่าใช้ได้ไหม

- `mean_reprojection_error_px` ยิ่งต่ำยิ่งดี
- โดยทั่วไป:
  - `< 0.5 px` ดีมาก
  - `< 1.0 px` ใช้งานได้ดี
  - `> 1.5 px` ควรถ่ายใหม่

## หมายเหตุ

- ต้อง calibrate ใหม่เมื่อเปลี่ยนกล้อง หรือเปลี่ยน resolution
- ถ้าแค่วางหุ่นคนละมุม ไม่ต้อง calibrate ใหม่
