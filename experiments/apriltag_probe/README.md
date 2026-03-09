# AprilTag Probe

ทดลองอ่าน `AprilTag 52h13` แบบแยกจาก mission หลัก เพื่อดู:

- `tag_id`
- ค่า `AB / C / DE`
- ขนาด tag ในภาพ (`side_px`)
- ระยะประมาณ (`approx_dist`) ถ้ามี `fx`
- ระยะและมุม (`roll / pitch / yaw`) ถ้ามี camera intrinsics ครบ
- มุม tag เทียบแกนกล้อง (`cam_angle_x / cam_angle_y`)

## รันแบบง่าย

```bash
cd /home/besstsu/Documents/AmazingRobot
python3 experiments/apriltag_probe/read_apriltag_pose.py --camera-index 0
```

## รันจาก device path แบบบังคับ V4L2/MJPG

```bash
cd /home/besstsu/Documents/AmazingRobot
python3 experiments/apriltag_probe/read_apriltag_pose.py \
  --device /dev/video0 \
  --backend v4l2 \
  --fourcc MJPG \
  --width 640 --height 480
```

## รันจากภาพนิ่ง

```bash
cd /home/besstsu/Documents/AmazingRobot
python3 experiments/apriltag_probe/read_apriltag_pose.py \
  --image /path/to/tag_image.jpg
```

## รันจากวิดีโอ

```bash
cd /home/besstsu/Documents/AmazingRobot
python3 experiments/apriltag_probe/read_apriltag_pose.py \
  --video /path/to/tag_video.mp4
```

## รันแบบมีระยะประมาณจาก pinhole

```bash
cd /home/besstsu/Documents/AmazingRobot
python3 experiments/apriltag_probe/read_apriltag_pose.py \
  --camera-index 0 \
  --tag-size-cm 4.2 \
  --fx 700
```

## รันแบบมี pose เต็ม

ต้องมีค่ากล้องจริง `fx fy cx cy`

```bash
cd /home/besstsu/Documents/AmazingRobot
python3 experiments/apriltag_probe/read_apriltag_pose.py \
  --camera-index 0 \
  --tag-size-cm 4.2 \
  --fx 700 --fy 700 --cx 320 --cy 240
```

## รันแบบมีมุมเทียบกล้องจาก FOV

ถ้ายังไม่มี `fx fy cx cy` แต่รู้มุมกล้องคร่าวๆ

```bash
cd /home/besstsu/Documents/AmazingRobot
python3 experiments/apriltag_probe/read_apriltag_pose.py \
  --image /path/to/tag_image.jpg \
  --hfov-deg 70 --vfov-deg 43
```

## หมายเหตุ

- ถ้าไม่ใส่ `fx fy cx cy` ครบ ระบบจะไม่คำนวณ pose
- ถ้ามีแค่ `fx` ระบบจะยังคำนวณระยะประมาณจาก `tag_size_cm * fx / side_px`
- `cam_angle_x / cam_angle_y` จะใช้ intrinsics ก่อน และ fallback ไป `hfov/vfov` ถ้าให้มา
- กด `q` เพื่อออก
- ถ้าอยู่บน WSL แล้วกล้องสดค้าง ให้ลองดึงภาพด้วย `v4l2-ctl` แล้วใช้ `--image` แทน
