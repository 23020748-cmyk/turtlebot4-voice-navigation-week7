# Bài 7 Phân vùng bản đồ và điều hướng TurtleBot 4 bằng giọng nói

Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress, TurtleBot 4 Lite và Nav2.

## Phương pháp

`mapping.py` đọc PGM và YAML, chuyển ô bản đồ sang xác suất chiếm chỗ rồi dùng distance transform để loại ô sát vật cản dưới 0,28 m. Ba seed ngữ nghĩa được đặt trong `config/region_seeds.yaml`. Multi-source Dijkstra trên lưới 4 láng giềng chia miền có thể đi theo khoảng cách đường đi. Tên phòng do người triển khai gán, kết quả là phân vùng bán tự động.

`maps/original/` chứa bản đồ nguồn. `maps/navigation_map.yaml` dùng ảnh PGM dẫn đường đã bổ sung footprint ghế `demo_chair` tại (-2, 3,5) m theo hình học world mô phỏng. Footprint được khai báo trong `known_obstacles` của file seed. `free_thresh` đổi từ 0,25 sang 0,196 để giá trị xám 205 thuộc miền chưa biết. Nav2 và segmentation cùng dùng bản đồ đã hiệu chỉnh. `region_labels.png` là ảnh nhãn uint16, 0 dành cho ô chưa gán. `regions.json` lưu tên, alias, tọa độ đích, clearance, diện tích và hash ảnh bản đồ.

Tọa độ tại tâm pixel được tính bằng `x = (col + 0.5) * resolution` và `y = (height - row - 0.5) * resolution`, sau đó quay theo yaw và cộng origin của YAML. Đích được chọn tại seed an toàn vì tâm hình học của vùng có thể nằm trong vật cản.

| ID | Nhãn | Đích x, y (m) | Yaw (rad) | Clearance bản đồ (m) |
|---|---|---|---|---|
| A | Phòng học | -3,50, 0,50 | 1,43 | 1,350 |
| B | Phòng thí nghiệm | -4,00, 2,50 | 1,82 | 1,006 |
| C | Hành lang | -3,00, 4,50 | 1,107 | 1,061 |

Whisper base đa ngôn ngữ chạy offline trên CPU qua `pywhispercpp`. Văn bản ASR được chuyển nguyên vẹn đến parser. Parser chuẩn hóa dấu và so khớp đầy đủ từng tên khu vực. Lệnh chứa vùng không có trong danh mục bị từ chối trước khi gửi goal. Confidence của Whisper được để `null` vì API đang dùng không cung cấp xác suất đã hiệu chuẩn.

`navigation_node.py` đợi lifecycle của bt_navigator ở trạng thái ACTIVE rồi gửi từng `NavigateToPose` trong frame `map`. Với lệnh đi từ A đến B, hàng đợi chứa A rồi B. Goal sau chỉ được gửi khi Nav2 thành công, pose TF map còn mới và sai số vị trí tới goal không quá 0,50 m. TF kết hợp AMCL và wheel odometry. Trong launch mô phỏng, pose Gazebo được dùng để kiểm chứng độc lập sai số đích. Nav2 điều khiển bằng định vị AMCL. Goal bị từ chối, thất bại hoặc quá thời gian sẽ xóa phần còn lại của hàng đợi. Lệnh dừng hủy action và phát vận tốc bằng 0.

## Cài đặt

Đặt package này và `turtlebot4_slam_nav2` kèm bộ nộp trong thư mục `src` của một ROS workspace. Cài ROS 2 Humble và TurtleBot 4 Simulator theo tài liệu chính thức trước khi build.

```bash
sudo apt update
sudo apt install python3-venv python3-opencv python3-numpy python3-yaml \
  python3-pil python3-pil.imagetk python3-tk ffmpeg \
  ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-ros-gz-bridge
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4_slam_nav2 turtlebot4_voice_nav
source install/setup.bash
bash src/turtlebot4_voice_nav/tools/setup_speech.sh
```

Model GGML base khoảng 148 MB được tải từ kho whisper.cpp chính thức trên Hugging Face. Script ghi SHA-256 và URL vào `models/whisper_manifest.json`. Trọng số được tải riêng khi cài đặt.

## Chạy demo tự động

Mở ba terminal Ubuntu trong workspace. Terminal 1 khởi động mô phỏng Gazebo headless, AMCL và Nav2. Chờ thông báo `Managed nodes are active` và `READY`.

```bash
bash src/turtlebot4_voice_nav/tools/run.sh simulation
```

Terminal 2 mở bản đồ trực tiếp từ các topic ROS. Chế độ demo khóa các nút nhập mic và WAV, giữ nút dừng hoạt động.

```bash
bash src/turtlebot4_voice_nav/tools/run.sh dashboard --demo-mode \
  --record src/turtlebot4_voice_nav/demo/dashboard_raw.mp4
```

Terminal 3 chạy nhận dạng các WAV tổng hợp rồi gửi câu nhận dạng đến Nav2.

```bash
bash src/turtlebot4_voice_nav/tools/run.sh demo
```

`demo/audio/manifest.json` ghi câu đọc và nguồn `SYNTHETIC_TTS_PIPER`. Đây là kiểm thử bằng tiếng nói tổng hợp theo yêu cầu demo tự động. Các giá trị trong `results/integration_results.json`, `integration_events.json` và `results/live/trajectory.csv` được ghi từ lần chạy thực tế. Khi ASR của C bị từ chối, harness có phép thử đích C độc lập với nguồn `TEXT_NAVIGATION_VALIDATION`.

Đóng cửa sổ dashboard để FFmpeg kết thúc bản ghi MP4. Dùng Ctrl+C ở terminal mô phỏng sau khi hoàn tất.

## Lệnh và giao diện

Các câu hợp lệ gồm “Đi đến khu vực A”, “Đi đến phòng thí nghiệm”, “Di chuyển từ khu vực A đến khu vực B” và “Đi từ phòng A qua hành lang C rồi đến phòng B”. Có thể nói “Dừng lại” để hủy. “Đi đến khu vực Z” bị từ chối. Câu phủ định và câu hỏi cũng bị từ chối.

Để thử tương tác, chạy dashboard bỏ tùy chọn `--demo-mode`. Nút GỬI TEXT và NHẬN DẠNG WAV dùng chung parser và action client với luồng giọng nói. Nút BẬT MIC dùng FFmpeg PulseAudio, VAD theo năng lượng rồi chuyển từng đoạn tiếng nói vào Whisper. Trên WSLg, nguồn mặc định thường là `RDPSource`. Cấp quyền microphone cho ứng dụng Windows/WSLg nếu cần. TẮT MIC chấm dứt capture.

```bash
# Chỉ nhận giọng nói trực tiếp và gửi lệnh ROS
bash src/turtlebot4_voice_nav/tools/run.sh microphone
# Nhận dạng một WAV PCM16 mono 16 kHz và gửi lệnh
bash src/turtlebot4_voice_nav/tools/run.sh wav src/turtlebot4_voice_nav/demo/audio/01_area_a.wav
# Tạo lại phân vùng và kiểm thử
bash src/turtlebot4_voice_nav/tools/run.sh segment
bash src/turtlebot4_voice_nav/tools/run.sh tests
```

Topic lệnh là `/voice_nav/command` kiểu `std_msgs/String`, nhận text hoặc JSON có `text`, `source`, `command_id`, `confidence`. `/voice_nav/status` chứa trạng thái, transcript, pose và tiến độ. `/voice_nav/regions` là MarkerArray nhãn đích để thêm vào RViz. `/plan` là đường Nav2, `/scan` cung cấp LiDAR để costmap tránh vật cản.

## Phạm vi kiểm chứng

Giới hạn khoảng 1,8 GB RAM của WSL khiến bản chạy dùng Gazebo headless, LiDAR và điều khiển bánh xe chính thức. Bước vật lý 5 ms, hệ số thời gian mục tiêu 0,5 giúp giảm trượt bánh do bước tích phân lớn. Timeout heartbeat lifecycle là 60 giây, timeout mỗi goal là 600 giây thực. Dashboard hiển thị pose và plan của mô phỏng. VAD cần điều chỉnh ngưỡng theo microphone và tiếng ồn. Độ chính xác giọng người thật sẽ được đo trong lần kiểm thử có người đọc lệnh.

Clip gọi C trong bộ WAV hiện nhận sai thành tên ngoài danh mục và bị từ chối. Demo chính kiểm chứng ba lệnh giọng nói A, phòng thí nghiệm và A → B, đồng thời kiểm tra đích C bằng text có ghi rõ nguồn. File kết quả giữ cả trường hợp ASR chưa đạt. Video ghép âm thanh từ WAV đầu vào theo timestamp bắt đầu ASR, giữ tốc độ thực và cắt phần chờ khởi động đầu buổi.

## Nguồn

- Quy trình localization và navigation theo [TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html).
- Action goal và feedback theo [Nav2 Humble NavigateToPose](https://api.nav2.org/actions/humble/navigatetopose.html).
- ASR dùng [pywhispercpp](https://github.com/absadiki/pywhispercpp) và model [whisper.cpp GGML base](https://huggingface.co/ggerganov/whisper.cpp).
- WAV demo được tạo với giọng Piper `vi_VN-vais1000-medium` từ VAIS-1000. Xem attribution và giấy phép giọng trong bộ nộp.
