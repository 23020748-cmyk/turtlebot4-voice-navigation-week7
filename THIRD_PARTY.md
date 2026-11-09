# Nguồn và giấy phép

Mã Bài 7 dùng Apache-2.0. Package hỗ trợ mô phỏng `turtlebot4_slam_nav2` được lấy từ workspace của dự án, giữ các thông báo copyright của Clearpath Robotics trong các launch file. Bản đồ SLAM nguồn và hình học maze được kế thừa từ package đó. Bản world Bài 7 điều chỉnh bước vật lý và hệ số thời gian cho mô phỏng.

TurtleBot 4, Create 3 và Nav2 được cài từ các package ROS chính thức. Các mô hình và thư viện hệ thống được tham chiếu qua ROS package share directory.

Whisper model base đa ngôn ngữ được tải riêng từ [ggerganov/whisper.cpp](https://huggingface.co/ggerganov/whisper.cpp). Whisper và whisper.cpp có giấy phép MIT. Python binding được dùng qua [pywhispercpp](https://github.com/absadiki/pywhispercpp), giấy phép MIT. URL tải, kích thước và SHA-256 nằm trong `models/whisper_manifest.json`.

Các WAV demo là tiếng nói tổng hợp, tạo bằng giọng Piper `vi_VN-vais1000-medium`. Giọng dựa trên VAIS-1000 Vietnamese Speech Synthesis Corpus, giấy phép CC BY 4.0 theo model card được phân phối với giọng. Credit thuộc VAIS-1000 corpus contributors và Piper voice contributors. Các WAV được resample từ 22.050 Hz sang PCM16 mono 16.000 Hz để dùng với ASR.

- [Model card Piper VAIS-1000](https://huggingface.co/rhasspy/piper-voices/blob/main/vi/vi_VN/vais1000/medium/MODEL_CARD)
- [VAIS-1000 corpus](https://ieee-dataport.org/documents/vais-1000-vietnamese-speech-synthesis-corpus)
- [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)

OpenCV dùng giấy phép Apache-2.0 ở các bản mới và BSD ở các bản cũ. NumPy, Pillow, PyYAML và Matplotlib giữ giấy phép upstream tương ứng. FFmpeg được gọi như chương trình hệ thống để capture, resample và tạo video.
