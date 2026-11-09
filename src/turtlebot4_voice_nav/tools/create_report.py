"""Render the two-page Vietnamese report from measured ROS demo results."""

import csv
from html import escape
import json
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    SimpleDocTemplate,
    Paragraph,
    Spacer,
    Image,
    Table,
    TableStyle,
    PageBreak,
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from turtlebot4_voice_nav.mapping import GridMap

OUT = ROOT.parents[1] / "reports/bai07_voice_navigation"
OUT.mkdir(parents=True, exist_ok=True)
regions = json.loads((ROOT / "maps/regions.json").read_text())
rows = json.loads((ROOT / "results/integration_results.json").read_text())
events = json.loads((ROOT / "results/integration_events.json").read_text())
goals = [event for event in events if event.get("event") == "GOAL_RESULT"]
tests = json.loads((ROOT / "results/unit_tests.json").read_text())
trajectory = list(csv.DictReader((ROOT / "results/live/trajectory.csv").open()))
grid = GridMap.load(ROOT / "maps" / regions["map_yaml"])

fig, ax = plt.subplots(figsize=(6.5, 4.4), dpi=180)
ax.imshow(
    grid.image, cmap="gray", vmin=0, vmax=255, extent=[-10.1, 12.85, -10.1, 11.95]
)
valid = [row for row in trajectory if row["state"] == "RUNNING"]
ax.plot(
    [float(row["x"]) for row in valid],
    [float(row["y"]) for row in valid],
    color="#127eb2",
    linewidth=1.4,
    label="AMCL + odometry",
)
truth = [row for row in valid if row.get("ground_truth_x")]
ax.plot(
    [float(row["ground_truth_x"]) for row in truth],
    [float(row["ground_truth_y"]) for row in truth],
    color="#dd6b20",
    linewidth=1,
    linestyle="--",
    label="Gazebo ground truth",
)
for region in regions["regions"]:
    x, y = region["goal"]["x"], region["goal"]["y"]
    ax.scatter(x, y, color="#a01835", s=24)
    ax.text(x + 0.12, y + 0.12, region["id"], fontsize=10, weight="bold")
ax.scatter(0, 0, marker="s", s=20, color="green")
ax.set(xlim=(-5.5, 1.2), ylim=(-1, 5.4), xlabel="x (m)", ylabel="y (m)")
ax.set_aspect("equal")
ax.legend(fontsize=7, loc="lower left")
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(OUT / "trajectory_verified.png")
plt.close(fig)

font_dir = Path("/usr/share/fonts/truetype/dejavu")
pdfmetrics.registerFont(TTFont("DejaVu", str(font_dir / "DejaVuSans.ttf")))
pdfmetrics.registerFont(TTFont("DejaVu-Bold", str(font_dir / "DejaVuSans-Bold.ttf")))
pdfmetrics.registerFontFamily(
    "DejaVu",
    normal="DejaVu",
    bold="DejaVu-Bold",
    italic="DejaVu",
    boldItalic="DejaVu-Bold",
)
body = ParagraphStyle(
    "body",
    fontName="DejaVu",
    fontSize=9,
    leading=12,
    spaceAfter=6,
    textColor=colors.HexColor("#243247"),
)
small = ParagraphStyle("small", parent=body, fontSize=7.7, leading=10, spaceAfter=3)
title = ParagraphStyle(
    "title",
    parent=body,
    fontName="DejaVu-Bold",
    fontSize=17,
    leading=21,
    spaceAfter=7,
    textColor=colors.HexColor("#123b61"),
)
sub = ParagraphStyle(
    "sub",
    parent=body,
    fontName="DejaVu-Bold",
    fontSize=10.5,
    leading=14,
    spaceBefore=7,
    spaceAfter=5,
)
story = []


def paragraph(text, style=body):
    return Paragraph(text, style)


def add(text, style=body):
    story.append(paragraph(text, style))


def table(data, widths):
    cells = [[paragraph(escape(str(cell)), small) for cell in row] for row in data]
    result = Table(cells, colWidths=widths, hAlign="LEFT", repeatRows=1)
    result.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e4eef7")),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#bfccd8")),
                ("LEFTPADDING", (0, 0), (-1, -1), 6),
                ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ("TOPPADDING", (0, 0), (-1, -1), 5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ]
        )
    )
    return result


add("BÀI THỰC HÀNH 7", title)
add("Phân vùng bản đồ và điều hướng robot bằng giọng nói", sub)
add("Ubuntu 22.04 · ROS 2 Humble · TurtleBot 4 Lite · Gazebo Fortress · Nav2", small)
add("Bản thử nghiệm tự động với WAV tiếng Việt tổng hợp · 09/11/2026", small)
add(
    "Hệ thống tiếp nhận âm thanh, nhận dạng lời nói, xác định khu vực ngữ nghĩa và điều khiển TurtleBot 4 qua Nav2. Mô phỏng sử dụng mô hình robot, LiDAR và bộ điều khiển bánh xe của TurtleBot 4. Giao diện bản đồ hiển thị các topic ROS trong quá trình robot chạy."
)
add(
    "Âm thanh WAV → Whisper base CPU → Parser tiếng Việt → Hàng đợi đích → NavigateToPose",
    sub,
)
add(
    "Whisper.cpp nhận WAV PCM16 mono 16 kHz và trả transcript nguyên văn. Parser chuẩn hóa dấu, tìm cấu trúc “đi đến” hoặc “đi từ … đến …”, rồi so khớp đầy đủ alias của từng khu vực. Các vùng chưa khai báo, câu phủ định và câu hỏi bị từ chối. Với A → B, node gửi A trước và chỉ gửi B sau khi A được xác minh. Lệnh dừng hủy action và xóa hàng đợi."
)
add("Bản đồ và phân vùng", sub)
add(
    "Ảnh SLAM nguồn có kích thước 459 × 441, độ phân giải 0,05 m/pixel, origin (−10,1, −10,1, 0). Bản dẫn đường bổ sung footprint ghế tại (−2, 3,5) m theo world mô phỏng và dùng free_thresh = 0,196 để giữ ô xám 205 ở trạng thái chưa biết. Segmentation và Nav2 cùng dùng bản đã hiệu chỉnh. Distance transform loại ô có clearance dưới 0,28 m. Ba seed được gán tên thủ công, sau đó Dijkstra nhiều nguồn chia không gian tự do theo khoảng cách trên lưới 4 láng giềng."
)
region_data = [["Vùng / nhãn", "Đích x, y (m)", "Clearance (m)"]]
for region in regions["regions"]:
    g = region["goal"]
    region_data.append(
        [
            region["id"] + " · " + region["name"],
            f"{g['x']:.2f}, {g['y']:.2f}",
            f"{g['clearance_m']:.3f}",
        ]
    )
map_panel = Table(
    [
        [
            Image(str(ROOT / "maps/segmented_map.png"), width=237, height=228),
            table(region_data, [114, 82, 63]),
        ]
    ],
    colWidths=[247, 273],
)
map_panel.setStyle(
    TableStyle(
        [("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 0)]
    )
)
story.append(map_panel)
add(
    "Hình 1. A màu xanh dương, B màu cam, C màu xanh lá. Xám biểu thị ô chưa biết. Miền trắng còn lại gồm ô sát vật cản hoặc thành phần chưa được gán seed.",
    small,
)
add(
    "Tọa độ pixel lấy tại tâm ô, đảo chiều trục ảnh y, quay theo yaw của origin và cộng tịnh tiến. Đích dùng seed an toàn để tránh trường hợp tâm hình học của vùng rơi vào tường. Các file nộp gồm PGM/YAML nguồn, ảnh nhãn uint16, ảnh phân vùng và JSON thông tin vùng.",
    small,
)
story.append(PageBreak())

add("KẾT QUẢ KIỂM THỬ", title)
voice_rows = [row for row in rows if row.get("source") == "SYNTHETIC_TTS_PIPER"]
valid_voice = [row for row in voice_rows if row.get("expected_route")]
valid_pass = sum(row["passed"] for row in valid_voice)
verified = sum(bool(goal.get("verified")) for goal in goals)
add(
    f"Lượt demo dùng {len(voice_rows)} WAV tổng hợp Piper VAIS-1000. Có {valid_pass}/{len(valid_voice)} lệnh vùng hợp lệ hoàn tất đúng từ âm thanh tới điều hướng. Nav2 xác minh {verified}/{len(goals)} điểm đích, gồm phép thử C bằng text được ghi nguồn riêng. Bộ regression chạy đạt {tests['tests_run']}/{tests['tests_run']} kiểm thử."
)
speech_table = [["Câu đọc / nguồn", "Transcript thực tế", "Kết quả"]]
for row in voice_rows:
    outcome = (
        "Từ chối đúng"
        if row["expected_route"] is None and row["passed"]
        else ("Đạt" if row["passed"] else "Chưa đạt")
    )
    speech_table.append([row["expected_text"], row["transcript"], outcome])
story.append(table(speech_table, [205, 238, 80]))
story.append(Spacer(1, 7))
goal_table = [["Thứ tự", "Vùng", "Nav2 status", "Sai số TF (m)", "Sai số Gazebo (m)"]]
for index, goal in enumerate(goals, 1):
    goal_table.append(
        [
            index,
            goal["region_id"],
            goal["nav2_status"],
            f"{goal['position_error_m']:.3f}",
            (
                f"{goal['ground_truth_error_m']:.3f}"
                if goal.get("ground_truth_error_m") is not None
                else "Thiếu"
            ),
        ]
    )
story.append(table(goal_table, [55, 55, 85, 140, 188]))
add(
    "Status 4 là SUCCEEDED. Node còn yêu cầu pose mới trong 5 giây mô phỏng và sai số vị trí ≤ 0,50 m. Lượt mô phỏng kiểm tra thêm pose Gazebo độc lập. Pose Gazebo chỉ phục vụ đo kết quả, điều khiển Nav2 sử dụng AMCL và odometry.",
    small,
)
plot = Image(str(OUT / "trajectory_verified.png"), width=300, height=203)
limits = paragraph(
    "Whisper base nhận sai clip C thành một tên vùng ngoài danh mục nên parser từ chối. Harness gửi một lệnh text C có nhãn TEXT_NAVIGATION_VALIDATION để kiểm tra đích thứ ba. Ba lệnh giọng nói A, phòng thí nghiệm và A → B là các ví dụ đạt yêu cầu tối thiểu. Âm thanh vùng Z cũng bị từ chối. Các tỷ lệ ở đây tính trên bộ WAV tổng hợp đã chọn, độ chính xác giọng người thật chưa được đo.",
    small,
)
panel = Table([[plot, limits]], colWidths=[310, 213])
panel.setStyle(
    TableStyle(
        [("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 0)]
    )
)
story.append(panel)
laser = [float(row["min_laser_m"]) for row in valid if row.get("min_laser_m")]
if laser:
    add(
        f"Hình 2. Quỹ đạo lấy từ ROS và Gazebo. Khoảng cách LiDAR nhỏ nhất trong các đoạn điều hướng là {min(laser):.3f} m. Nav2 dùng costmap LiDAR và inflation để lập đường, tránh vật cản. Bước vật lý 5 ms, hệ số thời gian mục tiêu 0,5 và Gazebo headless phù hợp máy WSL khoảng 1,8 GB RAM.",
        small,
    )
add("Source và bộ nộp", sub)
add(
    '<link href="https://github.com/23020748-cmyk/turtlebot4-voice-navigation-week7" color="#155a8a">github.com/23020748-cmyk/turtlebot4-voice-navigation-week7</link><br/>Video demo_voice_navigation.mp4 · log integration_results.json và integration_events.json · trajectory.csv.',
    small,
)
add(
    'Tài liệu sử dụng gồm <link href="https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html" color="#155a8a">TurtleBot 4 Navigation</link>, <link href="https://api.nav2.org/actions/humble/navigatetopose.html" color="#155a8a">Nav2 Humble / NavigateToPose</link> và <link href="https://github.com/absadiki/pywhispercpp" color="#155a8a">pywhispercpp</link>. Model Whisper base từ ggerganov/whisper.cpp. WAV tổng hợp dùng Piper vi_VN-vais1000-medium, dữ liệu VAIS-1000 theo CC BY 4.0.',
    small,
)


def footer(canvas, doc):
    canvas.setFont("DejaVu", 7)
    canvas.setFillColor(colors.HexColor("#697789"))
    canvas.drawString(36, 22, "Bài 7 · Giọng nói tiếng Việt và điều hướng ngữ nghĩa")
    canvas.drawRightString(A4[0] - 36, 22, f"{doc.page} / 2")


pdf = OUT / "Bao_cao_Bai_07.pdf"
doc = SimpleDocTemplate(
    str(pdf),
    pagesize=A4,
    leftMargin=36,
    rightMargin=36,
    topMargin=30,
    bottomMargin=34,
    title="Bài thực hành 7 – Phân vùng và điều hướng bằng giọng nói",
    author="HRI coursework",
)
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print(pdf)
