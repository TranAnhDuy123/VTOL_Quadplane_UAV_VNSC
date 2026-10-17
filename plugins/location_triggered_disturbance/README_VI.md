# Disturbance tự động theo vị trí cho Flight_Test.plan

## 1. Vị trí kích hoạt

Mission sử dụng home / VTOL takeoff tại:

- latitude: -35.3632621 deg
- longitude: 149.1652374 deg
- relative altitude: 50 m

Tọa độ địa phương tính từ điểm này:

| Điểm | North (m) | East (m) | Gazebo x (m) | Gazebo y (m) |
|---|---:|---:|---:|---:|
| WP2 | 910.482 | -69.429 | -69.429 | 910.482 |
| WP3 | 979.433 | 858.025 | 858.025 | 979.433 |
| Trung điểm WP2-WP3 | 944.958 | 394.298 | 394.298 | 944.958 |

Model đang dùng phép biến đổi `gazeboXYZToNED = 180 0 90`, nên Gazebo
`x = East`, `y = North`, `z = Up`.

Vùng trigger đã được đặt quanh trung điểm:

```text
x = 386 ... 402 m
y = 925 ... 965 m
z = 35  ... 70  m
```

Vùng dài 16 m theo hướng bay và rộng 40 m theo phương ngang. UAV bay khoảng
15 m/s nên chắc chắn đi qua vùng, kể cả khi có một ít cross-track error.

## 2. Disturbance

```text
Moment roll thân máy bay: +0.08 N.m
Thời gian:                0.50 s
Số lần:                   1 lần mỗi lần khởi động Gazebo
```

Moment được biến đổi từ trục +X thân máy bay sang hệ tọa độ world ở từng bước
mô phỏng. Vì vậy moment vẫn là roll disturbance khi heading thay đổi.

## 3. Biên dịch plugin

Mở terminal tại thư mục này:

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

Trước khi chạy Gazebo trong cùng terminal:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

Không cần `sudo make install`.

## 4. Thay model.sdf

Sao lưu model hiện tại, sau đó dùng file
`model_with_location_disturbance.sdf` làm `model.sdf` của
`model://VTOL_Quadplane`.

Không thay đổi world hoặc mission. World hiện tại có thể giữ nguyên plugin
`ApplyLinkWrench`; plugin mới không phụ thuộc vào thao tác terminal.

## 5. Kiểm tra trước khi bay A/B

Khi Gazebo khởi động, terminal phải có dòng:

```text
[LocationTriggeredWrench] Armed ...
```

Khi UAV đi vào vùng, terminal tự in:

```text
[LocationTriggeredWrench] ON ...
[LocationTriggeredWrench] OFF ...
```

Nếu không có dòng `Armed`, dừng bài bay và kiểm tra
`GZ_SIM_SYSTEM_PLUGIN_PATH`; không dùng log đó để so sánh.

## 6. Hai lượt bay

Giữ nguyên mission và toàn bộ controller configuration:

1. PID: `RLL_INDI_EN = 1`.
2. INDI: `RLL_INDI_EN = 2`.

Mỗi lượt phải khởi động lại Gazebo để trigger trở về trạng thái `armed`.
Không thay moment hoặc vùng kích hoạt giữa hai lượt.

Chỉ số so sánh chính sau onset:

- peak roll-angle error;
- peak roll-rate error;
- roll-rate RMSE / MAE trong 5 s;
- settling time về `|roll error| < 2 deg`;
- maximum cross-track error và recovery cross-track RMSE;
- peak controller output.

## 7. Đổi mức disturbance sau này

Chỉ sửa trong plugin block ở cuối model SDF:

```xml
<roll_torque_nm>0.08</roll_torque_nm>
<duration_s>0.50</duration_s>
```

Nên hoàn thành cặp PID/INDI ở 0.08 N.m trước khi tăng mức. Không đổi gain INDI
hoặc PID trong quá trình test disturbance.
