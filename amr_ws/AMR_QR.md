# AMR QR di stack SLAM — profil `amr-qr-01`

Stack SLAM ini (supervisor, slam_toolbox, AMCL, Nav2 RPP/Spin, route executor, web
`:5001`) sekarang bisa jalan di **hardware AMR QR**. Yang diganti cuma layer hardware.
Semua node di atas `/cmd_wheel_vel` / `/wheel_states` / `/imu/data_raw` /
`/amr/panel_state` tetap sama.

Kendaraan dipilih lewat `AGV_PROFILE`. `agv-01` (gvievo-01) tidak berubah perilakunya.

Instalasi langkah demi langkah: [INSTALL_AMR_QR.md](INSTALL_AMR_QR.md). Operasi harian (map, route, run): [OPERATE_AMR_QR.md](OPERATE_AMR_QR.md).

---

## 1. Apa yang berubah

| Area | gvievo-01 (`agv-01`) | AMR QR (`amr-qr-01`) |
|---|---|---|
| Node hardware | `drive_node` (CAN) + `panel_node` (DIO) | **`qr_base_node`**: satu owner untuk semuanya |
| Motor | BLV-R CiA 402, velocity loop di driver | Driver BLDC analog: **CKDA08ETH** 0–10 V + coil FWD/REV/BRK di **CK5162E**. Velocity loop **PI + feedforward di software** (50 Hz) |
| Encoder | 6064h di drive (sisi motor, 1:30) | **Encoder CANopen absolut di as roda** (node 1/2, 0x6004, 24-bit, 8192/putaran), TPDO 20 ms atau SDO poll |
| IMU | MLS di CAN | **WitMotion serial** (`/dev/ttyUSB0`). Yaw rate dari paket 0x52, fallback turunan sudut 0x53 |
| Panel | Selector AUTO/MANUAL fisik + Start + Reset | **Selector virtual** dari START/STOP + E-stop DI0 |
| Lidar | nanoScan3, `sick_safetyscanners2` | sama (config `nanoscan3.amr-qr-01.yaml`, **IP perlu diverifikasi**) |
| Geometri | track 0.487, lidar 0.964 m | track **0.37652**, lidar **0.81 m**, body **1.18 × 0.675**, IMU di pusat as |
| Distro | Humble | **Jazzy** (Ubuntu 24.04). Script deploy auto-deteksi |

### File baru / diubah

| File | Isi |
|---|---|
| `profiles/amr-qr-01.json` | Profil kendaraan, termasuk section baru `qr_base` |
| `config.py` | Key `platform` (`blvr_canopen` \| `qr_analog`), schema + validasi `qr_base`, tuning notes |
| `amr_base/qr_base_node.py` | Node ROS: interface identik dengan drive_node + panel_node |
| `amr_base/qr_motor.py` | Wheel loop: interlock arah, FF+PI, brake, latch runaway/stall/moving-at-rest, arm/fault/E-stop |
| `amr_base/qr_encoders.py` | Unwrap 24-bit, estimasi kecepatan, TPDO/SDO |
| `amr_base/wit_imu.py` | Parser WitMotion + sumber yaw rate |
| `amr_base/qr_panel.py` | Selector virtual (debounce, anti-tie-down) |
| `amr_base/qr_analog.py` | Penulis register CKDA08ETH |
| `amr_base/qr_calibrate.py` | Tool bench: `io`, `enc`, `imu`, `breakaway`, `ff` |
| `amr_description/config/vehicle.amr-qr-01.yaml`, `footprint.amr-qr-01.yaml`, `amr_description/vehicle.py` | Geometri per kendaraan (URDF, footprint) |
| `amr_bringup/config/nanoscan3.amr-qr-01.yaml` | Scanner AMR QR |
| `base.launch.py`, `scanner.launch.py`, `description.launch.py`, `navigation_layer.launch.py` | Pilih hardware/geometri sesuai profil. Plugin Spin mengikuti `ROS_DISTRO` |
| `deploy/*.sh`, `amr.env`, `env/vehicle.sh`, `deploy/qr/*` | Deteksi Jazzy/Humble, `AGV_PROFILE`, slcan `can0`, udev |
| `main.py`, `legacy_guard.py`, `ownerlock.py` | Controller lama menolak jalan di profil QR. Node menolak jalan kalau `app.py` QR masih aktif. Tambah lock `imu` |

---

## 2. Instalasi (Ubuntu 24.04 + ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-navigation2 ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-sick-safetyscanners2 ros-jazzy-foxglove-bridge \
  ros-jazzy-diagnostic-updater ros-jazzy-teleop-twist-keyboard \
  python3-can python3-serial python3-flask python3-yaml python3-numpy can-utils
# pymodbus: kode memakai keyword device_id= (pymodbus >= 3.10). Versi apt Ubuntu 24.04
# terlalu lama -> pakai pip (atau venv):
pip install --break-system-packages "pymodbus>=3.10"

echo 'export AGV_PROFILE=amr-qr-01' >> ~/.bashrc      # setiap shell di PC AMR QR
# deploy/amr.env: ganti AGV_PROFILE=agv-01 -> amr-qr-01 (untuk amr.service)
cd ~/agv_can/amr_ws && colcon build --symlink-install && source install/setup.bash
```

**CAN encoder.** Pasang `deploy/qr/99-amr-qr.rules` (nama tetap `/dev/amr_imu`,
`/dev/amr_can`) dan `deploy/qr/slcan-can0.service` (slcand 125 kbit/s → `can0`).
`amr.service` di-`BindsTo` ke `can0`. Setelah itu ubah `qr_base.imu_port` ke
`/dev/amr_imu`.

**Unit systemd.** `deploy/amr.service` masih hard-code user/path `gvipc-evo-01`.
Sesuaikan `User`, `Group`, dan path ke user PC AMR QR sebelum menjalankan
`install.sh`.

**Stop controller QR lama.** `app.py` (port 5050) memiliki modul I/O, bus encoder,
dan port IMU yang sama. `qr_base_node` menolak start selama `app.py` masih jalan.

---

## 3. Commissioning (urutan wajib)

Nilai bertanda **MEASURE/VERIFY** di profil dan file geometri masih asumsi.

| # | Langkah | Kondisi | Lulus bila |
|---|---|---|---|
| 1 | `ros2 run amr_base qr_calibrate io` | parkir | E-stop DI0 (NC), START DI1, STOP DI2 terbaca benar |
| 2 | `ros2 run amr_base qr_calibrate enc`, lalu dorong maju | parkir, bebas | Kedua kecepatan **positif**. Kalau negatif, flip `qr_base.enc_invert_<sisi>` |
| 3 | `ros2 run amr_base qr_calibrate imu`, putar CCW | parkir | Ada 0x52, rate ≥ 40 Hz, wz **positif** (kalau tidak, `qr_base.imu_gyro_sign: -1`). Kalau hanya 0x53: set unit ke 115200 baud / 50–100 Hz + output gyro dengan software WitMotion, lalu update `imu_baud` |
| 4 | **Konfigurasi safe state komunikasi-hilang** CK5162E/CKDA08ETH (kalau modul mendukung), dan pastikan **E-stop hard-wired** memutus driver | — | Cabut LAN modul → motor berhenti |
| 5 | `qr_calibrate breakaway --go` lalu `qr_calibrate ff --go` | **di atas blok** | Tool mencetak `ff_motor_rpm_per_volt` dan `ff_offset_v`. Masukkan ke profil, lalu naikkan `v_max_v` / `vehicle.motor_max_rpm` sesuai batas yang dicetak |
| 6 | `ros2 launch amr_bringup drivers.launch.py lidar:=false` + teleop 0.1 m/s | **di atas blok** | Arah roda benar. Tanda salah → FAULT "turning AGAINST the command" dalam < 1 s |
| 7 | Lidar: IP, window sudut, `laser_yaw`, `laser_x` (tes kotak 2 m depan / 1.2 m kiri, lihat README "Lidar commissioning") | parkir | +x di depan, +y di kiri, jarak ±1 cm |
| 8 | Ukur overhang depan/belakang → `footprint.amr-qr-01.yaml` + `chassis_x` | — | `test_vehicle.py` lulus (keduanya konsisten) |
| 9 | Ground test: lurus 1 m ×2, mundur 1 m, CCW/CW 90° dan 180° (seperti tabel gvievo 2026-09-16) | lantai | Odometri vs pita ≤ 1 %. Gyro vs roda setuju ≤ 1 %. Kalau tidak, koreksi track/`enc_counts_per_rev` |
| 10 | `amr.service` → survey → route → run (RUNBOOK §2) | lantai | — |

Top speed sebelum kalibrasi: **0.28 m/s** (`motor_max_rpm` 900 dengan asumsi 300 rpm/V,
`v_max_v` 4 V). Route yang meminta lebih cepat tetap jalan, tapi di-clamp oleh mux.

---

## 4. Operasi panel (selector virtual)

| Tombol | Saat MANUAL | Saat AUTO |
|---|---|---|
| **START** (DI1) | → pindah ke **AUTO** (tidak memulai apa pun) | **Start edge**: executor mulai / resume |
| **STOP** (DI2) | **Reset edge**: acknowledge FAULT executor | → kembali **MANUAL** (route berjalan di-abort) |
| START + STOP bersamaan | STOP menang | STOP menang |
| **E-stop** (DI0 NC) | Motor 0 V + rem, drive tidak operational, mode tidak berubah | sama. Executor BLOCKED, lanjut dengan Start setelah E-stop dilepas |

- Tombol yang sudah ditahan saat boot atau saat komunikasi DIO pulih **tidak** dihitung
  sebagai tekan (anti-tie-down).
- Kalau DIO hilang, panel dianggap invalid, mode jatuh ke MANUAL, dan motor berhenti
  dengan rem.
- `qr_base.auto_hold_s > 0` (misal 1.5): START harus **ditahan** untuk masuk AUTO,
  sedangkan tekan singkat di MANUAL menjadi Start edge. Mode ini diperlukan untuk
  blind-run di halaman **Commission**. Dengan nilai default 0, blind-run tidak bisa
  di-start dari panel ini.

---

## 5. Keterbatasan & risiko yang perlu diketahui

1. **Modul analog menahan tegangan terakhir** kalau PC/proses mati mendadak (`kill -9`,
   crash kernel). Coil FWD/REV adalah klaim yang kedaluwarsa (0.15 s) selama proses
   masih hidup. Tapi kalau prosesnya sendiri mati, coil juga tertahan di modul, sama
   seperti controller QR lama. Tidak ada padanan heartbeat 1016h milik BLV-R.
   Mitigasinya adalah langkah 4 di atas: safe state modul dan E-stop hard-wired.
2. **E-stop DI0 dibaca software.** Jalur berhenti yang sah harus hard-wired ke driver.
3. **Stack ini tidak ada di jalur safety** (sama seperti README repo). Field nanoScan3 →
   OSSD → relay/driver tetap wajib di hardware.
4. **IMU WitMotion** kelas konsumen. Bias dihapus oleh `imu_bias_node`. `gyro_var`
   masih placeholder (1e-5 (rad/s)²) sampai diukur saat parkir.
5. **Encoder dibaca SDO** kalau TPDO tidak aktif (mode `auto` fallback). Jitter
   timestamp lebih besar, tapi tetap jalan.
6. **Profile position (pp)** dan **monitor BLV-R** tidak tersedia di platform ini
   (divalidasi di loader).
7. **GLS621** tidak dipakai di stack ini (sesuai keputusan). Hardware-nya tetap
   terpasang.
8. Driver analog tidak mengirim alarm. Driver yang trip atau di-disable dari luar
   (alarm, E-stop/OSSD scanner yang di-wire ke enable driver, power motor mati)
   terlihat sebagai **stall**: roda diperintah jalan tapi diam 2 s → FAULT. Selama 2 s
   itu PI terus menaikkan tegangan (integral dibatasi `i_max_v`), jadi kalau driver
   hidup lagi sebelum FAULT, roda bisa tersentak. Kalau sinyal stop seperti itu ada,
   sambungkan ke DI kosong. Stack gvievo menangani "lidar stop → lanjut otomatis"
   lewat feedback torsi driver, dan fitur itu **tidak** tersedia di AMR QR.

---

## 6. Hasil test offline (tanpa hardware, tanpa ROS)

| Suite | Hasil |
|---|---|
| `tests/run_all.py` | 923 check. 1 FAIL **sudah ada sebelum perubahan ini**: `pp enabled with unset drive values is refused` (profil agv-01 sudah mengisi semua `pp.expect`, jadi premis test itu basi) |
| `amr_base/test/test_qr_*.py` | 33 lulus: encoder (wrap 24-bit, TPDO/SDO), IMU (checksum, wrap, batch), wheel loop (interlock, anti-windup, konvergensi plant, **negative control**: tanda encoder salah → runaway), panel, analog, dan **qr_base_node end-to-end dengan hardware simulasi** |
| `amr_description/test/test_vehicle.py` | 5 lulus: URDF ↔ profil ↔ footprint konsisten |
| Test amr_ws lain | Sama dengan baseline. Yang gagal hanya karena tidak ada `rclpy`/`xacro` di mesin test |
| `ruff check src/` | bersih |

Yang **belum** teruji: build `colcon` dan launch di Jazzy sungguhan, dan semua perilaku
di hardware. Langkah §3 adalah tempat itu diuji.
