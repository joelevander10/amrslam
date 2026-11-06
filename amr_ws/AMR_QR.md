# AMR QR di stack gvievo-01 — profil `amr-qr-01`

Repo ini = **kode gvievo-01 terbaru (gvievo-01-kyb-trackless, 2026-11-03)** + platform AMR QR.
Supervisor, slam_toolbox, AMCL, Nav2 RPP/Spin, route executor, web `:5001` (view operator /
engineer, layout baru: Home menggantikan Status), model daya drive (standby/armed),
pre-move warning, **wait step + Continue**, step **Go home**, perubahan kecepatan
**S-curve**, tool **Measure** di editor route, dan semua perbaikan gvievo ikut. Mode tape
(LINE), RFID dan profile position sudah dihapus dari kode (KYB trackless). Yang diganti hanya layer hardware: semua node di atas
`/cmd_wheel_vel` / `/wheel_states` / `/imu/data_raw` / `/amr/panel_state` /
`/drives/status` tetap sama.

Kendaraan dipilih lewat `AGV_PROFILE` (`amr.env` dan `env/vehicle.sh` di repo ini default
`amr-qr-01`). `agv-01` (gvievo-01) tetap bisa dipakai dan perilakunya tidak berubah.

Instalasi langkah demi langkah (dan update dari kode lama, Part U):
[INSTALL_AMR_QR.md](INSTALL_AMR_QR.md). Operasi harian: [OPERATE_AMR_QR.md](OPERATE_AMR_QR.md).

---

## 1. Apa yang berbeda dari gvievo-01

| Area | gvievo-01 (`agv-01`) | AMR QR (`amr-qr-01`) |
|---|---|---|
| Produk | Trackless (SLAM) saja | Sama |
| Node hardware | `drive_node` (CAN) + `panel_node` (DIO) + `gls_node` | **`qr_base_node`**: satu owner untuk semuanya |
| Motor | BLV-R CiA 402, velocity loop di driver | Driver BLDC analog: **CKDA08ETH** 0–10 V + coil FWD/REV/BRK di **CK5162E**. Feedforward terukur (970 rpm/V) + trim PI pelan di software, coast saat perintah ~0 |
| Daya drive | Standby/armed lewat `arm_policy`, Manual Arm DI08 | Sama (`arm_policy`), dibaca QR: MANUAL selalu armed (tidak ada saklar Manual Arm); AUTO standby (0 V, **rem ditahan**) sampai run minta daya. `/drives/disarm` = **push mode** (rem dilepas) |
| Encoder | 6064h di drive | **Encoder CANopen absolut di as roda** (node 1/2, 0x6004, 8192/putaran), range wrap dipelajari dari data |
| IMU | MLS di CAN | **WitMotion serial** `/dev/amr_imu`, 115200 baud, paket 0x52 |
| Panel | Selector AUTO/MANUAL fisik + Start + Reset + Continue + 3 tombol tujuan + pendant | **Selector virtual** dari START/STOP + E-stop DI0. Tanpa Reset/Continue/tombol tujuan (`panel.destinations: []`) |
| Emergency | Flexi Soft Q3 (DI08) → STO; setelah lepas E-stop harus **Reset** | E-stop DI0 (`panel.di_estop_ok` = `qr_base.di_estop`). **Tanpa Reset**: lepas E-stop → NORMAL, run yang ditahan lanjut dengan START. Teks web "press Reset" bisa diabaikan |
| Wait step | Lanjut dengan PB Continue (DI13) atau Start | Lanjut dengan **START** atau tombol **Continue** di halaman Run (`web_continue`, hanya untuk run yang WAITING) |
| Kecepatan route | Straight 0.60, arc 0.30 m/s | **0.80 m/s** straight (`qr_base.route_v_mps`), **0.30 m/s** arc / go home (`motor_max_rpm` 3000 = top 0.94 m/s, `v_max_v` 10 V = skala penuh driver; route maks. 95 % top speed). Straight boleh punya speed sendiri yang lebih rendah (`v_mps`, slow zone / speed waypoint). RPP `desired_linear_vel` 0.80, approach ramp RPP hanya 0.3 m (harus < setengah local costmap 2.5 m, kalau tidak semua gerakan tertahan di ~48 %); pengereman di akhir gerakan oleh executor (`end_decel` 0.2). Akselerasi AUTO QR 0.2 / 0.25 m/s², jerk 0.4 (`route.QR_AUTO_*`) |
| Kamera GLS621 | Dipakai (read-only, rekam label lantai) | **Tidak terpasang**: `gls.enabled: false`, node tidak jalan, bagian label di halaman Run disembunyikan |
| Field scanner | Mux + executor berhenti dari `/output_paths` | **Sama** sejak lidar diganti (2026-11-02). Field set di scanner harus cocok dengan `scanner_fields.yaml` (cek §5 no. 9) |
| Filter lidar | — | **Tidak ada** (sama dengan gvievo). Noise 2026-09/10 ternyata dari lidar yang rusak; filter-nya dimatikan, kodenya masih ada |
| Geometri | track 0.487, lidar 0.964 m | track **0.37652**, lidar **0.81 m**, body **1.18 × 0.675** |
| Distro | Humble (Ubuntu 22.04) | **Jazzy** (Ubuntu 24.04). Script deploy auto-deteksi; slam_toolbox lifecycle & plugin Spin mengikuti distro |

### File AMR QR di repo ini

| File | Isi |
|---|---|
| `profiles/amr-qr-01.json` | Profil kendaraan: `platform: qr_analog`, `tracked: false`, section `qr_base` |
| `agv_core/config.py` | Key `platform`, schema + validasi `qr_base`, tuning notes `platform.*` / `qr_base.*` |
| `agv_core/ownerlock.py` | Lock `imu` |
| `amr_base/qr_base_node.py` | Node ROS: interface sama dengan drive_node + panel_node (termasuk `link_state`, `power_wanted`, `manual_arm`, topik `_ui`) |
| `amr_base/qr_motor.py` | Wheel loop: interlock arah, FF + PI, coast, rem, latch runaway/stall, standby/armed/push |
| `amr_base/qr_encoders.py`, `wit_imu.py`, `qr_panel.py`, `qr_analog.py` | Encoder, IMU, selector virtual, output analog |
| `amr_base/qr_keypad.py` | Keypad USB (evdev, hot-plug, grab): pendant kedua + tombol START/STOP/Q/E; `qr_keypad --list/--watch` |
| `amr_base/qr_calibrate.py`, `wasd_teleop.py` | Tool bench (`io`, `enc`, `imu`, `imu-setup`, `breakaway`, `ff`, `brake-off`) dan teleop WASD |
| `amr_localization/speckle.py`, `scan_mask*.py`, `robot_fixed.py` | Filter scan di `scan_gate_node` (mati; untuk lidar rusak) |
| `amr_description/vehicle.py`, `config/*.amr-qr-01.yaml` | Geometri per kendaraan (URDF, footprint) |
| `amr_bringup/config/nanoscan3.amr-qr-01.yaml` | Scanner 192.168.3.2 (subnet sendiri; PC 192.168.3.10 di enp1s0, host_ip auto) |
| `base/scanner/mapping_layer/navigation_layer.launch.py` | Pilih hardware & config sesuai profil, lifecycle slam_toolbox di Jazzy, start gate 0.20 m / 10° |
| `amr_web`: halaman Manual (push mode), Maps (cek Map id), log alasan run, live view `/scan_gated` | Web |
| `deploy/*` (user `amrslam`, `AGV_PROFILE=amr-qr-01`), `deploy/qr/*` (udev, slcan `can0`) | Deploy |

---
## 2. Commissioning (urutan wajib)

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

Top speed: **0.28 m/s** (`motor_max_rpm` 900). Feedforward sudah dari pengukuran roda kiri
(970 rpm/V, offset 0.25 V, `v_max_v` 2 V); ulangi `ff --wheel both` untuk roda kanan.
Route yang meminta lebih cepat tetap jalan, tapi di-clamp oleh mux.

---

## 3. Operasi panel (selector virtual)

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
- **Keypad USB** (`amr_base/qr_keypad.py`, map tombol sama dengan KIM2A): W/A/S/D dan panah
  = pendant kedua (tahan untuk jalan, hanya MANUAL), **G** = START, **ENTER** = STOP,
  **Q** = ganti MANUAL/AUTO, **E** = E-stop keypad (latched, dilepas dengan ENTER; software,
  bukan safety). Masuk ke image panel di `qr_base_node`, jadi mux/executor tidak berubah.
  Setting `qr_base.keypad_*`; butuh `python3-evdev` dan grup `input`
  (`SupplementaryGroups=input` di `amr.service`).
- `qr_base.auto_hold_s > 0` (misal 1.5): START harus **ditahan** untuk masuk AUTO,
  sedangkan tekan singkat di MANUAL menjadi Start edge. Mode ini diperlukan untuk
  blind-run di halaman **Commission**. Dengan nilai default 0, blind-run tidak bisa
  di-start dari panel ini.

---

## 4. Keterbatasan & risiko yang perlu diketahui

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
6. **Monitor BLV-R** tidak tersedia di platform ini (divalidasi di loader). Profile
   position sudah dihapus dari kode di KYB trackless.
7. **GLS621** tidak terpasang (`gls.enabled: false`). Kalau nanti dipasang: set
   `gls.enabled: true`, kalibrasi `gls.*`, dan tambahkan `gls_node` ke cabang QR di
   `base.launch.py` (sekarang hanya cabang drive_node yang menjalankannya).
8. Driver analog tidak mengirim alarm. Driver yang trip atau di-disable dari luar
   (alarm, E-stop/OSSD scanner yang di-wire ke enable driver, power motor mati)
   terlihat sebagai **stall**: roda diperintah jalan tapi diam 2 s → FAULT. Selama 2 s
   itu PI terus menaikkan tegangan (integral dibatasi `i_max_v`), jadi kalau driver
   hidup lagi sebelum FAULT, roda bisa tersentak. Kalau sinyal stop seperti itu ada,
   sambungkan ke DI kosong. Stack gvievo menangani "lidar stop → lanjut otomatis"
   lewat feedback torsi driver, dan fitur itu **tidak** tersedia di AMR QR.
9. **Field scanner dipakai** sejak lidar yang rusak diganti, dengan field set AMR QR
   sendiri: `scanner_fields.amr-qr-01.yaml` (gvievo tetap `scanner_fields.yaml`). Satu
   warning field di output path 0 (LED kuning → AUTO setengah kecepatan, "Warning 1") dan
   protective di path 1 (LED merah → berhenti). Diukur 2026-11-05: hijau `[true, true]`,
   kuning `[false, true]`, merah `[false, false]`. Executor juga membaca protective dari
   path 1 (`field_output_index`). Kalau field di Safety Designer diubah, ukur ulang.

---

## 5. Test offline

| Suite | Hasil (2026-11-04) |
|---|---|
| `tests/run_all.py` | 763 check, semua lulus (731 KYB trackless + 32 profil QR). Runner mem-pin `AGV_PROFILE=agv-01` |
| `amr_base/test/test_qr_*.py` | 48 lulus, plus `qr_base_node` end-to-end dengan hardware simulasi: standby di AUTO, armed saat run minta daya, push mode, emergency ACTIVE/NORMAL tanpa Reset |
| `amr_mission/test/test_wait_step.py` | lulus, termasuk Continue dari web (hanya WAITING, ditolak saat E-stop / di luar titik wait) |
| `amr_mission/test/test_rejoin.py` (2026-11-05) | 13 lulus: lanjut route dari titik berhenti setelah Abort/FAULT (straight, arc, sisa putaran, wait, step yang sudah selesai → step berikut, file `route_checkpoint.json` tahan restart, ditolak kalau route/map berubah) |
| `amr_web/test` | lulus: tombol Continue hanya di QR, bagian GLS tersembunyi tanpa kamera, push mode, edit peta |
| Test paket lain | Sama persis dengan baseline KYB trackless di mesin tanpa ROS |

Yang **belum** teruji: build `colcon` dan launch di Jazzy sungguhan, dan semua perilaku
di hardware. Ulangi langkah §2 setelah update (INSTALL Part U).
