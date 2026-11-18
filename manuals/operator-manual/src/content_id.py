"""Manual AMR, Bahasa Indonesia. Nama tombol, halaman dan kode tetap seperti di layar."""

META = {
    "lang": "id",
    "title": "AMR New Gen (AMR QR-01)",
    "subtitle": "Buku Panduan Operasi, Perawatan dan Troubleshooting",
    "edition": "Edisi 1.1 · November 2026 · software: AMR SLAM New Gen (profil amr-qr-01)",
    "fill_title": "Isi untuk kendaraan ini",
    "fill": [("Alamat halaman web", "http://______________________:5001"), ("Jaringan Wi-Fi", ""),
             ("Engineer / kontak", ""), ("Telepon", ""), ("Peta yang dipakai (map id, revisi)", "")],
    "contents": "Daftar isi",
    "part": "Bagian",
    "page": "hal.",
    "footer": "AMR New Gen - Panduan Operasi, Perawatan dan Troubleshooting",
    "chapter": "Bab",
    "who_title": "Siapa membaca apa",
    "who": [("Bagian 1 - Operasi", "Semua orang yang bekerja dengan AMR"), ("Bagian 2 - Perawatan dan troubleshooting", "Teknisi, kepala shift"),
            ("Bagian 3 - Tugas engineer", "Engineer (perlu PIN engineer)")],
}

B = []
A = B.append

# ---------------------------------------------------------------- pendahuluan
A(("h1", "Keselamatan dulu", False))
A(("p", "AMR (autonomous mobile robot) berjalan sendiri mengikuti route yang dibuat di atas peta. AMR melihat sekelilingnya dengan "
        "laser scanner safety di depan, lalu melambat atau berhenti kalau ada orang atau benda. Walau begitu, AMR tetap mesin yang berat. "
        "Baca halaman ini sebelum bekerja dengan AMR."))
A(("warn", "<b>Tombol E-stop merah adalah stop yang sesungguhnya.</b> Ketahui letaknya sebelum AMR bergerak. Menekannya langsung "
           "memutus motor dan mengaktifkan rem."))
A(("bul", [
    "Jangan berada di jalur AMR yang sedang bergerak, dan jangan pernah menaikinya.",
    "Scanner hanya melihat ke <b>depan</b>. Saat AMR mundur (langkah mundur pendek di beberapa route), AMR tidak melihat ke belakang.",
    "Pada 0,7 m/s AMR butuh sekitar <b>0,6 m untuk berhenti</b>. Jangan melangkah ke depannya di saat terakhir.",
    "AMR yang berhenti bisa <b>jalan lagi sendiri</b>: setelah benda keluar dari field scanner, setelah AMR menemukan posisinya lagi, "
    "atau setelah peringatan 2 detik waktu seseorang menekan START. AMR tidak punya klakson - perhatikan AMR.",
    "Sebelum tangan mendekati roda, tekan E-stop.",
    "<b>Push mode</b> (rem dilepas) hanya di lantai datar: AMR menggelinding bebas.",
    "Jangan memutus power utama saat AMR berjalan (lihat Bab 8, Akhir shift).",
    "Hanya orang yang sudah dilatih yang boleh mengubah peta, route atau setting (Bagian 3).",
]))
A(("h2", "Istilah di buku ini"))
A(("table", ["Istilah", "Arti"], [
    ["Peta (map)", "Gambar lantai milik AMR (dinding, mesin, rak), dibuat dengan menjalankan AMR mengelilingi area sekali. Setiap perubahan membuat <b>revisi</b> baru (rev 1, rev 2, ...)."],
    ["Route", "Jalur yang digambar di peta: lurus, belokan, putar di tempat, titik tunggu."],
    ["Mission (misi)", "Route yang siap dijalankan di satu revisi peta. Operator memuat dan menjalankan misi."],
    ["Homepost", "Titik awal route, ditandai di lantai. AMR mulai dan berakhir di sini."],
    ["POS 1-4", "Empat titik berhenti (stasiun). Tiap route POS berjalan dari homepost ke titik berhentinya lalu kembali."],
    ["Lokalisasi", "AMR mengetahui posisinya di peta. Harus <b>READY</b> sebelum misi bisa dijalankan."],
    ["MANUAL / AUTO", "MANUAL: AMR dijalankan dengan tangan. AUTO: misi yang menjalankan AMR."],
    ["E-stop", "Tombol berhenti darurat (merah)."],
], [0.22, 0.78]))

# ---------------------------------------------------------------- BAGIAN 1
A(("part", "1", "Operasi", "Untuk semua orang yang bekerja dengan AMR"))

A(("h1", "Mengenal AMR", True))
A(("fig", "overview", "Bagian utama (skema tampak atas). E-stop ada di depan; letak tombol lain dan caster hanya ilustrasi.", 0.95))
A(("table", ["", ""], [
    ["Ukuran", "Panjang 1,18 m × lebar 0,675 m"],
    ["Penggerak", "Dua roda penggerak (kiri dan kanan) dengan encoder; caster menahan sisanya"],
    ["Kecepatan saat misi", "0,70 m/s di lurusan, 0,40 m/s di belokan (tiap belokan bisa punya kecepatan sendiri), lebih pelan di slow zone"],
    ["Kecepatan manual", "0,10-0,40 m/s dari halaman web, 0,50 m/s dari keypad USB"],
    ["Sensor", "Laser scanner safety SICK nanoScan3 (depan), encoder roda, giroskop (IMU)"],
    ["Kontrol", "PC industri; halaman web di tablet, HP atau laptop lewat Wi-Fi; tombol START / STOP; E-stop; keypad USB"],
    ["Navigasi", "Peta laser (SLAM). Tidak perlu pita atau kode QR di lantai"],
], [0.28, 0.72], False))

A(("h1", "Tombol, keypad dan halaman web", True))
A(("fig", "panel", "Tombol panel dan keypad USB.", 1.0))
A(("h2", "Tombol panel"))
A(("table", ["Tombol", "Saat MANUAL", "Saat AUTO"], [
    ["<b>START</b> (hijau)", "Pindah ke AUTO. Tidak ada yang bergerak. Misi terakhir dimuat otomatis.", "Menjalankan misi yang dimuat, atau melanjutkan misi yang berhenti."],
    ["<b>STOP</b>", "Acknowledge (reset) fault misi.", "Kembali ke MANUAL. Misi yang berjalan dibatalkan."],
    ["<b>E-stop</b> (merah)", "Motor mati, rem aktif.", "Motor mati, rem aktif. Misi ditahan (BLOCKED). Lepas E-stop, lalu tekan START untuk melanjutkan."],
], [0.18, 0.37, 0.45]))
A(("note", "<b>Setelah E-stop tidak ada tombol reset di AMR ini.</b> Melepas (memutar) E-stop sudah cukup. Beberapa teks di layar "
           "berbunyi \"press the blue Reset button on the cabinet\" - teks itu untuk kendaraan lain; abaikan. Saat misi, tekan START untuk "
           "melanjutkan."))
A(("bul", [
    "Setelah dinyalakan, AMR selalu dalam mode <b>MANUAL</b>.",
    "Tombol yang sudah ditekan waktu power dinyalakan tidak berfungsi sampai dilepas dan ditekan lagi.",
    "Halaman web tidak pernah menjalankan misi sendiri. Gerakan butuh START di panel/keypad, tombol POS, atau tombol jog yang ditahan.",
]))
A(("h2", "Keypad USB"))
A(("p", "Keypad wireless bekerja seperti panel. Dongle-nya dicolok ke PC AMR dan terdeteksi otomatis."))
A(("table", ["Tombol", "Fungsi"], [
    ["W / ↑, S / ↓", "Maju / mundur selama ditahan (hanya MANUAL)"],
    ["A / ←, D / →", "Putar di tempat kiri / kanan selama ditahan (hanya MANUAL). W+A, W+D ... untuk belok"],
    ["G", "= tombol START"],
    ["ENTER", "= tombol STOP"],
    ["Q", "Ganti MANUAL ⇄ AUTO"],
    ["E", "Stop keypad (terkunci). ENTER melepasnya. <b>Ini stop software, bukan E-stop safety.</b>"],
    ["F1 ... F4", "Ke POS 1 ... POS 4 (saat AUTO, AMR di homepost, tidak ada misi berjalan). F5 dan F6 tidak berfungsi"],
], [0.2, 0.8]))
A(("h2", "Halaman web"))
A(("p", "Buka alamat yang tertulis di sampul (port 5001) di browser tablet, HP atau laptop yang tersambung ke Wi-Fi AMR. "
        "Tampilan operator punya empat halaman:"))
A(("table", ["Halaman", "Untuk apa"], [
    ["<b>Home</b>", "Satu baris kondisi AMR, kecepatan, dan satu kalimat apa yang harus dilakukan. Pause, Abort, Return to idle, Recover."],
    ["<b>Run</b>", "Pilih peta, set posisi AMR, muat misi, tombol POS, Continue, Pause, Abort, Ack fault. Peta mengikuti AMR."],
    ["<b>Manual</b>", "Jalan manual dengan jog pad; push mode (rem off / on)."],
    ["<b>Alarms</b>", "Apa yang menahan kendaraan sekarang (\"Standing now\"), riwayat kejadian, kehilangan lokalisasi, Save report."],
], [0.18, 0.82]))
A(("fig", "webpage", "Tata letak halaman Run.", 1.0))
A(("h2", "Tile status (sisi kiri setiap halaman)"))
A(("table", ["Tile", "Normal", "Arti / kalau tidak normal"], [
    ["Mode", "IDLE (tanpa peta), NAVIGATION (peta dipakai), MAPPING", "FAULT atau banner merah: lihat halaman Alarms"],
    ["Selector", "MANUAL atau AUTO", "STALE / INVALID: modul I/O panel tidak menjawab (Bab 10)"],
    ["Drives", "ARMED saat MANUAL dan saat misi berjalan; OFF saat AUTO tanpa misi (standby, rem ditahan)",
     "OFF \"push mode\": rem dilepas - halaman Manual, Brakes ON. Merah: E-stop atau fault driver"],
    ["Command", "none, atau siapa yang menjalankan (manual, follow ...)", ""],
    ["Scanner", "CLEAR", "WARNING (setengah kecepatan) atau PROTECT (berhenti): ada benda di depan"],
    ["Localis.", "READY", "CHECKING (menunggu Confirm), LOST (posisi hilang)"],
    ["Run", "READY, EXECUTING, DONE", "BLOCKED, WAITING, FAULT: lihat Bab 7"],
    ["Wheels", "Kecepatan perintah kiri / kanan", ""],
], [0.14, 0.4, 0.46]))
A(("fig", "modes", "Mode kendaraan (kiri) dan status misi (kanan).", 1.0))

A(("h1", "Awal shift", True))
A(("fig", "startflow", "Dari menyalakan sampai misi pertama.", 1.0))
A(("steps", [
    "Periksa sekeliling AMR: tidak ada barang di atas yang bisa jatuh, tidak ada apa-apa di bawah roda, jendela scanner bersih, area aman.",
    "Lepas E-stop. Nyalakan AMR. Tunggu sekitar satu menit sampai software siap.",
    "Buka halaman web. <b>Home</b> harus menunjukkan: Mode <b>IDLE</b> atau NAVIGATION, tidak ada banner merah, Selector <b>MANUAL</b>, Drives <b>ARMED</b>.",
    "Jangan sentuh atau dorong AMR beberapa detik setelah menyala: giroskop mengkalibrasi diri selama AMR diam.",
    "Kalau halaman Home menulis <b>\"The vehicle lost power last time\"</b> (UNCLEAN_SHUTDOWN): matikan lalu nyalakan lagi power driver motor "
    "(tanya teknisi di mana saklarnya) sebelum misi pertama.",
    "Lanjut ke Bab 5 (posisi) dan Bab 6 (menjalankan misi).",
]))
A(("tip", "Cek harian 1 menit: E-stop menghentikan motor; START pindah ke AUTO dan STOP kembali ke MANUAL; LED scanner hijau kalau "
          "tidak ada apa-apa di depan. Catat masalah di logbook."))

A(("h1", "Menjalankan secara manual", True))
A(("bul", [
    "AMR harus dalam mode <b>MANUAL</b> (tekan STOP kalau tile Selector menunjukkan AUTO).",
    "<b>Halaman web</b> (halaman Manual, Run atau Maps): tahan tombol jog pad atau tombol W/A/S/D / panah. Dilepas = AMR berhenti. "
    "Pilih 0,10-0,40 m/s. Pindah tab browser juga menghentikan AMR.",
    "<b>Keypad USB</b>: tahan W/A/S/D atau panah; 0,50 m/s. Lepas semua tombol untuk berhenti.",
    "Field scanner tidak memperlambat jalan manual, tetapi output safety scanner tetap menghentikan motor. Kalau tombol terus ditahan "
    "saat AMR dihentikan, roda melaporkan \"stalled\" setelah 2 detik - lepas tombol dan panggil teknisi untuk menghapusnya.",
]))
A(("h2", "Push mode (mendorong AMR dengan rem dilepas)"))
A(("steps", [
    "MANUAL, tidak ada misi berjalan. Di halaman <b>Manual</b> tekan <b>Brakes OFF (push)</b>. Tile Drives menunjukkan OFF · push mode.",
    "Dorong AMR pelan-pelan. Hanya di lantai datar - AMR menggelinding bebas.",
    "Tekan <b>Brakes ON (drive)</b> setelah selesai. AMR tidak bisa dijalankan sebelum itu.",
]))

A(("h1", "Memberi tahu posisi AMR (lokalisasi)", True))
A(("p", "Setelah AMR dinyalakan, atau setelah posisinya hilang, AMR harus diberi tahu posisinya di peta. Misi hanya bisa mulai kalau "
        "Lokalisasi <b>READY</b>."))
A(("steps", [
    "Halaman <b>Run</b>, tab <b>Map & pose</b>. Kalau tile Active map bukan peta yang dipakai: pilih di <b>View / activate</b> lalu tekan "
    "<b>Use this map on the vehicle</b> (hanya saat AMR diam). Mode menjadi NAVIGATION.",
    "Cara termudah: parkir AMR tepat di tanda homepost di lantai, menghadap arah yang sama dengan tanda, lalu tekan <b>Initial pose = route start</b> "
    "(untuk misi yang dipilih di tab Mission) atau <b>Initial pose = survey mark</b>.",
    "Cara lain: tekan <b>Set initial pose on map</b>, tekan di peta posisi AMR yang sebenarnya, tarik ke arah hadap AMR, lepas.",
    "Lokalisasi menunjukkan CHECKING. Lihat peta: titik merah scanner harus tepat di dinding hitam (gambar di bawah). Kalau tidak, set posisi lagi.",
    "Kalau <b>Can confirm</b> menunjukkan YES dan titik-titik tepat di dinding, tekan <b>Confirm: scans align</b>. Lokalisasi menjadi <b>READY</b>.",
]))
A(("fig", "localise", "Cek sebelum confirm: titik scanner harus tepat di dinding peta.", 0.95))
A(("tip", "Kalau Can confirm tetap NO, jalankan AMR sekitar setengah meter maju-mundur dan putar sedikit di mode MANUAL - ini membantu AMR menemukan posisinya."))

A(("h1", "Menjalankan misi", True))
A(("h2", "Cara cepat: tombol POS"))
A(("steps", [
    "Lokalisasi READY, AMR di homepost (dalam 20 cm dan 10°), area aman.",
    "Tekan START sekali (atau Q di keypad): Selector menunjukkan AUTO.",
    "Tekan <b>Go to POS n</b> di halaman Run (tab Mission) atau F1-F4 di keypad. Setelah peringatan 2 detik AMR berjalan ke POS tersebut.",
    "Di POS misi <b>menunggu</b> (status Run WAITING, dengan nama titik tunggunya). Kerjakan tugasnya (muat / bongkar), menjauh, lalu tekan "
    "<b>Continue</b> (halaman Run) atau <b>START</b>. AMR kembali ke homepost.",
]))
A(("h2", "Misi apa saja"))
A(("steps", [
    "Lokalisasi READY. Taruh AMR di homepost route (dalam <b>0,20 m dan 10°</b>).",
    "Hanya pertama kali, atau untuk ganti misi: di tab Mission pilih misi lalu tekan <b>Load (READY)</b>. Setelah itu misi terakhir "
    "dimuat otomatis setiap kali AMR masuk AUTO tanpa misi yang dimuat.",
    "Pastikan area aman dan tetap dekat dengan E-stop.",
    "Tekan <b>START</b>: Selector AUTO; status Run menunjukkan READY (\"loaded by itself ... press START to run it\").",
    "Tekan <b>START</b> lagi: status Run EXECUTING. Selama 2 detik tidak ada yang bergerak (peringatan; driver dinyalakan), lalu AMR berjalan.",
    "Di akhir: status Run <b>DONE</b>. Untuk menjalankan lagi tekan START lagi (masih AUTO).",
]))
A(("fig", "fields", "Saat berjalan, scanner memperlambat dan menghentikan AMR kalau ada sesuatu di depan.", 0.95))
A(("h2", "Selama misi berjalan"))
A(("table", ["Anda ingin ...", "Lakukan"], [
    ["Berhenti sekarang (misi selesai)", "Tekan STOP (AMR ke MANUAL) atau Abort di halaman web"],
    ["Berhenti darurat", "Tekan E-stop. Untuk lanjut: lepas, tekan START"],
    ["Menahan dan lanjut nanti", "Pause (web). Untuk lanjut: Prepare resume, lalu START"],
    ["Lanjut setelah titik tunggu", "Continue (halaman Run) atau START"],
    ["Melanjutkan misi yang di-abort atau fault", "Halaman Run, tab Mission: <b>Continue route from here</b>, lalu START (Bab 7)"],
], [0.4, 0.6]))
A(("note", "Peta di halaman Run menjaga AMR tetap di tengah (FOLLOW menyala). Geser peta untuk melihat tempat lain; tekan FOLLOW untuk kembali."))

A(("h1", "Saat AMR berhenti", True))
A(("p", "Baca <b>status Run</b> dan baris di bawahnya di halaman Run. Halaman Home juga menulis dalam satu kalimat apa yang harus dilakukan."))
A(("table", ["Yang terlihat", "Penyebab", "Lakukan"], [
    ["BLOCKED - E-stop", "E-stop ditekan", "Lepas, tekan START"],
    ["BLOCKED - lidar stop", "Ada sesuatu di field merah scanner", "Bersihkan area. AMR lanjut sendiri"],
    ["BLOCKED - path blocked", "Software melihat benda di jalur", "Singkirkan. AMR mencoba lagi sendiri, selama diperlukan"],
    ["BLOCKED - relocalising", "Posisi hilang dan AMR sedang mencarinya lagi (sampai 3 kali per misi)", "Menjauh. AMR lanjut sendiri dalam beberapa detik"],
    ["WAITING", "Titik tunggu di route", "Kerjakan tugasnya, menjauh, Continue atau START"],
    ["Continue refused", "Bukan AUTO, tidak READY, AMR bergeser dari titik tunggu, atau E-stop", "Baca alasannya, perbaiki, tekan Continue lagi"],
    ["FAULT", "Masalah yang tidak bisa diatasi sendiri (lihat alasannya)", "Perbaiki penyebabnya, tekan <b>Ack fault</b> (atau STOP saat MANUAL), lalu <b>Continue route from here</b> + START, atau mulai misi lagi dari homepost"],
    ["FAULT - posisi tidak ditemukan lagi", "Relokalisasi gagal (sering di tempat lantainya berubah)", "Set posisi manual (Bab 5), Confirm, Ack fault, Continue route from here"],
    ["FAULT - turn centre drifted", "Saat putar di tempat AMR bergeser ke samping (lantai licin, benda di bawah roda, troli tergandeng)", "Cek roda dan lantai; Ack fault; Continue"],
    ["Lokalisasi LOST", "Posisi hilang di luar misi", "Set posisi lagi (Bab 5)"],
    ["Drives tidak ARMED", "E-stop, push mode masih aktif, atau masalah driver / I/O", "Bab 10"],
], [0.24, 0.36, 0.4]))
A(("h2", "Melanjutkan misi setelah Abort atau FAULT"))
A(("steps", [
    "Setelah FAULT: perbaiki penyebabnya dan tekan <b>Ack fault</b>. Setelah restart: aktifkan peta yang sama dan set posisi (Bab 5).",
    "Tab Mission: kotak <b>Stopped route</b> menunjukkan di mana misi berhenti. Tekan <b>Continue route from here</b>. Tidak ada yang bergerak.",
    "Pastikan AMR dekat dengan jalur tempat berhenti (maksimal 0,30 m di samping jalur, arah dalam 20°). Kalau tidak, jalankan ke sana saat MANUAL.",
    "START (AUTO), START lagi: setelah peringatan 2 detik AMR melanjutkan dari langkah itu.",
]))

A(("h1", "Akhir shift", True))
A(("steps", [
    "Biarkan misi selesai (DONE) atau Abort. Tekan <b>STOP</b> sehingga AMR dalam mode MANUAL.",
    "Halaman Run: <b>Return to idle</b>.",
    "Parkir AMR di tempat parkirnya, di lantai datar.",
    "Matikan PC dengan benar <b>sebelum</b> memutus power utama: tekan singkat tombol power PC (jangan ditahan), dan tunggu sampai PC "
    "mati - atau minta engineer. (Pastikan sekali dengan engineer bahwa tekan singkat melakukan ini di kendaraan Anda.)",
    "Matikan power utama. Isi daya atau ganti baterai sesuai prosedur di lokasi Anda.",
]))
A(("warn", "Memutus power saat software berjalan bisa membuat driver motor tetap memegang perintah terakhirnya dan membuat start berikutnya "
           "lebih lama (\"The vehicle lost power last time\" - matikan lalu nyalakan lagi driver)."))

# ---------------------------------------------------------------- BAGIAN 2
A(("part", "2", "Perawatan dan troubleshooting", "Untuk teknisi dan kepala shift"))

A(("h1", "Perawatan", True))
A(("p", "Jadwal di bawah ini adalah titik awal untuk kendaraan ini. Ikuti petunjuk pabrik untuk baterai, driver motor dan scanner "
        "safety kalau petunjuknya lebih lengkap. Catat setiap pengecekan dan perbaikan di logbook."))
A(("table", ["Kapan", "Cek", "Cara / OK kalau"], [
    ["Setiap shift", "E-stop", "Tekan saat jalan manual pelan: AMR langsung berhenti. Dilepas: Drives ARMED lagi"],
    ["Setiap shift", "Jendela scanner", "Bersihkan dengan kain lembut (tanpa pelarut). Scanner tidak boleh menunjukkan peringatan kotor (contamination)"],
    ["Setiap shift", "Roda dan caster", "Tidak ada yang terlilit, tidak ada sobek atau bagian datar, caster berputar bebas"],
    ["Setiap shift", "Tombol dan keypad", "START → AUTO, STOP → MANUAL; keypad bisa menjalankan saat MANUAL"],
    ["Mingguan", "Tes stop scanner", "Saat AUTO pelan, berjalan masuk ke jalur: setengah kecepatan (kuning) lalu berhenti (merah). Harus berhenti sebelum Anda"],
    ["Mingguan", "Kabel dan konektor", "Kabel encoder, CAN, scanner dan I/O kencang, tidak bergesekan; tidak ada colokan longgar di PC"],
    ["Mingguan", "Kopling encoder", "Kopling antara encoder dan roda kencang, tidak ada main (encoder goyang = jarak salah)"],
    ["Mingguan", "Tanda lantai", "Tanda homepost dan POS masih terlihat dan di tempatnya"],
    ["Mingguan", "Halaman Alarms", "Lihat Localisation losses: tempat yang sama berulang-ulang → peta perlu diperbarui (Bagian 3)"],
    ["Bulanan", "Rem", "Saat MANUAL dan diam, coba dorong AMR: rem menahan"],
    ["Bulanan", "Tes jalan lurus", "Jalankan manual 10 m lurus: tidak menarik ke satu sisi (tile Wheels R − L mendekati 0)"],
    ["Bulanan", "Backup peta", "Engineer: push peta ke GitHub (Bab 17)"],
    ["Bulanan", "Ruang disk", "Tidak ada alarm DISK_LOW. Engineer menghapus report / trash lama bila perlu"],
    ["Setelah tabrakan", "Scanner dan dudukannya", "Scanner tidak bergeser atau longgar; lakukan tes stop scanner sebelum AUTO lagi"],
    ["Setelah listrik mati", "Driver", "Matikan lalu nyalakan driver; cek Home tidak menunjukkan UNCLEAN_SHUTDOWN"],
    ["Saat tata letak lantai berubah", "Peta", "Engineer memperbarui peta (dynamic area atau Re-map area, Bagian 3)"],
], [0.17, 0.22, 0.61]))

A(("h1", "Troubleshooting", True))
A(("fig", "trouble", None, 1.0))
A(("h2", "Berdasarkan gejala"))
A(("table", ["Gejala", "Kemungkinan penyebab", "Yang dilakukan"], [
    ["Halaman web tidak terbuka", "Tablet tidak di Wi-Fi AMR; AMR mati atau masih start; alamat salah", "Cek Wi-Fi dan alamat di sampul; tunggu 1 menit setelah menyala; reload (tombol ↻ kanan atas)"],
    ["Halaman menunjukkan nilai lama / tile abu-abu (STALE)", "Wi-Fi lemah atau putus", "Mendekat, reload. AMR sendiri tetap bekerja"],
    ["Selector STALE / INVALID", "Modul I/O panel (192.168.1.40) tidak menjawab", "Cek power dan kabel jaringannya. AMR tetap MANUAL dengan rem aktif sampai modul kembali"],
    ["Drives OFF saat MANUAL", "E-stop ditekan, push mode aktif, modul I/O mati, CAN putus", "Lepas E-stop; halaman Manual Brakes ON; cek halaman Alarms"],
    ["Drives FAULT \"stalled\"", "Roda diperintah tapi tidak berputar 2 detik: roda terhalang, scanner stop saat tombol ditahan, driver mati", "Hilangkan penyebab; teknisi menghapus fault (Bab 18)"],
    ["Drives FAULT \"AGAINST the command\"", "Roda berputar ke arah yang salah (kabel atau setting)", "<b>Jangan dijalankan.</b> Panggil engineer"],
    ["Drives FAULT \"encoder feedback lost\" / CAN_DOWN", "Kabel encoder atau adaptor USB-CAN tercabut", "Cek adaptor USB-CAN dan kabel encoder; lalu fault bisa dihapus"],
    ["Scanner SILENT / STALE", "Kabel atau power scanner; jaringan scanner (192.168.3.2)", "AMR masih bisa dijalankan manual; tidak ada misi sampai diperbaiki"],
    ["Berhenti padahal tidak ada apa-apa di depan", "Jendela scanner kotor, pantulan, benda rendah di field", "Bersihkan jendela; cek lantai di depan; panggil engineer kalau berulang di satu tempat"],
    ["Posisi sering hilang di satu tempat", "Lantai di sana berubah (troli, palet, mesin baru)", "Lihat Alarms → Localisation losses → Show on map; engineer menandai dynamic area atau re-map (Bagian 3)"],
    ["Bergoyang kiri-kanan di lurusan", "Sedikit benda tetap yang terlihat (dynamic area); caster aus", "Laporkan tempatnya; engineer memeriksa peta dan caster"],
    ["Start refused \"... from the route start\"", "AMR tidak di homepost (lebih dari 0,20 m / 10°)", "STOP (MANUAL), jalankan ke tanda, START dua kali"],
    ["\"localisation not READY\"", "Posisi belum di-set atau belum dikonfirmasi", "Bab 5"],
    ["Keypad tidak berfungsi", "Dongle tercabut; baterai keypad; E keypad ditekan", "Colok dongle; baterai; tekan ENTER untuk melepas stop keypad"],
    ["Home: \"The vehicle lost power last time\"", "Power diputus saat software berjalan", "Matikan lalu nyalakan lagi driver"],
    ["Home: pesan \"Needs service\"", "Sebagian software berhenti", "Tekan <b>Recover</b> atau <b>Restart</b> di halaman Home sesuai petunjuknya. Tunggu 1 menit"],
], [0.26, 0.37, 0.37]))

A(("h1", "Pesan alarm", True))
A(("p", "Setiap alarm punya kode (misalnya <b>LOC_JUMP</b>). Halaman Alarms menunjukkan kode, judul dan apa yang harus dilakukan. Yang paling sering:"))
A(("table", ["Kode", "Arti", "Lakukan"], [
    ["ESTOP", "E-stop ditekan", "Amankan, lepas E-stop. (Tidak ada tombol Reset di AMR ini)"],
    ["ESTOP_HOLD", "Misi ditahan oleh E-stop", "Area aman, lalu START atau Continue"],
    ["FIELD_PROTECTIVE / FIELD_BLOCKED", "Ada sesuatu di field merah scanner", "Singkirkan; lanjut sendiri"],
    ["FIELD_WARNING", "Ada sesuatu di field kuning: setengah kecepatan", "Tidak perlu apa-apa; kecepatan penuh kembali kalau sudah aman"],
    ["PATH_BLOCKED", "Jalur di depan terhalang", "Bersihkan route; AMR mencoba lagi sendiri"],
    ["RUN_WAITING", "Menunggu di titik tunggu", "Kerjakan tugasnya, Continue atau START"],
    ["CONTINUE_REFUSED", "Continue tidak menjalankan AMR", "Perbaiki sesuai detailnya, Continue lagi"],
    ["LOC_CORRECTED", "Posisi dikoreksi, AMR tetap berjalan", "Tidak perlu apa-apa. Sering di tempat yang sama → lapor engineer"],
    ["LOC_RELOCALIZING / LOC_RELOCALIZED", "Posisi hilang saat misi / ditemukan lagi", "Menjauh; AMR lanjut sendiri"],
    ["LOC_RELOC_FAILED", "Posisi tidak bisa ditemukan lagi", "Set posisi manual, Confirm, Ack fault, Continue route from here"],
    ["LOC_JUMP / LOC_LOST / LOC_SCAN_MISMATCH / LOC_COV_GREW", "Posisi hilang atau tidak yakin", "Set posisi lagi (Bab 5)"],
    ["LOC_NOT_SET / LOC_NOT_CONFIRMED", "Posisi belum diberikan / belum dikonfirmasi", "Bab 5"],
    ["EXEC_OFF_PATH / EXEC_ENDPOINT_MISSED / EXEC_TURN_FAILED / EXEC_OVERSHOOT", "AMR tidak mengikuti route dalam batasnya", "Ack fault; cek halangan atau lantai licin; Continue route from here"],
    ["EXEC_PREREQ_LOST", "Sensor atau panel putus saat misi", "Ack fault, lihat halaman Alarms"],
    ["DRIVES_NOT_READY", "Motor tidak bertenaga", "Lepas E-stop, Brakes ON; saat AUTO tekan START"],
    ["UNCLEAN_SHUTDOWN", "Power diputus terakhir kali", "Matikan lalu nyalakan lagi driver"],
    ["AUTO_LOAD_REFUSED", "Misi terakhir tidak bisa dimuat otomatis (peta lain aktif)", "Muat misi di halaman Run"],
    ["DEST_REFUSED", "Tombol POS tidak menjalankan misi", "Baca detailnya: MANUAL, misi sedang berjalan, tidak di homepost ..."],
    ["SCANNER_SILENT / SCANNER_STALE", "Tidak ada data dari scanner safety", "Hanya jalan manual; panggil engineer"],
    ["PANEL_STALE", "I/O panel tidak menjawab", "Cek modul I/O; panggil engineer"],
    ["CAN_DOWN", "Encoder motor tidak bisa dihubungi", "Cek adaptor USB-CAN; panggil engineer"],
    ["DISK_LOW / DISK_FULL", "Penyimpanan hampir penuh", "Panggil engineer"],
    ["Needs service: ... (BASE_EXITED, SUPERVISOR_DOWN, WEB_DOWN ...)", "Bagian software berhenti", "Recover / Restart di halaman Home; kalau berulang, Save report dan panggil engineer"],
], [0.33, 0.33, 0.34]))
A(("note", "Pesan yang menyebut \"blue Reset button on the cabinet\", \"pendant enable switch\" atau \"safety reset\" berasal dari "
           "tipe kendaraan lain. Di AMR ini: lepas E-stop; saat MANUAL, STOP meng-acknowledge fault."))

A(("h1", "Masalah posisi dan perubahan peta", True))
A(("p", "Setiap kali AMR kehilangan atau mengoreksi posisinya, AMR menulis laporan. <b>Alarms → Localisation losses</b> menampilkannya, "
        "yang terbaru di atas, dengan kemungkinan penyebabnya. <b>Show on map</b> menggambar kejadiannya: titik hijau = scanner di dinding peta, "
        "merah = di benda yang tidak ada di peta, oranye = menembus dinding peta (sesuatu yang dipetakan sudah tidak ada)."))
A(("table", ["Laporan menulis", "Arti", "Siapa / apa"], [
    ["Banyak % \"on things not on the map\"", "Troli, palet, benda baru", "Engineer: tandai area sebagai dynamic, atau re-map kalau bendanya menetap"],
    ["Banyak % \"through mapped walls\"", "Sesuatu yang dipetakan sudah hilang", "Engineer: re-map area itu"],
    ["Koreksi searah jalur, scan cocok", "Drift odometri di lorong panjang - normal kalau kecil", "Tidak perlu apa-apa, kecuali sangat sering"],
    ["Sensor diam (went quiet)", "Kabel atau jaringan sensor itu", "Teknisi: cek kabel sensor tersebut"],
    ["Kecepatan di laporan jauh di atas kecepatan route (0,7 m/s)", "Masalah pengukuran roda, bukan kecepatan sebenarnya", "Engineer: cek encoder (Bab 18)"],
], [0.32, 0.34, 0.34]))
A(("tip", "Catat jam dan tempat saat AMR berhenti atau bergoyang. Engineer bisa menemukannya di laporan."))

A(("h1", "Mengumpulkan bukti untuk engineer", True))
A(("steps", [
    "Secepatnya setelah masalah: halaman <b>Alarms</b> → <b>Save report</b>. Bacakan nama report ke engineer (atau foto).",
    "Catat: jam, tempat di lantai, apa yang dilakukan AMR, apa yang tertulis di layar.",
    "Foto layar dan foto lantai di tempat kejadian sangat membantu.",
    "Kalau bisa, jangan matikan AMR sebelum report disimpan.",
]))

# ---------------------------------------------------------------- BAGIAN 3
A(("part", "3", "Tugas engineer", "Peta, route, update dan backup - perlu PIN engineer"))

A(("h1", "Tampilan engineer", True))
A(("p", "Tekan <b>Op</b> di header halaman dan ketik PIN engineer. Halaman tambahan muncul: <b>Maps</b> dan <b>Routes</b> (di bawah Run), "
        "<b>Monitor</b>, <b>I/O</b>, <b>Params</b>, <b>Blind run</b>. Tekan <b>Eng</b> untuk kembali ke tampilan operator. PIN disimpan di robot "
        "dalam <b>~/.amr/web.json</b>; berikan hanya ke orang yang sudah dilatih. Panduan engineer yang lengkap ada di <b>OPERATE_AMR_QR.md</b> dan "
        "<b>RUNBOOK.md</b> di folder software."))

A(("h1", "Peta", True))
A(("h2", "Membuat peta baru"))
A(("steps", [
    "Tempel tanda panah di lantai (survey mark). Parkir AMR di atasnya, menghadap panah, diam.",
    "Halaman Maps: Map id, deskripsi tanda, <b>New map (start survey)</b>. Mode menjadi MAPPING.",
    "Jalankan pelan-pelan melewati setiap lorong yang akan dipakai route (jog web, keypad, atau push mode). Hindari putaran cepat; kembali melewati area yang sudah dipetakan.",
    "Kembali ke tanda yang sama, arah yang sama, <b>Returned to start</b>. Selisih penutupan (closure) sebaiknya beberapa cm dan 1-2°.",
    "<b>Save map revision</b>.",
]))
A(("h2", "Menjaga peta tetap sesuai"))
A(("table", ["Perubahan di lantai", "Tool (Maps → Edit areas)"], [
    ["Benda yang datang dan pergi (troli, palet)", "Kotak <b>Dynamic area</b> - lokalisasi mengabaikannya. Biarkan dinding/mesin tetap di luar kotak"],
    ["Titik noise", "<b>Remove noise</b>"],
    ["Halangan baru yang permanen", "<b>Static obstacle</b>"],
    ["Satu bagian benar-benar berubah (mesin dipindah, rak hilang)", "<b>Re-map area</b>: gambar Re-map box, Continue mapping melewati bagian itu, lalu Preview dan Save di revisi baru"],
    ["Peta miring", "<b>Rotate map</b> dengan reference line di sepanjang dinding panjang"],
    ["Menambah area baru", "<b>Continue mapping</b> dari revisi yang dipakai"],
], [0.4, 0.6]))
A(("p", "Edit areas juga menampilkan <b>Changes seen during runs</b>: merah = ada di peta tapi hilang, kuning = ada sekarang tapi tidak di peta, dengan saran "
        "per area. Setelah setiap revisi baru: di Routes buka setiap route di revisi itu, <b>Validate</b>, <b>Save revision</b>, <b>Create mission</b>, lalu pakai "
        "revisi baru di halaman Run."))

A(("h1", "Route, misi dan POS", True))
A(("bul", [
    "Halaman <b>Routes</b>: pilih revisi peta, ketik route id, set start pose (homepost: survey mark, atau <b>Start = AMR here</b>).",
    "Tambahkan langkah: lurus, putar (45/90/180/270), arc (radius ≥ 1 m), mundur pendek, <b>Wait</b> (titik berhenti bernama), <b>Go home</b> (langkah terakhir).",
    "Kecepatan: lurusan 0,70 m/s; belokan 0,40 m/s kecuali belokan punya kotak kecepatan sendiri; slow zone untuk pintu, persimpangan dan jalur pejalan kaki.",
    "<b>Validate</b> (merah = AMR akan menyentuh dinding), <b>Save revision</b>, <b>Create mission from saved revision</b>.",
    "Route POS: gambar main route sekali; untuk setiap POS pakai <b>Branch → Stop point on map</b>; route id pos1 ... pos4 (dipakai tombol POS dan F1-F4). Jumlah POS diatur di profil (panel.destinations).",
    "Hapus peta, route atau misi dengan tombol Remove; tidak ada yang benar-benar terhapus - dipindah ke <b>~/amr_maps/.trash</b>.",
]))
A(("warn", "Pada 0,70 m/s AMR butuh sekitar 0,6 m untuk berhenti. Pastikan field merah scanner cukup panjang (sekitar 1,0 m, kuning sekitar 1,8 m) "
           "dan uji jarak berhenti terhadap sebuah kotak setiap kali kecepatan atau field diubah."))

A(("h1", "Update software, restart dan backup", True))
A(("h2", "Update software"))
A(("code", "sudo systemctl stop amr.service\ncd ~/agv_can && git pull\ncd amr_ws && source /opt/ros/jazzy/setup.bash\n"
           "colcon build --packages-select <paket yang berubah>\nsudo systemctl start amr.service"))
A(("p", "Lalu reload halaman web dengan tombol ↻ (atau Ctrl+Shift+R)."))
A(("h2", "Backup peta (GitHub, repository private amr-maps)"))
A(("code", "cd ~/amr_maps && git add -A && git commit -m \"maps $(date +%F)\" && git push"))
A(("p", "Lakukan setiap ada revisi peta, route atau misi baru. Untuk mengembalikan di PC baru: hentikan service, "
        "<b>git clone</b> repository ke ~/amr_maps."))
A(("h2", "Restart dan log"))
A(("table", ["Tugas", "Perintah / tempat"], [
    ["Status service / log langsung", "systemctl status amr.service · journalctl -u amr.service -f"],
    ["Restart software", "Restart di halaman Home, atau sudo systemctl restart amr.service"],
    ["Log", "~/.amr/logs/base.log (driver, panel), layer.log (peta, navigasi), events.jsonl (riwayat kejadian)"],
    ["Report (Save report)", "~/.amr/reports"],
    ["Laporan kehilangan lokalisasi", "~/.amr/loc_losses"],
    ["Setting kendaraan ini", "~/agv_can/profiles/amr-qr-01.json, ~/agv_can/amr_ws/deploy/amr.env"],
], [0.32, 0.68]))

A(("h1", "Tool dan perbaikan yang berguna", True))
A(("table", ["Masalah", "Tool / perintah"], [
    ["Menghapus fault driver yang terkunci (stalled, encoder lost)", "ros2 service call /drives/ack_fault std_srvs/srv/Trigger"],
    ["Merekam jalan untuk analisis (kirim foldernya ke engineer)", "~/agv_can/tools/run_log.sh, jalankan, Ctrl-C"],
    ["Apakah AMR menarik ke satu sisi?", "python3 ~/agv_can/tools/straight_run.py"],
    ["Apakah panjang peta sudah benar?", "python3 ~/agv_can/tools/map_scale.py"],
    ["Cek roda vs lidar", "python3 ~/agv_can/tools/scan_odom_sync.py"],
    ["Keypad tidak terdeteksi", "ros2 run amr_base qr_keypad --list (service dihentikan)"],
    ["Tes bench I/O, encoder, IMU", "ros2 run amr_base qr_calibrate io | enc | imu (service dihentikan)"],
], [0.45, 0.55]))
A(("p", "Sebelum menjalankan tool secara manual: <b>source ~/agv_can/amr_ws/install/setup.bash</b> dan <b>source ~/agv_can/amr_ws/env/vehicle.sh</b>. "
        "Tool yang menggerakkan motor butuh service dihentikan dan AMR di atas blok atau di area yang aman."))

A(("h1", "Kartu ringkas", True))
A(("table", ["", ""], [
    ["Menjalankan misi", "Posisi READY · AMR di homepost · START (AUTO) · START (jalan)"],
    ["Ke POS", "AUTO · Go to POS n (web) atau F1-F4 · di POS: Continue / START"],
    ["Berhenti sekarang", "STOP (batal, MANUAL) · E-stop (ditahan; lepas + START untuk lanjut)"],
    ["Berhenti karena benda di depan", "Singkirkan - lanjut sendiri"],
    ["WAITING", "Kerjakan tugas, menjauh, Continue / START"],
    ["FAULT", "Baca alasan · perbaiki · Ack fault · Continue route from here · START"],
    ["Posisi hilang", "Halaman Run: set initial pose · cek titik di dinding · Confirm"],
    ["Drives OFF", "E-stop sudah dilepas? push mode mati (Brakes ON)?"],
    ["Masalah aneh", "Alarms → Save report · catat jam dan tempat · panggil engineer"],
    ["Akhir shift", "STOP · Return to idle · parkir · matikan PC · power off"],
], [0.3, 0.7], False))
