# Radar Simulator with Menu Navigation using ESP32

Proyek ini adalah simulator radar yang dibangun menggunakan dua modul ESP32 (ESP1 dan ESP2). ESP1 bertanggung jawab atas simulasi radar, logging data ke SD Card, playback CSV, komunikasi CAN bus, dan pengiriman data ke ESP2 via UART. ESP2 menangani antarmuka pengguna (UI) melalui LCD I2C, tombol navigasi, display 7-segment untuk RPM, dan pengiriman data RPM/speed ke CAN bus atau pulse.

Sistem ini mendukung mode Manual, Auto, Record, dan Play, dengan dukungan untuk berbagai unit truck (seperti HINO, HOWO, dll.) melalui konfigurasi CAN atau pulse.

## Deskripsi

- **ESP1.ino**: Menangani:
  - Pembacaan sensor ADC untuk speed, distance, dan velocity.
  - Simulasi mode Auto (Pre-Collision, Soft Brake, Full Brake, Full Scenario).
  - Logging data ke SD Card dalam format CSV.
  - Playback data dari CSV untuk mode Play.
  - Pengiriman pesan CAN untuk simulasi radar.
  - Komunikasi UART dengan ESP2 untuk kontrol menu dan sinkronisasi data.

- **ESP2.ino**: Menangani:
  - UI menu navigasi menggunakan LCD I2C dan tombol.
  - Pemilihan unit truck dan mode simulasi.
  - Display RPM pada 7-segment.
  - Pengiriman data speed/RPM ke CAN bus berdasarkan unit yang dipilih.
  - Integrasi dengan ESP1 via UART untuk update data real-time.

Proyek ini menggunakan RTOS (FreeRTOS) untuk multitasking, seperti pembacaan sensor, pengiriman CAN, dan update display.

## Fitur Utama

- **Mode Operasi**:
  - Manual: Kontrol manual via potensiometer ADC.
  - Auto: Simulasi otomatis dengan jarak dan velocity tetap berdasarkan sub-mode (Pre-Collision, Soft Brake, Full Brake, Full Scenario).
  - Record: Logging data ke SD Card dalam format CSV (speed, RPM, distance, velocity, RCS).
  - Play: Playback data CSV dari SD Card dengan timer countdown.

- **Dukungan Unit Truck**: 18 unit berbeda (HINO, HOWO, HANVAN, dll.) dengan konfigurasi CAN baudrate, ID, atau mode pulse/hybrid.

- **Integrasi SD Card**: Inisialisasi, monitoring, logging CSV, dan scanning file untuk playback.

- **UI LCD**: Menu navigasi dengan tombol SELECT/BACK, UP/DOWN. Tampilan timer untuk Record/Play.

- **Display 7-Segment**: Untuk speed, distance, velocity (ESP1) dan RPM (ESP2), dengan mode blinking saat idle.

- **Komunikasi**:
  - UART2 antara ESP1 dan ESP2 untuk perintah dan data (e.g., mode, RPM, speed).
  - CAN bus untuk simulasi data radar dan kendaraan.

- **Filtering Data**: Kalman filter untuk smoothing data ADC.

- **Error Handling**: Deteksi SD card hilang, inisialisasi gagal, dan konfirmasi save/stop recording.

## Persyaratan Hardware

- 2x ESP32 DevKit (atau board serupa).
- LCD I2C 16x2 (alamat 0x27).
- 3x TM1637 7-Segment Display (untuk speed, distance, velocity di ESP1; RPM di ESP2).
- SD Card Module (SPI: SCK=14, MOSI=15, MISO=2, CS=13).
- Potensiometer untuk input ADC (speed, distance, velocity, RPM).
- Tombol push (SELECT, BACK, UP, DOWN).
- Transceiver CAN (seperti SN65HVD230) untuk CAN bus.
- SD Card (FAT32 formatted).

Pinout detail ada di kode (lihat komentar `#define`).

## Instalasi

1. Instal Arduino IDE dan tambahkan board ESP32 via Boards Manager.
2. Instal library yang diperlukan:
   - `TM1637Display` (untuk 7-segment).
   - `LiquidCrystal_I2C` (untuk LCD).
   - `SD` (built-in untuk SD Card).
   - `SPI` (built-in).
   - `Wire` (built-in untuk I2C).
   - `driver/twai.h` (built-in ESP32 untuk CAN).

3. Buka file `ESP1.ino` dan `ESP2.ino` di Arduino IDE.
4. Upload `ESP1.ino` ke ESP32 pertama (ESP1).
5. Upload `ESP2.ino` ke ESP32 kedua (ESP2).
6. Hubungkan UART2: TX1 ESP1 ke RX2 ESP2, RX1 ESP1 ke TX2 ESP2.
7. Hubungkan GND bersama.

## Penggunaan

1. Nyalakan kedua ESP32.
2. ESP2 akan menampilkan menu utama: Manual, Auto, Rec, Play.
3. Gunakan tombol UP/DOWN untuk navigasi, SELECT untuk pilih, BACK untuk kembali.
4. Pilih unit truck saat diminta.
5. Di mode Auto: Pilih sub-mode simulasi.
6. Di mode Rec: Pilih unit, mode, lalu start recording (data disimpan ke SD sebagai `PC-XX.csv`, dll.).
7. Di mode Play: Pilih mode, lalu file CSV untuk playback.
8. Monitor data via Serial Monitor (baud 115200) untuk debugging.

Catatan: Pastikan SD Card terpasang sebelum masuk mode Rec/Play.

## Kontribusi

Kontribusi dipersilakan! Silakan fork repository ini, buat branch baru, dan submit pull request. Pastikan kode sesuai dengan gaya yang ada.

## Lisensi

Proyek ini dilisensikan di bawah [MIT License](LICENSE). Lihat file LICENSE untuk detail.

## Kontak

Jika ada pertanyaan, buka issue di GitHub atau hubungi [nama Anda atau email].
